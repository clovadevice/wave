/*
 * TAS5731 ASoC codec driver
 *
 * Copyright (c) 2013 Daniel Mack <zonque@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * TODO:
 *  - implement DAPM and input muxing
 *  - implement modulation limit
 *  - implement non-default PWM start
 *
 * Note that this chip has a very unusual register layout, specifically
 * because the registers are of unequal size, and multi-byte registers
 * require bulk writes to take effect. Regmap does not support that kind
 * of devices.
 *
 * Currently, the driver does not touch any of the registers >= 0x20, so
 * it doesn't matter because the entire map can be accessed as 8-bit
 * array. In case more features will be added in the future
 * that require access to higher registers, the entire regmap H/W I/O
 * routines have to be open-coded.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/qdsp6v2/apr.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/tas5731.h>
#include <sound/apr_audio-v2.h>
#include <sound/q6afe-v2.h>
#include "../msm/msm-audio-pinctrl.h"

#define ENABLE_MCLK1
#define RUNTIME_SUSPEND_RESUME
#define TAS5731_PCM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE  |     \
				 SNDRV_PCM_FMTBIT_S20_3LE |     \
				 SNDRV_PCM_FMTBIT_S24_3LE)

#define TAS5731_PCM_RATES   (SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100  | \
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200  | \
				 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
				 SNDRV_PCM_RATE_192000)

/*
 * TAS5731 registers
 */
#define TAS5731_REG_CLOCK_CONTROL   0x00    /* Clock control register  */
#define TAS5731_CLOCK_RATE(val)     (val << 5)
#define TAS5731_CLOCK_RATE_MASK     (0x7 << 5)
#define TAS5731_CLOCK_RATIO(val)    (val << 2)
#define TAS5731_CLOCK_RATIO_MASK    (0x7 << 2)
#define TAS5731_CLOCK_SCLK_RATIO_48 (1 << 1)
#define TAS5731_CLOCK_VALID     (1 << 0)

#define TAS5731_DEEMPH_MASK     0x03
#define TAS5731_SOFT_MUTE_ALL       0x3f

#define TAS5731_REG_DEV_ID      0x01    /* Device ID register */
#define TAS5731_REG_ERROR_STATUS    0x02    /* Error status register */
#define TAS5731_REG_SYS_CONTROL_1   0x03    /* System control register 1 */
#define TAS5731_REG_SERIAL_DATA_IF  0x04    /* Serial data interface register  */
#define TAS5731_REG_SYS_CONTROL_2   0x05    /* System control register 2 */
#define TAS5731_REG_SOFT_MUTE       0x06    /* Soft mute register */
#define TAS5731_REG_MASTER_VOL      0x07    /* Master volume  */
#define TAS5731_REG_CHANNEL_VOL(X)  (0x08 + (X))    /* Channel 1-6 volume */
#define TAS5731_REG_MOD_LIMIT       0x10    /* Modulation limit register */
#define TAS5731_REG_OSC_TRIM        0x1b    /* Oscillator trim register */
#define TAS5731_REG_BKNDERR         0x1c

/*
 * Default TAS5731 power-save configuration
 */
static void tas5731_power_save(struct work_struct *work_ptr);
#define TAS5731_POWER_SAVE_DELAY (HZ * 5) // 5 seconds
static struct power_save_work {
	struct tas5731_private *priv;
	struct delayed_work work;
}  tas5731_power_save_work;


/*
 * Default TAS5731 power-up configuration
 */
static struct afe_clk_set mi2s_rx_clk = {
	AFE_API_VERSION_I2S_CONFIG,
	Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT,
	Q6AFE_LPASS_IBIT_CLK_1_P536_MHZ,
	Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_NO,
	Q6AFE_LPASS_CLK_ROOT_DEFAULT,
	0,
};

struct cdc_pdm_pinctrl_info {
		struct pinctrl *pinctrl;
		struct pinctrl_state *cdc_lines_sus;
		struct pinctrl_state *cdc_lines_act;
		struct pinctrl_state *cross_conn_det_sus;
		struct pinctrl_state *cross_conn_det_act;
};

static struct cdc_pdm_pinctrl_info pinctrl_info;

static bool tas5731_accessible_reg(struct device *dev, unsigned int reg)
{
	pr_info("%s entered\n", __func__);
	return !((reg == 0x0f) || (reg >= 0x0b && reg <= 0x0d) || (reg >= 0x15 && reg <= 0x18));
}

static bool tas5731_volatile_reg(struct device *dev, unsigned int reg)
{
	pr_info("%s entered\n", __func__);
	switch (reg) {
	case TAS5731_REG_DEV_ID:
	case TAS5731_REG_ERROR_STATUS:
		return true;
	}

	return false;
}

static bool tas5731_writeable_reg(struct device *dev, unsigned int reg)
{
	pr_info("%s entered\n", __func__);
	return tas5731_accessible_reg(dev, reg) && (reg != TAS5731_REG_DEV_ID);
}


struct tas5731_private {
	struct i2c_client *i2c;
	struct regmap   *regmap;
	struct clk      *tas_ext_clk;
#ifdef RUNTIME_SUSPEND_RESUME
	struct regulator *vdd_18v;
#endif
	unsigned int    mclk, sclk;
	unsigned int    format;
	/* Current sample rate for de-emphasis control */
	int     rate;
	/* GPIO driving Reset pin, if any */
	int     gpio_nreset;
	int     gpio_npdn;
	int     master_vol;
};

static int tas5731_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct tas5731_private *priv = snd_soc_codec_get_drvdata(codec);

	pr_info("%s entered\n", __func__);
	switch (clk_id) {
	case TAS5731_CLK_IDX_MCLK:
		priv->mclk = freq;
		break;
	case TAS5731_CLK_IDX_SCLK:
		priv->sclk = freq;
		break;
	}

	return 0;
}

static int tas5731_set_dai_fmt(struct snd_soc_dai *codec_dai,
				   unsigned int format)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct tas5731_private *priv = snd_soc_codec_get_drvdata(codec);

	pr_info("%s entered\n", __func__);
	/* The TAS5731 can only be slave to all clocks */
	if ((format & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS) {
		dev_err(codec->dev, "Invalid clocking mode\n");
		return -EINVAL;
	}

	/* we need to refer to the data format from hw_params() */
	priv->format = format;

	return 0;
}

static int tas5731_i2c_write(struct i2c_client *client, char *buf, int len)
{
	int ret = 0;

	ret = i2c_master_send(client, buf, len);
	if (ret < 0)
		dev_err(&client->dev, "Error %d writing to :0x%x\n", ret, buf[0]);

	return ret;
}

static int tas5731_i2c_read(struct i2c_client *client, char addr, char *buf, int len)
{
	int ret = 0;
	
	ret = i2c_master_send(client, &addr, sizeof(addr));
	if (ret < 0)
		goto fail;
	
	ret = i2c_master_recv(client, buf, len);
	if (ret < 0)
		goto fail;
	
	return ret;

fail:
	dev_err(&client->dev, "Error %d reading from :0x%x\n", ret, addr);
	return ret;
}

#define I2C_CMD1(reg, addr01) \
i2c_cmd1[0] = reg; \
i2c_cmd1[1] = addr01; \
tas5731_i2c_write(client, i2c_cmd1, sizeof(i2c_cmd1))


#define I2C_CMD4(reg, addr01, addr02, addr03, addr04) \
i2c_cmd4[0] = reg; \
i2c_cmd4[1] = addr01; i2c_cmd4[2] = addr02; \
i2c_cmd4[3] = addr03; i2c_cmd4[4] = addr04; \
tas5731_i2c_write(client, i2c_cmd4, sizeof(i2c_cmd4))


#define I2C_CMD8(reg, addr01, addr02, addr03, addr04, \
				addr05, addr06, addr07, addr08) \
i2c_cmd8[0] = reg; \
i2c_cmd8[1] = addr01; i2c_cmd8[2] = addr02; \
i2c_cmd8[3] = addr03; i2c_cmd8[4] = addr04; \
i2c_cmd8[5] = addr05; i2c_cmd8[6] = addr06; \
i2c_cmd8[7] = addr07; i2c_cmd8[8] = addr08; \
tas5731_i2c_write(client, i2c_cmd8, sizeof(i2c_cmd8))


#define I2C_CMD12(reg, addr01, addr02, addr03, addr04, \
					addr05, addr06, addr07, addr08, \
					addr09, addr10, addr11, addr12) \
i2c_cmd12[0] = reg; \
i2c_cmd12[1] = addr01; i2c_cmd12[2] = addr02; \
i2c_cmd12[3] = addr03; i2c_cmd12[4] = addr04; \
i2c_cmd12[5] = addr05; i2c_cmd12[6] = addr06; \
i2c_cmd12[7] = addr07; i2c_cmd12[8] = addr08; \
i2c_cmd12[9] = addr09; i2c_cmd12[10] = addr10;\
i2c_cmd12[11] = addr11; i2c_cmd12[12] = addr12; \
tas5731_i2c_write(client, i2c_cmd12, sizeof(i2c_cmd12))


#define I2C_CMD16(reg, addr01, addr02, addr03, addr04, \
					addr05, addr06, addr07, addr08, \
					addr09, addr10, addr11, addr12, \
					addr13, addr14, addr15, addr16) \
i2c_cmd16[0] = reg; \
i2c_cmd16[1] = addr01; i2c_cmd16[2] = addr02; \
i2c_cmd16[3] = addr03; i2c_cmd16[4] = addr04; \
i2c_cmd16[5] = addr05; i2c_cmd16[6] = addr06; \
i2c_cmd16[7] = addr07; i2c_cmd16[8] = addr08; \
i2c_cmd16[9] = addr09; i2c_cmd16[10] = addr10; \
i2c_cmd16[11] = addr11; i2c_cmd16[12] = addr12; \
i2c_cmd16[13] = addr13; i2c_cmd16[14] = addr14; \
i2c_cmd16[15] = addr15; i2c_cmd16[16] = addr16; \
tas5731_i2c_write(client, i2c_cmd16, sizeof(i2c_cmd16))

#define I2C_CMD20(reg, addr01, addr02, addr03, addr04, \
					addr05, addr06, addr07, addr08, \
					addr09, addr10, addr11, addr12, \
					addr13, addr14, addr15, addr16, \
					addr17, addr18, addr19, addr20) \
i2c_cmd20[0] = reg; \
i2c_cmd20[1] = addr01; i2c_cmd20[2] = addr02; \
i2c_cmd20[3] = addr03; i2c_cmd20[4] = addr04; \
i2c_cmd20[5] = addr05; i2c_cmd20[6] = addr06; \
i2c_cmd20[7] = addr07; i2c_cmd20[8] = addr08; \
i2c_cmd20[9] = addr09; i2c_cmd20[10] = addr10; \
i2c_cmd20[11] = addr11; i2c_cmd20[12] = addr12; \
i2c_cmd20[13] = addr13; i2c_cmd20[14] = addr14; \
i2c_cmd20[15] = addr15; i2c_cmd20[16] = addr16; \
i2c_cmd20[17] = addr17; i2c_cmd20[18] = addr18; \
i2c_cmd20[19] = addr19; i2c_cmd20[20] = addr20; \
tas5731_i2c_write(client, i2c_cmd20, sizeof(i2c_cmd20))

static int tas5731_init(struct i2c_client *client)
{
	pr_info("%s entered\n", __func__);

	unsigned char i2c_cmd1[2] = {0};
	unsigned char i2c_cmd4[5] = {0};
	unsigned char i2c_cmd8[9] = {0};
	unsigned char i2c_cmd12[13] = {0};
	unsigned char i2c_cmd16[17] = {0};
	unsigned char i2c_cmd20[21] = {0};

	// I2C Configuration file for TAS570x
	I2C_CMD1(0x1B, 0x00);
	msleep(50);
	I2C_CMD1(0x06, 0x00);
	I2C_CMD1(0x0A, 0x3C);
	I2C_CMD1(0x09, 0x28);
	I2C_CMD1(0x08, 0x36);
	I2C_CMD1(0x14, 0x54);
	I2C_CMD1(0x13, 0xAC);
	I2C_CMD1(0x12, 0x54);
	I2C_CMD1(0x11, 0xAC);
	// Connect Subwoofer to 0x0A
	I2C_CMD1(0x0E, 0xD1);
	//
	I2C_CMD4(0x20, 0x00, 0x01, 0x77, 0x72);
	I2C_CMD1(0x10, 0x02);
	I2C_CMD1(0x0B, 0x00);
	I2C_CMD1(0x10, 0x02);
	I2C_CMD1(0x1C, 0x02);
	I2C_CMD1(0x19, 0x30);
	I2C_CMD4(0x25, 0x01, 0x02, 0x13, 0x45);
	// Biquads
	I2C_CMD4(0x50, 0x00, 0x00, 0x00, 0x00);
	I2C_CMD20(0x29, 0x00, 0x7F, 0xBF, 0xC9, 0x0F, 0x80, 0x40, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x7F, 0x92, 0x00, 0x00, 0x00, 0x00);
	I2C_CMD20(0x30, 0x00, 0x7F, 0xBF, 0xC9, 0x0F, 0x80, 0x40, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x7F, 0x92, 0x00, 0x00, 0x00, 0x00);
	I2C_CMD20(0x2A, 0x00, 0x79, 0xC5, 0xB5, 0x0F, 0x15, 0xDD, 0x42, 0x00, 0x75, 0x98, 0x1A, 0x00, 0xEA, 0x22, 0xBE, 0x0F, 0x90, 0xA2, 0x30);
	I2C_CMD20(0x2B, 0x00, 0x79, 0x8B, 0xFF, 0x0F, 0x22, 0x54, 0x26, 0x00, 0x70, 0x22, 0xBB, 0x00, 0xDD, 0xAB, 0xDA, 0x0F, 0x96, 0x51, 0x45);
	I2C_CMD20(0x2C, 0x00, 0x7C, 0x8E, 0xD0, 0x0F, 0x2E, 0x65, 0xAE, 0x00, 0x6E, 0xAF, 0x53, 0x00, 0xD1, 0x9A, 0x52, 0x0F, 0x94, 0xC1, 0xDC);
	I2C_CMD20(0x2D, 0x00, 0x78, 0x21, 0xC3, 0x0F, 0x4F, 0xB0, 0x41, 0x00, 0x63, 0xE9, 0xA2, 0x00, 0xB0, 0x4F, 0xBF, 0x0F, 0xA3, 0xF4, 0x9B);
	I2C_CMD20(0x2E, 0x00, 0x7C, 0x91, 0xE2, 0x0F, 0x5B, 0x97, 0x26, 0x00, 0x6B, 0xF0, 0xBE, 0x00, 0xA4, 0x68, 0xDA, 0x0F, 0x97, 0x7D, 0x60);
	I2C_CMD20(0x2F, 0x00, 0x88, 0x52, 0x2E, 0x00, 0x43, 0x67, 0x2B, 0x00, 0x1D, 0x53, 0x7D, 0x0F, 0xB4, 0xCE, 0x56, 0x0F, 0xE2, 0x24, 0xD3);
	I2C_CMD20(0x58, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	I2C_CMD20(0x59, 0x00, 0x6F, 0xCA, 0x83, 0x0F, 0x90, 0x35, 0x7D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5F, 0x95, 0x06, 0x00, 0x00, 0x00, 0x00);
	I2C_CMD20(0x31, 0x00, 0x7F, 0x85, 0x21, 0x0F, 0x02, 0xA8, 0x06, 0x00, 0x7E, 0x8E, 0x37, 0x00, 0xFD, 0x57, 0xFA, 0x0F, 0x81, 0xEC, 0xA8);
	I2C_CMD20(0x32, 0x00, 0x7D, 0xF0, 0x6B, 0x0F, 0x82, 0x0F, 0x95, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7B, 0xE0, 0xD7, 0x00, 0x00, 0x00, 0x00);
	I2C_CMD20(0x33, 0x00, 0x7F, 0xD9, 0x05, 0x0F, 0x01, 0x1E, 0xB0, 0x00, 0x7F, 0x1C, 0x0E, 0x00, 0xFE, 0xE1, 0x50, 0x0F, 0x81, 0x0A, 0xED);
	I2C_CMD20(0x34, 0x00, 0x7F, 0x4F, 0x5E, 0x0F, 0x03, 0x2D, 0x7B, 0x00, 0x7D, 0xBF, 0x96, 0x00, 0xFC, 0xD2, 0x85, 0x0F, 0x82, 0xF1, 0x0C);
	I2C_CMD20(0x35, 0x00, 0x7F, 0xAB, 0x73, 0x0F, 0x02, 0x7D, 0xEB, 0x00, 0x7E, 0x3D, 0x93, 0x00, 0xFD, 0x82, 0x15, 0x0F, 0x82, 0x16, 0xFA);
	I2C_CMD20(0x36, 0x00, 0x6E, 0x5F, 0x4B, 0x0F, 0x3E, 0xEF, 0x91, 0x00, 0x5A, 0xFB, 0xDE, 0x00, 0xC1, 0x10, 0x6F, 0x0F, 0xB6, 0xA4, 0xD7);
	I2C_CMD20(0x5C, 0x00, 0x0C, 0x85, 0x0A, 0x00, 0x0C, 0x85, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0xF5, 0xEB, 0x00, 0x00, 0x00, 0x00);
	I2C_CMD20(0x5D, 0x00, 0x0D, 0x8B, 0xE5, 0x00, 0x0D, 0x8B, 0xE5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0xE8, 0x35, 0x00, 0x00, 0x00, 0x00);
	I2C_CMD20(0x5A, 0x00, 0x7F, 0xF1, 0x39, 0x0F, 0x00, 0x93, 0x45, 0x00, 0x7F, 0x7F, 0x17, 0x00, 0xFF, 0x6C, 0xBB, 0x0F, 0x80, 0x8F, 0xAF);
	I2C_CMD20(0x5B, 0x00, 0x02, 0x0F, 0x94, 0x00, 0x02, 0x0F, 0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7B, 0xE0, 0xD7, 0x00, 0x00, 0x00, 0x00);


	// DRCs
	I2C_CMD8(0x3A, 0x00, 0x00, 0x88, 0x3F, 0x00, 0x7F, 0x77, 0xC0);
	I2C_CMD8(0x3B, 0x00, 0x00, 0x44, 0x32, 0x00, 0x7F, 0xBB, 0xCD);
	I2C_CMD8(0x3C, 0x00, 0x00, 0x44, 0x32, 0x00, 0x7F, 0xBB, 0xCD);
	I2C_CMD4(0x40, 0xFD, 0x57, 0xAB, 0x4C);
	I2C_CMD4(0x41, 0x0F, 0x84, 0x44, 0x44);
	I2C_CMD4(0x42, 0x00, 0x08, 0x42, 0x10);
	I2C_CMD4(0x46, 0x00, 0x00, 0x00, 0x03);
	I2C_CMD8(0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	I2C_CMD8(0x3D, 0x00, 0x00, 0x88, 0x3F, 0x00, 0x7F, 0x77, 0xC0);
	I2C_CMD8(0x3E, 0x00, 0x00, 0x44, 0x32, 0x00, 0x7F, 0xBB, 0xCD);
	I2C_CMD8(0x3F, 0x00, 0x00, 0x44, 0x32, 0x00, 0x7F, 0xBB, 0xCD);
	I2C_CMD4(0x43, 0xFC, 0xC2, 0xD8, 0xC5);
	I2C_CMD4(0x44, 0x0F, 0x84, 0x44, 0x44);
	I2C_CMD4(0x45, 0x00, 0x08, 0x42, 0x10);
	I2C_CMD4(0x46, 0x00, 0x00, 0x00, 0x03);

	// Mixer
	I2C_CMD12(0x52, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00);
	I2C_CMD16(0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00);
	I2C_CMD16(0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00);
	I2C_CMD8(0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00);
	I2C_CMD4(0x56, 0x00, 0x80, 0x00, 0x00);
	I2C_CMD4(0x57, 0x00, 0x02, 0x00, 0x00);
	I2C_CMD12(0x51, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	I2C_CMD12(0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00);
	I2C_CMD12(0x52, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00);
	I2C_CMD8(0x61, 0x00, 0x40, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00);
	I2C_CMD1(0x05, 0x00);
	I2C_CMD1(0x07, 0x2F);

	return 0;
}

#ifdef ENABLE_MCLK1
static int tas5731_enable_mclk(struct clk *tas_ext_clk, bool enable)
{
	int ret = 0;

	/* enable mclk2 */
	if (enable) {
		ret = clk_prepare_enable(tas_ext_clk);
		if (ret) {
			pr_err("%s: ext clk enable failed\n", __func__);
			return -EINVAL;
		} else pr_info("%s: ext clk enabled\n", __func__);
	} else {
		clk_disable_unprepare(tas_ext_clk);
	}

	return ret;
}
#endif

static void tas5731_preserve_master_vol(struct tas5731_private *priv)
{
	int ret = 0;
	int master_vol;

	ret = tas5731_i2c_read(priv->i2c, TAS5731_REG_MASTER_VOL, &master_vol, 1 );
	if (ret == 0) {
		priv->master_vol = master_vol;
		pr_info("%s: preserve master volume 0x%x\n", __func__, priv->master_vol);
	} else {
		pr_err("%s: failed to set preserve master volume\n", __func__);
	}
}

static void tas5731_power_save(struct work_struct *work_ptr)
{
	struct tas5731_private *priv = container_of(work_ptr, struct power_save_work, work)->priv;
	int ret = 0;
	unsigned char buf[2] = {0};
	unsigned char val = 0;

	buf[0] = TAS5731_REG_MASTER_VOL;
	buf[1] = 0xFF;
	ret = tas5731_i2c_write(priv->i2c, buf, 2);

	buf[0] = TAS5731_REG_SYS_CONTROL_2;
	buf[1] = 0x40;
	ret = tas5731_i2c_write(priv->i2c, buf, 2);

	// disable vdd_18v
	if (priv->vdd_18v) {
		ret = regulator_disable(priv->vdd_18v);
		if (ret < 0) {
			pr_err("qci_vdd_18v regulator disable failed\n");
			ret = -1;
		} else pr_info("%s: qci_vdd_18v is disable\n", __func__);
	}  else {
		pr_err("none of qci_vdd_18v regulator\n");
		ret = -1;
	}
	return ret;
}

static int tas5731_prepare(struct snd_pcm_substream *substream,
			 struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas5731_private *priv = snd_soc_codec_get_drvdata(codec);
#ifdef RUNTIME_SUSPEND_RESUME
	int ret = 0;
	char dev_id;
#endif

	pr_info("%s entered\n", __func__);
	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return;

#ifdef RUNTIME_SUSPEND_RESUME
	// enable vdd_18v
	if (priv->vdd_18v) {
		cancel_delayed_work_sync(&tas5731_power_save_work.work);
		ret = regulator_enable(priv->vdd_18v);
		if (ret < 0) {
			pr_err("qci_vdd_18v regulator enable failed\n");
			return ret;
		}
	} else {
		pr_err("none of qci_vdd_18v regulator\n");
		return -1;
	}

	//pull up priv->gpio_npdn to power down devices
	if ((gpio_is_valid(priv->gpio_npdn))&&(gpio_is_valid(priv->gpio_nreset))) {
		pr_info("%s: pull PDN up\n", __func__);
		gpio_set_value(priv->gpio_npdn, 1);
		udelay(200);
		pr_info("%s: pull reset up\n", __func__);
		gpio_set_value(priv->gpio_nreset, 1);

		/* Codec needs ~15ms to wake up, longer than 13.5ms */
		msleep(15);
	}
	msleep(300);

	/* The TAS5731 always returns 0x03 in its TAS5731_REG_DEV_ID register */
	tas5731_i2c_read(priv->i2c, TAS5731_REG_DEV_ID, &dev_id, 1 );
	if (dev_id != 0x00) {
		pr_err("%s: Failed to identify TAS5731 codec (got %02x)\n", __func__, dev_id);
	}

	tas5731_init(priv->i2c);
#endif

#ifdef ENABLE_MCLK1
	tas5731_enable_mclk(priv->tas_ext_clk, true);
#endif

	return 0;
}

static void tas5731_shutdown(struct snd_pcm_substream *substream,
			 struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas5731_private *priv = snd_soc_codec_get_drvdata(codec);
#ifdef RUNTIME_SUSPEND_RESUME
	int err = 0;
#endif

	pr_info("%s entered\n", __func__);
	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return;

#ifdef ENABLE_MCLK1
	tas5731_enable_mclk(priv->tas_ext_clk, false);
#endif

#ifdef RUNTIME_SUSPEND_RESUME
	//pull down priv->gpio_npdn to power down devices
	if ((gpio_is_valid(priv->gpio_npdn))&&(gpio_is_valid(priv->gpio_nreset))) {
		pr_info("%s: pull reset down\n", __func__);
		gpio_set_value(priv->gpio_nreset, 0);
		pr_info("%s: pull PDN down\n", __func__);
		gpio_set_value(priv->gpio_npdn, 0);
	}

	// disable vdd_18v
	schedule_delayed_work(&tas5731_power_save_work.work, TAS5731_POWER_SAVE_DELAY);
#endif

	return;
}

static int tas5731_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas5731_private *priv = snd_soc_codec_get_drvdata(codec);

	pr_info("%s entered\n", __func__);
	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return;

	priv->rate = params_rate(params);

	return 0;
}

static int tas5731_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas5731_private *priv = snd_soc_codec_get_drvdata(codec);
	unsigned char val = 0;
	int ret = 0;
	unsigned char buf[2] = {0};

	pr_info("%s entered\n", __func__);
	if (stream != SNDRV_PCM_STREAM_PLAYBACK)
		return;

	if (mute)
		val = TAS5731_SOFT_MUTE_ALL;

	//TODO: write soft mute register.
	buf[0] = TAS5731_REG_SOFT_MUTE;
	buf[1] = val;
	ret = tas5731_i2c_write(priv->i2c, buf, 2);

	return ret;
}

/* TAS5731 controls */
static const DECLARE_TLV_DB_SCALE(tas5731_dac_tlv, -10350, 50, 1);

static const struct snd_kcontrol_new tas5731_controls[] = {
	SOC_SINGLE_TLV("Master Playback Volume", TAS5731_REG_MASTER_VOL,
			   0, 0xff, 1, tas5731_dac_tlv),
	SOC_DOUBLE_R_TLV("Channel 1/2 Playback Volume",
			 TAS5731_REG_CHANNEL_VOL(0), TAS5731_REG_CHANNEL_VOL(1),
			 0, 0xff, 1, tas5731_dac_tlv),
	SOC_DOUBLE_R_TLV("Channel 3/4 Playback Volume",
			 TAS5731_REG_CHANNEL_VOL(2), TAS5731_REG_CHANNEL_VOL(3),
			 0, 0xff, 1, tas5731_dac_tlv),
	SOC_DOUBLE_R_TLV("Channel 5/6 Playback Volume",
			 TAS5731_REG_CHANNEL_VOL(4), TAS5731_REG_CHANNEL_VOL(5),
			 0, 0xff, 1, tas5731_dac_tlv),
};

static const struct snd_soc_dai_ops tas5731_dai_ops = {
	.prepare = tas5731_prepare,
	.shutdown = tas5731_shutdown,
	.hw_params  = tas5731_hw_params,
	.set_sysclk = tas5731_set_dai_sysclk,
	.set_fmt    = tas5731_set_dai_fmt,
	.mute_stream    = tas5731_mute_stream,
};

static struct snd_soc_dai_driver tas5731_dai = {
	.name = "tas5731-hifi",
	.playback = {
		.stream_name    = "Playback",
		.channels_min   = 2,
		.channels_max   = 6,
		.rates      = TAS5731_PCM_RATES,
		.formats    = TAS5731_PCM_FORMATS,
	},
	.ops = &tas5731_dai_ops,
};

#ifdef CONFIG_PM
static int tas5731_soc_resume(struct snd_soc_codec *codec)
{
	struct tas5731_private *priv = snd_soc_codec_get_drvdata(codec);
	struct i2c_client *client;
	unsigned char i2c_cmd1[2] = {0};
	char dev_id;

	pr_info("%s entered\n", __func__);

#ifndef RUNTIME_SUSPEND_RESUME
	//pull up priv->gpio_npdn to power down devices
	if ((gpio_is_valid(priv->gpio_npdn))&&(gpio_is_valid(priv->gpio_nreset))) {
		pr_info("%s: pull PDN up\n", __func__);
		gpio_set_value(priv->gpio_npdn, 1);
		udelay(200);
		pr_info("%s: pull reset up\n", __func__);
		gpio_set_value(priv->gpio_nreset, 1);

		/* Codec needs ~15ms to wake up, longer than 13.5ms */
		msleep(15);
	}
	msleep(300);

	/* The TAS5731 always returns 0x03 in its TAS5731_REG_DEV_ID register */
	tas5731_i2c_read(priv->i2c, TAS5731_REG_DEV_ID, &dev_id, 1 );
	if (dev_id != 0x00) {
		pr_err("%s: Failed to identify TAS5731 codec (got %02x)\n", __func__, dev_id);
	}

	tas5731_init(priv->i2c);
	client = priv->i2c;
#endif
	return 0;
}

static int tas5731_soc_suspend(struct snd_soc_codec *codec)
{
	struct tas5731_private *priv = snd_soc_codec_get_drvdata(codec);
	
	pr_info("%s entered\n", __func__);

#ifndef RUNTIME_SUSPEND_RESUME
	//pull down priv->gpio_npdn to power down devices
	if ((gpio_is_valid(priv->gpio_npdn))&&(gpio_is_valid(priv->gpio_nreset))) {
		pr_info("%s: pull reset down\n", __func__);
		gpio_set_value(priv->gpio_nreset, 0);
		pr_info("%s: pull PDN down\n", __func__);
		gpio_set_value(priv->gpio_npdn, 0);
	}
#endif
	return 0;
}
#else
#define tas5731_soc_resume  NULL
#define tas5731_soc_suspend NULL
#endif /* CONFIG_PM */

#ifdef CONFIG_OF
static const struct of_device_id tas5731_dt_ids[] = {
	{ .compatible = "ti,tas5731", },
	{ }
};
MODULE_DEVICE_TABLE(of, tas5731_dt_ids);
#endif

static int tas5731_probe(struct snd_soc_codec *codec)
{
	pr_info("%s entered\n", __func__);
	//TODO: currently we move init flow to tas5731_i2c_probe(), but
	//      maybe we need to move to here.
	return 0;
}

static int tas5731_remove(struct snd_soc_codec *codec)
{
	struct tas5731_private *priv = snd_soc_codec_get_drvdata(codec);

	pr_info("%s entered\n", __func__);
	if (gpio_is_valid(priv->gpio_nreset))
		/* Set codec to the reset state */
		gpio_set_value(priv->gpio_nreset, 0);

	return 0;
};

static struct snd_soc_codec_driver soc_codec_dev_tas5731 = {
	.probe          = tas5731_probe,
	.remove         = tas5731_remove,
	.resume         = tas5731_soc_resume,
	.suspend        = tas5731_soc_suspend,
	.controls       = tas5731_controls,
	.num_controls       = ARRAY_SIZE(tas5731_controls),
};

static const struct i2c_device_id tas5731_i2c_id[] = {
	{ "tas5731", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, tas5731_i2c_id);

#ifdef ENABLE_MI2S
static int msm_q6_enable_mi2s_clocks(bool enable)
{
	union afe_port_config port_config;
	int rc = 0;

	pr_info("%s: set msm_q6_enable_mi2s_clocks %d\n", __func__, enable);

	if(enable) {
		port_config.i2s.channel_mode = AFE_PORT_I2S_SD0;
		port_config.i2s.mono_stereo = MSM_AFE_CH_STEREO;
		port_config.i2s.data_format = 0;
		port_config.i2s.bit_width = 16;
		port_config.i2s.reserved = 0;
		port_config.i2s.i2s_cfg_minor_version = AFE_API_VERSION_I2S_CONFIG;
		port_config.i2s.sample_rate = 48000;
		port_config.i2s.ws_src = 1;

		rc = afe_port_start(AFE_PORT_ID_PRIMARY_MI2S_RX, &port_config, 48000);
		if (IS_ERR_VALUE(rc)) {
			printk(KERN_ERR"fail to open AFE port\n");
			return -EINVAL;
		}

	} else {
		rc = afe_close(AFE_PORT_ID_PRIMARY_MI2S_RX);
		if (IS_ERR_VALUE(rc)) {
			printk(KERN_ERR"fail to close AFE port\n");
			return  -EINVAL;
		}
	}

	return rc;
}
#endif

static int tas5731_i2c_probe(struct i2c_client *i2c,
				 const struct i2c_device_id *id)
{
	struct tas5731_private *priv;
	struct device *dev = &i2c->dev;
	int gpio_nreset = -EINVAL;
	int gpio_npdn = -EINVAL;
	int ret;
	char dev_id;
	struct clk *tas_ext_clk;
	int modem_state;
	int val = 0;
	void __iomem *vaddr = NULL;
	int port_id = AFE_PORT_ID_PRIMARY_MI2S_RX;
#ifdef RUNTIME_SUSPEND_RESUME
	static struct regulator *vdd_18v;
#endif

	pr_info("%s entered\n", __func__);

	modem_state = apr_get_modem_state();
	if (modem_state == APR_SUBSYS_DOWN) {
		pr_err("debug %s Modem is not loaded yet %d\n", __func__, modem_state);
		return -EPROBE_DEFER;
	}
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->i2c = i2c;
	i2c_set_clientdata(i2c, priv);

	if (of_match_device(of_match_ptr(tas5731_dt_ids), dev)) {
		struct device_node *of_node = dev->of_node;
		gpio_npdn = of_get_named_gpio(of_node, "pdn-gpio", 0);
	}

	if (gpio_is_valid(gpio_npdn))
		if (devm_gpio_request(dev, gpio_npdn, "TAS5731 PDN"))
			gpio_nreset = -EINVAL;

	pr_info("gpio_pdn is 0x%x\n", gpio_npdn);

	if (of_match_device(of_match_ptr(tas5731_dt_ids), dev)) {
		struct device_node *of_node = dev->of_node;
		gpio_nreset = of_get_named_gpio(of_node, "reset-gpio", 0);
	}

	if (gpio_is_valid(gpio_nreset))
		if (devm_gpio_request(dev, gpio_nreset, "TAS5731 Reset"))
			gpio_nreset = -EINVAL;

	pr_info("gpio_nreset is 0x%x\n", gpio_nreset);

	priv->gpio_nreset = gpio_nreset;
	priv->gpio_npdn = gpio_npdn;

#ifdef RUNTIME_SUSPEND_RESUME
	// setup vdd_18v
	vdd_18v = devm_regulator_get(dev, "qci_vdd_18v");
	if (IS_ERR(vdd_18v)) {
		pr_err("qci_vdd_18v regulator get failed\n");
		vdd_18v = NULL;
		return -ENODEV;
	} else {
		pr_info("%s: qci_vdd_18v is get\n", __func__);
	}
	priv->vdd_18v = vdd_18v;

	// enable vdd_18v
	if (priv->vdd_18v) {
		cancel_delayed_work_sync(&tas5731_power_save_work.work);
		ret = regulator_enable(priv->vdd_18v);
		if (ret < 0) {
			pr_err("qci_vdd_18v regulator enable failed\n");
			return -ENODEV;
		}
		pr_info("%s: qci_vdd_18v is enable\n", __func__);
	} else {
		pr_err("none of qci_vdd_18v regulator\n");
	}
#endif

#ifdef ENABLE_MCLK1
	/* Register for mclk2 */
	tas_ext_clk = clk_get(dev, "tas_clk");
	if (IS_ERR(tas_ext_clk)) {
		dev_err(dev, "%s: clk get tas_ext_clk failed\n", __func__);
	} else dev_info(dev, "%s: clk get tas_ext_clk\n", __func__);
	priv->tas_ext_clk = tas_ext_clk;

	ret = tas5731_enable_mclk(priv->tas_ext_clk, true);
	if (ret < 0) {
		dev_err(dev, "%s: mclk enable failed\n", __func__);
		goto err_release_mclk;
	}
#endif

#ifdef ENABLE_MI2S
	/* mi2s_rx gpio initialize */
	ret = msm_gpioset_initialize(CLIENT_WCD_EXT, dev);
	if (ret < 0) {
		pr_err("Error reading dtsi file for gpios\n");
		goto err_disable_mclk;
	}

	/* vaddr_gpio_mux_spkr_ctl */
	vaddr = ioremap(0x07702004 , 4);
	val = ioread32(vaddr);
	val = val | 0x201C;
	iowrite32(val, vaddr);
	pr_info("vaddr_gpio_mux_spkr_ctl is 0x%x\n", val);
	iounmap(vaddr);

	/* enable mi2s_rx */
	mi2s_rx_clk.enable = true;
	mi2s_rx_clk.clk_id = Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT;
	mi2s_rx_clk.clk_freq_in_hz = 48000*16*2;
	ret = afe_set_lpass_clock_v2(port_id, &mi2s_rx_clk);
	if (ret < 0) {
		pr_err("%s:set afe_set_lpass_clock as true failed\n", __func__);
		goto err_disable_mclk;
	}

	ret = msm_gpioset_activate(CLIENT_WCD_EXT, "pri_i2s");
	if (ret < 0) {
		pr_err("%s: failed to activate the Pri_mi2s gpio's state\n",
			__func__);
		goto err;
	}

	msm_q6_enable_mi2s_clocks(1);
#endif

#ifndef RUNTIME_SUSPEND_RESUME
	msleep(300);

	if ((gpio_is_valid(gpio_npdn))&&(gpio_is_valid(gpio_nreset))) {
		/* Reset codec - minimum assertion time is 400ns */
		gpio_direction_output(gpio_nreset, 1);
		gpio_direction_output(gpio_npdn, 1);

		pr_info("%s: pull reset down\n", __func__);
		gpio_set_value(gpio_nreset, 0);
		pr_info("%s: pull PDN down\n", __func__);
		gpio_set_value(gpio_npdn, 0);
		msleep(100);
		pr_info("%s: pull PDN up\n", __func__);
		gpio_set_value(gpio_npdn, 1);
		udelay(200);
		pr_info("%s: pull reset up\n", __func__);
		gpio_set_value(gpio_nreset, 1);

		/* Codec needs ~15ms to wake up, longer than 13.5ms */
		msleep(15);
	}
	msleep(300);


	/* The TAS5731 always returns 0x03 in its TAS5731_REG_DEV_ID register */
	tas5731_i2c_read(i2c, TAS5731_REG_DEV_ID, &dev_id, 1 );
	if (dev_id != 0x00) {
		dev_err(dev,
			"Failed to identify TAS5731 codec (got %02x)\n", dev_id);
		return -ENODEV;
	}

	pr_info("start to init table, id is %02x\n", dev_id);
	tas5731_init(i2c);
#endif

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_tas5731,
		&tas5731_dai, 1);
	if (ret) {
		dev_err(dev, "Failed to register TAS5731 codec (return %d)\n", ret);
	}

#ifdef ENABLE_MCLK1
	//turn off mclk
	tas5731_enable_mclk(priv->tas_ext_clk, false);
#endif

#ifdef RUNTIME_SUSPEND_RESUME
	//pull down priv->gpio_npdn to power down devices
	if ((gpio_is_valid(priv->gpio_npdn))&&(gpio_is_valid(priv->gpio_nreset))) {
		pr_info("%s: pull reset down\n", __func__);
		gpio_set_value(priv->gpio_nreset, 0);
		pr_info("%s: pull PDN down\n", __func__);
		gpio_set_value(priv->gpio_npdn, 0);
	}
	// enter power save mode
	INIT_DELAYED_WORK(&tas5731_power_save_work.work, tas5731_power_save);
	tas5731_power_save_work.priv = priv;
	schedule_delayed_work(&tas5731_power_save_work.work, TAS5731_POWER_SAVE_DELAY);
#endif

	return ret;

err:
#ifdef ENABLE_MI2S
	mi2s_rx_clk.enable = false;
	mi2s_rx_clk.clk_freq_in_hz = 0;
	ret = afe_set_lpass_clock_v2(port_id, &mi2s_rx_clk);
	if (ret < 0)
		pr_err("%s:afe_set_lpass_clock close failed\n", __func__);
#endif
#ifdef ENABLE_MCLK1
err_disable_mclk:
	tas5731_enable_mclk(tas_ext_clk, false);
err_release_mclk:
	clk_put(tas_ext_clk);
#endif
#if defined(ENABLE_MI2S) || defined(ENABLE_MCLK1)
	return ret;	
#endif
}

static int tas5731_i2c_remove(struct i2c_client *i2c)
{
	pr_info("%s entered\n", __func__);
	snd_soc_unregister_codec(&i2c->dev);
	return 0;
}

static struct i2c_driver tas5731_i2c_driver = {
	.driver = {
		.name   = "tas5731",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(tas5731_dt_ids),
	},
	.id_table   = tas5731_i2c_id,
	.probe      = tas5731_i2c_probe,
	.remove     = tas5731_i2c_remove,
};

module_i2c_driver(tas5731_i2c_driver);

MODULE_AUTHOR("Will Huang <will.huang@quantatw.com>");
MODULE_DESCRIPTION("Texas Instruments TAS5731 ALSA SoC Codec Driver");
MODULE_LICENSE("GPL");
