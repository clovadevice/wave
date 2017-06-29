/* Copyright (c) 2014-2016, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <debug.h>
#include <platform/iomap.h>
#include <reg.h>
#include <target.h>
#include <platform.h>
#include <uart_dm.h>
#include <mmc.h>
#include <dev/keys.h>
#include <spmi_v2.h>
#include <pm8x41.h>
#include <board.h>
#include <baseband.h>
#include <hsusb.h>
#include <scm.h>
#include <platform/gpio.h>
#include <platform/irqs.h>
#include <platform/clock.h>
#include <crypto5_wrapper.h>
#include <partition_parser.h>
#include <stdlib.h>
#include <gpio.h>
#include <rpm-smd.h>
#include <qpic_nand.h>
#include <smem.h>
#include <reboot.h>

#if LONG_PRESS_POWER_ON
#include <shutdown_detect.h>
#endif

#if PON_VIB_SUPPORT
#include <vibrator.h>
#endif

#define PMIC_ARB_CHANNEL_NUM    0
#define PMIC_ARB_OWNER_ID       0
#define TLMM_VOL_UP_BTN_GPIO    90
#define TLMM_VOL_DOWN_BTN_GPIO  91
#define TLMM_MUTE_BTN_GPIO  57

#define TLMM_ACOK_GPIO  46
#define TLMM_LED_EN  33
#define TLMM_5V_EN  74

#if PON_VIB_SUPPORT
#define VIBRATE_TIME    250
#endif
#define HW_SUBTYPE_APQ_NOWGR 0xA

#define CE1_INSTANCE            1
#define CE_EE                   1
#define CE_FIFO_SIZE            64
#define CE_READ_PIPE            3
#define CE_WRITE_PIPE           2
#define CE_READ_PIPE_LOCK_GRP   0
#define CE_WRITE_PIPE_LOCK_GRP  0
#define CE_ARRAY_SIZE           20
#define SUB_TYPE_SKUT           0x0A

extern void smem_ptable_init(void);
extern void smem_add_modem_partitions(struct ptable *flash_ptable);
void target_sdc_init();

static struct ptable flash_ptable;

/* NANDc BAM pipe numbers */
#define DATA_CONSUMER_PIPE                            0
#define DATA_PRODUCER_PIPE                            1
#define CMD_PIPE                                      2

/* NANDc BAM pipe groups */
#define DATA_PRODUCER_PIPE_GRP                        0
#define DATA_CONSUMER_PIPE_GRP                        0
#define CMD_PIPE_GRP                                  1

/* NANDc EE */
#define QPIC_NAND_EE                                  0

/* NANDc max desc length. */
#define QPIC_NAND_MAX_DESC_LEN                        0x7FFF

#define LAST_NAND_PTN_LEN_PATTERN                     0xFFFFFFFF

struct qpic_nand_init_config config;

struct mmc_device *dev;

static uint32_t mmc_pwrctl_base[] =
	{ MSM_SDC1_BASE, MSM_SDC2_BASE };

static uint32_t mmc_sdhci_base[] =
	{ MSM_SDC1_SDHCI_BASE, MSM_SDC2_SDHCI_BASE };

static uint32_t  mmc_sdc_pwrctl_irq[] =
	{ SDCC1_PWRCTL_IRQ, SDCC2_PWRCTL_IRQ };

static void set_sdc_power_ctrl(void);
static void set_ebi2_config(void);

bool boot_factoryreset = false;

void update_ptable_names(void)
{
	uint32_t ptn_index;
	struct ptentry *ptentry_ptr = flash_ptable.parts;
	struct ptentry *boot_ptn;
	unsigned i;
	uint32_t len;

	/* Change all names to lower case. */
	for (ptn_index = 0; ptn_index != (uint32_t)flash_ptable.count; ptn_index++)
	{
		len = strlen(ptentry_ptr[ptn_index].name);

		for (i = 0; i < len; i++)
		{
			if (isupper(ptentry_ptr[ptn_index].name[i]))
			{
				ptentry_ptr[ptn_index].name[i] = tolower(ptentry_ptr[ptn_index].name[i]);
			}
		}

		/* SBL fills in the last partition length as 0xFFFFFFFF.
		* Update the length field based on the number of blocks on the flash.
		*/
		if ((uint32_t)(ptentry_ptr[ptn_index].length) == LAST_NAND_PTN_LEN_PATTERN)
		{
			ptentry_ptr[ptn_index].length = flash_num_blocks() - ptentry_ptr[ptn_index].start;
		}
	}
}

void target_early_init(void)
{
#if WITH_DEBUG_UART
	/* Do not intilaise UART in case the h/w
	* is RCM.
	*/
	if( board_hardware_id()!= HW_PLATFORM_RCM )
		uart_dm_init(1, 0, BLSP1_UART0_BASE);
	else
		return;
#endif

}

int target_is_emmc_boot(void)
{
	return platform_boot_dev_isemmc();
}

void target_sdc_init()
{
	struct mmc_config_data config;

	/* Set drive strength & pull ctrl values */
	set_sdc_power_ctrl();

	config.bus_width = DATA_BUS_WIDTH_8BIT;
	config.max_clk_rate = MMC_CLK_177MHZ;

	/* Try slot 1*/
	config.slot         = 1;
	config.sdhc_base    = mmc_sdhci_base[config.slot - 1];
	config.pwrctl_base  = mmc_pwrctl_base[config.slot - 1];
	config.pwr_irq      = mmc_sdc_pwrctl_irq[config.slot - 1];
	config.hs400_support = 0;

	if (!(dev = mmc_init(&config))) {
	/* Try slot 2 */
		config.slot         = 2;
		config.max_clk_rate = MMC_CLK_200MHZ;
		config.sdhc_base    = mmc_sdhci_base[config.slot - 1];
		config.pwrctl_base  = mmc_pwrctl_base[config.slot - 1];
		config.pwr_irq      = mmc_sdc_pwrctl_irq[config.slot - 1];

		if (!(dev = mmc_init(&config))) {
			dprintf(CRITICAL, "mmc init failed!");
			ASSERT(0);
		}
	}
}

void *target_mmc_device()
{
	return (void *) dev;
}

/* Return 1 if vol_up pressed */
int target_volume_up()
{
	static uint8_t first_time = 0;
	uint8_t status = 0;

	if (!first_time) {
		gpio_tlmm_config(TLMM_VOL_UP_BTN_GPIO, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA, GPIO_ENABLE);

		/* Wait for the gpio config to take effect - debounce time */
		udelay(10000);

		first_time = 1;
	}

	/* Get status of GPIO */
	status = gpio_status(TLMM_VOL_UP_BTN_GPIO);

	/* Active low signal. */
	return !status;
}

uint32_t target_mute()
{
       uint32_t status = 0;

       /* Get status of GPIO */
       status = gpio_status(TLMM_MUTE_BTN_GPIO);

       /* Active low signal. */
       return !status;

}


/* Return 1 if vol_down pressed */
uint32_t target_volume_down()
{
	if ((board_hardware_id() == HW_PLATFORM_QRD) &&
			(board_hardware_subtype() == SUB_TYPE_SKUT)) {
		uint32_t status = 0;

		gpio_tlmm_config(TLMM_VOL_DOWN_BTN_GPIO, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA, GPIO_ENABLE);

		/* Wait for the gpio config to take effect - debounce time */
		thread_sleep(10);

		/* Get status of GPIO */
		status = gpio_status(TLMM_VOL_DOWN_BTN_GPIO);

		/* Active low signal. */
		return !status;
	} else {
		/* Volume down button tied in with PMIC RESIN. */
		return pm8x41_resin_status();
	}
}

 uint32_t target_charger_status()
 {
 	uint32_t status = 0;
 
 	/* Get status of GPIO */
 	status = gpio_status(TLMM_ACOK_GPIO);

 	return status;
 
 }

static void target_keystatus()
{
	keys_init();

	if(target_volume_down())
		keys_post_event(KEY_VOLUMEDOWN, 1);

	if(target_volume_up())
		keys_post_event(KEY_VOLUMEUP, 1);

}

static void set_sdc_power_ctrl()
{
	/* Drive strength configs for sdc pins */
	struct tlmm_cfgs sdc1_hdrv_cfg[] =
	{
		{ SDC1_CLK_HDRV_CTL_OFF,  TLMM_CUR_VAL_16MA, TLMM_HDRV_MASK, SDC1_HDRV_PULL_CTL },
		{ SDC1_CMD_HDRV_CTL_OFF,  TLMM_CUR_VAL_10MA, TLMM_HDRV_MASK, SDC1_HDRV_PULL_CTL },
		{ SDC1_DATA_HDRV_CTL_OFF, TLMM_CUR_VAL_10MA, TLMM_HDRV_MASK, SDC1_HDRV_PULL_CTL },
	};

	/* Pull configs for sdc pins */
	struct tlmm_cfgs sdc1_pull_cfg[] =
	{
		{ SDC1_CLK_PULL_CTL_OFF,  TLMM_NO_PULL, TLMM_PULL_MASK, SDC1_HDRV_PULL_CTL },
		{ SDC1_CMD_PULL_CTL_OFF,  TLMM_PULL_UP, TLMM_PULL_MASK, SDC1_HDRV_PULL_CTL },
		{ SDC1_DATA_PULL_CTL_OFF, TLMM_PULL_UP, TLMM_PULL_MASK, SDC1_HDRV_PULL_CTL },
	};

	/* Set the drive strength & pull control values */
	tlmm_set_hdrive_ctrl(sdc1_hdrv_cfg, ARRAY_SIZE(sdc1_hdrv_cfg));
	tlmm_set_pull_ctrl(sdc1_pull_cfg, ARRAY_SIZE(sdc1_pull_cfg));
}

static void set_ebi2_config()
{
	/* Drive strength configs for ebi2 pins */
	struct tlmm_cfgs ebi2_hdrv_cfg[] =
	{
		{ EBI2_BUSY_HDRV_CTL_OFF,  TLMM_CUR_VAL_16MA, TLMM_HDRV_MASK, TLMM_EBI2_EMMC_GPIO_CFG },
		{ EBI2_WE_HDRV_CTL_OFF,  TLMM_CUR_VAL_16MA, TLMM_HDRV_MASK, TLMM_EBI2_EMMC_GPIO_CFG },
		{ EBI2_OE_HDRV_CTL_OFF,  TLMM_CUR_VAL_16MA, TLMM_HDRV_MASK, TLMM_EBI2_EMMC_GPIO_CFG },
		{ EBI2_CLE_HDRV_CTL_OFF,  TLMM_CUR_VAL_16MA, TLMM_HDRV_MASK, TLMM_EBI2_EMMC_GPIO_CFG },
		{ EBI2_ALE_HDRV_CTL_OFF,  TLMM_CUR_VAL_16MA, TLMM_HDRV_MASK, TLMM_EBI2_EMMC_GPIO_CFG },
		{ EBI2_CS_HDRV_CTL_OFF,  TLMM_CUR_VAL_10MA, TLMM_HDRV_MASK, TLMM_EBI2_EMMC_GPIO_CFG },
		{ EBI2_DATA_HDRV_CTL_OFF, TLMM_CUR_VAL_6MA, TLMM_HDRV_MASK, SDC1_HDRV_PULL_CTL },
	};

	/* Pull configs for ebi2 pins */
	struct tlmm_cfgs ebi2_pull_cfg[] =
	{
		{ EBI2_BUSY_PULL_CTL_OFF,  TLMM_NO_PULL, TLMM_PULL_MASK, TLMM_EBI2_EMMC_GPIO_CFG },
		{ EBI2_WE_PULL_CTL_OFF,  TLMM_PULL_UP, TLMM_PULL_MASK, TLMM_EBI2_EMMC_GPIO_CFG },
		{ EBI2_OE_PULL_CTL_OFF,  TLMM_PULL_UP, TLMM_PULL_MASK, TLMM_EBI2_EMMC_GPIO_CFG },
		{ EBI2_CLE_PULL_CTL_OFF,  TLMM_PULL_UP, TLMM_PULL_MASK, TLMM_EBI2_EMMC_GPIO_CFG },
		{ EBI2_ALE_PULL_CTL_OFF,  TLMM_PULL_UP, TLMM_PULL_MASK, TLMM_EBI2_EMMC_GPIO_CFG },
		{ EBI2_CS_PULL_CTL_OFF,  TLMM_PULL_UP, TLMM_PULL_MASK, TLMM_EBI2_EMMC_GPIO_CFG },
		{ EBI2_DATA_PULL_CTL_OFF, TLMM_PULL_UP, TLMM_PULL_MASK, SDC1_HDRV_PULL_CTL },
	};

	/* Set the drive strength & pull control values */
	tlmm_set_hdrive_ctrl(ebi2_hdrv_cfg, ARRAY_SIZE(ebi2_hdrv_cfg));
	tlmm_set_pull_ctrl(ebi2_pull_cfg, ARRAY_SIZE(ebi2_pull_cfg));

}

#include <blsp_qup.h>
#include <err.h>
#include <i2c_qup.h>
#define LED_I2C_ADDR1	0x32
#define LED_I2C_ADDR2	0x33
#define LED_I2C_ADDR3	0x34
#define LED_I2C_ADDR4	0x35
#define LED_RESET_REG_ADDR	0x3D
#define LED_ENABLE_REG_ADDR	0x00
#define LED_CH_MSB_REG_ADDR	0x04
#define LED_CH_LSB_REG_ADDR	0x05
#define LED_MISC_REG_ADDR	0x36
#define LED_CLR_BASE_REG_ADDR	0x16
#define LED_CUR_BASE_REG_ADDR	0x26
#define LED_VDD5_GPIO	74
#define LED_BTN_ENA1_GPIO	69
#define LED_BTN_TRIG_GPIO	72
#define LED_SIDE_ENA1_GPIO	68
#define LED_SIDE_TRIG_GPIO	71
#define GPIO_RESET_IN	26
#define LED_BTN_CUR	0xC8
#define LED_SIDE_CUR	0x28

static struct qup_i2c_dev *i2c_side_dev, *i2c_btn_dev;
int led_pwm_index = 0;
uint8_t btn_pwm[2][3] = {{0xFF, 0x64, 0x28}, /* normal node: R, G, B */
			{0xFF, 0x32, 0}};  /* factory node: R, G, B */
uint8_t side_pwm[3] = {0xC8, 0x82, 0x32};

static int led_i2c_write(struct qup_i2c_dev *i2c_dev, uint8_t i2c_addr, uint8_t addr, uint8_t val)
{
	int ret = 0;
	uint8_t data_buf[] = { addr, val };

	/* Create a i2c_msg buffer, that is used to put the controller into write
	mode and then to write some data. */
	struct i2c_msg msg_buf[] = {
		{i2c_addr, I2C_M_WR, 2, data_buf}
	};

	ret = qup_i2c_xfer(i2c_dev, msg_buf, 1);
	if(ret < 0) {
		dprintf(CRITICAL, "qup_i2c_xfer error %d\n", ret);
		return ret;
	}
	return 0;
}

static int led_i2c_read(struct qup_i2c_dev *i2c_dev, uint8_t i2c_addr, uint8_t addr)
{
	int ret = 0, val;
	/* Create a i2c_msg buffer, that is used to put the controller into read
	   mode and then to read some data. */
	struct i2c_msg msg_buf[] = {
		{i2c_addr, I2C_M_WR, 1, &addr},
		{i2c_addr, I2C_M_RD, 1, &val}
	};

	ret = qup_i2c_xfer(i2c_dev, msg_buf, 2);
	if(ret < 0) {
		dprintf(CRITICAL, "qup_i2c_xfer error %d\n", ret);
		return ret;
	}
	return val;
}

static uint32_t is_pwrkey_pon()
{
#if PMI_CONFIGURED
	return target_is_pwrkey_pon_reason();
#else
	uint8_t pon_reason = pm8x41_get_pon_reason();

	if (pm8x41_get_is_cold_boot() && (pon_reason == KPDPWR_N))
		return 1;
	else
		return 0;
#endif
}

void target_init(void)
{
	uint32_t base_addr;
	uint8_t slot;
       int i =0;
       uint8_t reboot_mode = 0;

	dprintf(INFO, "target_init()\n");

	spmi_init(PMIC_ARB_CHANNEL_NUM, PMIC_ARB_OWNER_ID);

	target_keystatus();

       gpio_tlmm_config(TLMM_ACOK_GPIO, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA, GPIO_ENABLE);


#if BOOT_CONFIG_SUPPORT
	platform_read_boot_config();
#endif

	if (platform_boot_dev_isemmc()) {
		target_sdc_init();
		if (partition_read_table())
		{
			dprintf(CRITICAL, "Error reading the partition table info\n");
			ASSERT(0);
		}

	} else {
		set_ebi2_config();
		config.pipes.read_pipe = DATA_PRODUCER_PIPE;
		config.pipes.write_pipe = DATA_CONSUMER_PIPE;
		config.pipes.cmd_pipe = CMD_PIPE;

		config.pipes.read_pipe_grp = DATA_PRODUCER_PIPE_GRP;
		config.pipes.write_pipe_grp = DATA_CONSUMER_PIPE_GRP;
		config.pipes.cmd_pipe_grp = CMD_PIPE_GRP;

		config.bam_base = MSM_NAND_BAM_BASE;
		config.nand_base = MSM_NAND_BASE;
		config.ee = QPIC_NAND_EE;
		config.max_desc_len = QPIC_NAND_MAX_DESC_LEN;

		qpic_nand_init(&config);

		ptable_init(&flash_ptable);
		smem_ptable_init();
		smem_add_modem_partitions(&flash_ptable);

		update_ptable_names();
		flash_set_ptable(&flash_ptable);


	}

#if LONG_PRESS_POWER_ON
		//shutdown_detect();
#endif


	gpio_tlmm_config(TLMM_MUTE_BTN_GPIO, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA, GPIO_ENABLE);
	//gpio_tlmm_config(GPIO_RESET_IN, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA, GPIO_ENABLE);
	mdelay(100);

	if(is_pwrkey_pon()&&target_mute()==1){
	dprintf(INFO, "factory reset detect\n");
       //disable s2
	pm8x41_reg_write(0x843, 0x0);
	pm8x41_reg_write(0x84b, 0x0);
	pm8x41_reg_write(0x8d0, 0xa5);
	pm8x41_reg_write(0x875, 0x05);
       while(true){

               if(target_mute()==0||pm8x41_get_pwrkey_is_pressed()==0){

		  while(true){
		    if(target_mute()==0&&pm8x41_get_pwrkey_is_pressed()==0){
		    shutdown_device();	   	
                  break;
		    }
		    mdelay(50);
		  }
               }
               i++;
               if(i==200){
                  boot_factoryreset = true;
                break;
               }
               mdelay(50);
       }
}


       reboot_mode = pm8x41_reg_read(PON_SOFT_RB_SPARE);
       reboot_mode = (reboot_mode & 0xFC) >> 2;

	if(reboot_mode!=CHARGING_MODE){
		if (boot_factoryreset || reboot_mode == FACTORY_RESET ) led_pwm_index = 1;

		//dprintf(CRITICAL, "led_pwm_index=%d, reboot_mode=0x%02X\n", led_pwm_index, reboot_mode);
		gpio_tlmm_config(LED_VDD5_GPIO, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA, GPIO_ENABLE);
		gpio_tlmm_config(LED_BTN_ENA1_GPIO, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA, GPIO_ENABLE);
		gpio_tlmm_config(LED_BTN_TRIG_GPIO, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA, GPIO_DISABLE);
		gpio_tlmm_config(LED_SIDE_ENA1_GPIO, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA, GPIO_ENABLE);
		gpio_tlmm_config(LED_SIDE_TRIG_GPIO, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA, GPIO_DISABLE);
		udelay(1000);
		// For apq8009/msm8009, i2c-3 maps to BLSP1 QUP3
		i2c_btn_dev = qup_blsp_i2c_init(BLSP_ID_1, QUP_ID_3, 400000, 19200000);
		if(!i2c_btn_dev) {
			dprintf(CRITICAL, "qup_blsp_i2c_init failed\n");
			ASSERT(0);
		} else {
			int ret = NO_ERROR;
			/******** BTN LED1: 3-32 Configuration Settings ********/
			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_RESET_REG_ADDR, 0xFF);
			if (ret) dprintf(CRITICAL, "LED_RESET_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_RESET_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_RESET_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_ENABLE_REG_ADDR, 0x40);
			if (ret) dprintf(CRITICAL, "LED_ENABLE_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_ENABLE_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_ENABLE_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_MISC_REG_ADDR, 0x7E);
			if (ret) dprintf(CRITICAL, "LED_MISC_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_MISC_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_MISC_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CH_MSB_REG_ADDR, 0x01);
			if (ret) dprintf(CRITICAL, "LED_CH_MSB_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_CH_MSB_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CH_MSB_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CH_LSB_REG_ADDR, 0xFF);
			if (ret) dprintf(CRITICAL, "LED_CH_LSB_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_CH_LSB_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CH_LSB_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D1_BCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_RCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 1, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D1_GCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_GCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 1));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 2, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D1_RCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_BCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 2));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 3, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D2_BCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_RCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 3));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 4, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D2_GCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_GCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 4));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 5, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D2_RCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_BCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 5));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 6, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D3_BCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_RCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 6));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 7, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D3_GCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_GCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 7));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 8, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D3_RCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_BCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 8));

			/******** BTN LED2: 3-34 Configuration Settings ********/
			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_RESET_REG_ADDR, 0xFF);
			if (ret) dprintf(CRITICAL, "LED_RESET_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_RESET_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_RESET_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_ENABLE_REG_ADDR, 0x40);
			if (ret) dprintf(CRITICAL, "LED_ENABLE_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_ENABLE_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_ENABLE_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_MISC_REG_ADDR, 0x7E);
			if (ret) dprintf(CRITICAL, "LED_MISC_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_MISC_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_MISC_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CH_MSB_REG_ADDR, 0x01);
			if (ret) dprintf(CRITICAL, "LED_CH_MSB_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_CH_MSB_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CH_MSB_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CH_LSB_REG_ADDR, 0xFF);
			if (ret) dprintf(CRITICAL, "LED_CH_LSB_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_CH_LSB_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CH_LSB_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D1_BCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_RCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR + 1, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D1_GCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_GCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR + 1));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR + 2, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D1_RCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_BCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR + 2));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR + 3, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D2_BCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_RCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR + 3));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR + 4, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D2_GCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_GCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR + 4));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR + 5, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D2_RCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_BCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR + 5));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR + 6, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D3_BCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_RCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR + 6));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR + 7, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D3_GCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_GCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR + 7));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR + 8, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D3_RCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_BCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CUR_BASE_REG_ADDR + 8));

			/******** BTN LED3: 3-33 Configuration Settings ********/
			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_RESET_REG_ADDR, 0xFF);
			if (ret) dprintf(CRITICAL, "LED_RESET_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_RESET_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_RESET_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_ENABLE_REG_ADDR, 0x40);
			if (ret) dprintf(CRITICAL, "LED_ENABLE_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_ENABLE_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_ENABLE_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_MISC_REG_ADDR, 0x7E);
			if (ret) dprintf(CRITICAL, "LED_MISC_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_MISC_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_MISC_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CH_MSB_REG_ADDR, 0x01);
			if (ret) dprintf(CRITICAL, "LED_CH_MSB_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_CH_MSB_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CH_MSB_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CH_LSB_REG_ADDR, 0xFF);
			if (ret) dprintf(CRITICAL, "LED_CH_LSB_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_CH_LSB_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CH_LSB_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D1_BCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_RCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR + 1, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D1_GCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_GCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR + 1));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR + 2, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D1_RCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_BCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR + 2));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR + 3, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D2_BCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_RCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR + 3));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR + 4, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D2_GCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_GCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR + 4));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR + 5, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D2_RCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_BCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR + 5));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR + 6, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D3_BCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_RCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR + 6));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR + 7, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D3_GCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_GCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR + 7));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR + 8, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D3_RCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_BCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CUR_BASE_REG_ADDR + 8));

			/******** BTN LED4: 3-35 Configuration Settings ********/
			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_RESET_REG_ADDR, 0xFF);
			if (ret) dprintf(CRITICAL, "LED_RESET_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_RESET_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_RESET_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_ENABLE_REG_ADDR, 0x40);
			if (ret) dprintf(CRITICAL, "LED_ENABLE_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_ENABLE_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_ENABLE_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_MISC_REG_ADDR, 0x7E);
			if (ret) dprintf(CRITICAL, "LED_MISC_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_MISC_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_MISC_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CH_MSB_REG_ADDR, 0x01);
			if (ret) dprintf(CRITICAL, "LED_CH_MSB_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_CH_MSB_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CH_MSB_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CH_LSB_REG_ADDR, 0xFF);
			if (ret) dprintf(CRITICAL, "LED_CH_LSB_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_CH_LSB_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CH_LSB_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D1_BCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_RCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR + 1, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D1_GCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_GCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR + 1));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR + 2, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D1_RCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_BCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR + 2));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR + 3, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D2_BCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_RCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR + 3));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR + 4, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D2_GCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_GCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR + 4));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR + 5, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D2_RCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_BCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR + 5));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR + 6, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D3_BCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_RCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR + 6));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR + 7, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D3_GCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_GCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR + 7));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR + 8, LED_BTN_CUR);
			if (ret) dprintf(CRITICAL, "LED_D3_RCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_BCUR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CUR_BASE_REG_ADDR + 8));


			/******** BTN LED1: 3-32 PWM Settings ********/
			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR, btn_pwm[led_pwm_index][0]);
			if (ret) dprintf(CRITICAL, "LED_D1_BCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_RCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 1, btn_pwm[led_pwm_index][1]);
			if (ret) dprintf(CRITICAL, "LED_D1_GCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_GCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 1));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 2, btn_pwm[led_pwm_index][2]);
			if (ret) dprintf(CRITICAL, "LED_D1_RCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_BCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 2));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 3, btn_pwm[led_pwm_index][0]);
			if (ret) dprintf(CRITICAL, "LED_D2_BCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_RCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 3));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 4, btn_pwm[led_pwm_index][1]);
			if (ret) dprintf(CRITICAL, "LED_D2_GCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_GCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 4));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 5, btn_pwm[led_pwm_index][2]);
			if (ret) dprintf(CRITICAL, "LED_D2_RCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_BCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 5));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 6, btn_pwm[led_pwm_index][0]);
			if (ret) dprintf(CRITICAL, "LED_D3_BCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_RCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 6));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 7, btn_pwm[led_pwm_index][1]);
			if (ret) dprintf(CRITICAL, "LED_D3_GCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_GCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 7));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 8, btn_pwm[led_pwm_index][2]);
			if (ret) dprintf(CRITICAL, "LED_D3_RCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_BCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 8));

			/******** BTN LED2: 3-34 PWM Settings ********/
			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR, btn_pwm[led_pwm_index][0]);
			if (ret) dprintf(CRITICAL, "LED_D1_BCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_RCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR + 1, btn_pwm[led_pwm_index][1]);
			if (ret) dprintf(CRITICAL, "LED_D1_GCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_GCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR + 1));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR + 2, btn_pwm[led_pwm_index][2]);
			if (ret) dprintf(CRITICAL, "LED_D1_RCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_BCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR + 2));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR + 3, btn_pwm[led_pwm_index][0]);
			if (ret) dprintf(CRITICAL, "LED_D2_BCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_RCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR + 3));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR + 4, btn_pwm[led_pwm_index][1]);
			if (ret) dprintf(CRITICAL, "LED_D2_GCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_GCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR + 4));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR + 5, btn_pwm[led_pwm_index][2]);
			if (ret) dprintf(CRITICAL, "LED_D2_RCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_BCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR + 5));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR + 6, btn_pwm[led_pwm_index][0]);
			if (ret) dprintf(CRITICAL, "LED_D3_BCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_RCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR + 6));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR + 7, btn_pwm[led_pwm_index][1]);
			if (ret) dprintf(CRITICAL, "LED_D3_GCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_GCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR + 7));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR + 8, btn_pwm[led_pwm_index][2]);
			if (ret) dprintf(CRITICAL, "LED_D3_RCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_BCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR3, LED_CLR_BASE_REG_ADDR + 8));

			/******** BTN LED3: 3-33 PWM Settings ********/
			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR, btn_pwm[led_pwm_index][0]);
			if (ret) dprintf(CRITICAL, "LED_D1_BCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_RCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR + 1, btn_pwm[led_pwm_index][1]);
			if (ret) dprintf(CRITICAL, "LED_D1_GCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_GCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR + 1));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR + 2, btn_pwm[led_pwm_index][2]);
			if (ret) dprintf(CRITICAL, "LED_D1_RCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_BCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR + 2));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR + 3, btn_pwm[led_pwm_index][0]);
			if (ret) dprintf(CRITICAL, "LED_D2_BCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_RCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR + 3));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR + 4, btn_pwm[led_pwm_index][1]);
			if (ret) dprintf(CRITICAL, "LED_D2_GCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_GCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR + 4));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR + 5, btn_pwm[led_pwm_index][2]);
			if (ret) dprintf(CRITICAL, "LED_D2_RCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_BCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR + 5));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR + 6, btn_pwm[led_pwm_index][0]);
			if (ret) dprintf(CRITICAL, "LED_D3_BCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_RCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR + 6));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR + 7, btn_pwm[led_pwm_index][1]);
			if (ret) dprintf(CRITICAL, "LED_D3_GCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_GCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR + 7));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR + 8, btn_pwm[led_pwm_index][2]);
			if (ret) dprintf(CRITICAL, "LED_D3_RCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_BCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR2, LED_CLR_BASE_REG_ADDR + 8));

			/******** BTN LED4: 3-35 PWM Settings ********/
			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR, btn_pwm[led_pwm_index][0]);
			if (ret) dprintf(CRITICAL, "LED_D1_BCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_RCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR + 1, btn_pwm[led_pwm_index][1]);
			if (ret) dprintf(CRITICAL, "LED_D1_GCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_GCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR + 1));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR + 2, btn_pwm[led_pwm_index][2]);
			if (ret) dprintf(CRITICAL, "LED_D1_RCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D1_BCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR + 2));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR + 3, btn_pwm[led_pwm_index][0]);
			if (ret) dprintf(CRITICAL, "LED_D2_BCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_RCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR + 3));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR + 4, btn_pwm[led_pwm_index][1]);
			if (ret) dprintf(CRITICAL, "LED_D2_GCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_GCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR + 4));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR + 5, btn_pwm[led_pwm_index][2]);
			if (ret) dprintf(CRITICAL, "LED_D2_RCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D2_BCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR + 5));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR + 6, btn_pwm[led_pwm_index][0]);
			if (ret) dprintf(CRITICAL, "LED_D3_BCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_RCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR + 6));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR + 7, btn_pwm[led_pwm_index][1]);
			if (ret) dprintf(CRITICAL, "LED_D3_GCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_GCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR + 7));

			ret = led_i2c_write(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR + 8, btn_pwm[led_pwm_index][2]);
			if (ret) dprintf(CRITICAL, "LED_D3_RCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_D3_BCLR_REG: %d\n", led_i2c_read(i2c_btn_dev, LED_I2C_ADDR4, LED_CLR_BASE_REG_ADDR + 8));

			dprintf(INFO, "BTN1_LED initialization completed\n");

		}
		// For apq8009/msm8009, i2c-2 maps to BLSP1 QUP2
		i2c_side_dev = qup_blsp_i2c_init(BLSP_ID_1, QUP_ID_2, 400000, 19200000);
		if(!i2c_side_dev) {
			dprintf(CRITICAL, "qup_blsp_i2c_init failed\n");
			ASSERT(0);
		} else {
			int ret = NO_ERROR;

			ret = led_i2c_write(i2c_side_dev, LED_I2C_ADDR1, LED_RESET_REG_ADDR, 0xFF);
			if (ret) dprintf(CRITICAL, "LED_RESET_REG: I2C Write failure\n");
			//else dprintf(INFO, "SIDE LED_RESET_REG: %d\n", led_i2c_read(i2c_side_dev, LED_I2C_ADDR1, LED_RESET_REG_ADDR));

			ret = led_i2c_write(i2c_side_dev, LED_I2C_ADDR1, LED_ENABLE_REG_ADDR, 0x40);
			if (ret) dprintf(CRITICAL, "LED_ENABLE_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_ENABLE_REG: %d\n", led_i2c_read(i2c_side_dev, LED_I2C_ADDR1, LED_ENABLE_REG_ADDR));

			ret = led_i2c_write(i2c_side_dev, LED_I2C_ADDR1, LED_MISC_REG_ADDR, 0x7E);
			if (ret) dprintf(CRITICAL, "LED_MISC_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_MISC_REG: %d\n", led_i2c_read(i2c_side_dev, LED_I2C_ADDR1, LED_MISC_REG_ADDR));

			ret = led_i2c_write(i2c_side_dev, LED_I2C_ADDR1, LED_CH_MSB_REG_ADDR, 0x01);
			if (ret) dprintf(CRITICAL, "LED_CH_MSB_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_CH_MSB_REG: %d\n", led_i2c_read(i2c_side_dev, LED_I2C_ADDR1, LED_CH_MSB_REG_ADDR));

			ret = led_i2c_write(i2c_side_dev, LED_I2C_ADDR1, LED_CH_LSB_REG_ADDR, 0xF8);
			if (ret) dprintf(CRITICAL, "LED_CH_LSB_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_CH_LSB_REG: %d\n", led_i2c_read(i2c_side_dev, LED_I2C_ADDR1, LED_CH_LSB_REG_ADDR));

			ret = led_i2c_write(i2c_side_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 3, LED_SIDE_CUR);
			if (ret) dprintf(CRITICAL, "LED_PWR_BCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_PWR_BCUR_REG: %d\n", led_i2c_read(i2c_side_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 3));

			ret = led_i2c_write(i2c_side_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 4, LED_SIDE_CUR);
			if (ret) dprintf(CRITICAL, "LED_PWR_GCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_PWR_GCUR_REG: %d\n", led_i2c_read(i2c_side_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 4));

			ret = led_i2c_write(i2c_side_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 5, LED_SIDE_CUR);
			if (ret) dprintf(CRITICAL, "LED_PWR_RCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_PWR_RCUR_REG: %d\n", led_i2c_read(i2c_side_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 5));

			ret = led_i2c_write(i2c_side_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 6, LED_SIDE_CUR);
			if (ret) dprintf(CRITICAL, "LED_MUTE_BCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_MUTE_BCUR_REG: %d\n", led_i2c_read(i2c_side_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 6));

			ret = led_i2c_write(i2c_side_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 7, LED_SIDE_CUR);
			if (ret) dprintf(CRITICAL, "LED_MUTE_GCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_MUTE_GCUR_REG: %d\n", led_i2c_read(i2c_side_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 7));

			ret = led_i2c_write(i2c_side_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 8, LED_SIDE_CUR);
			if (ret) dprintf(CRITICAL, "LED_MUTE_RCUR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_MUTE_RCUR_REG: %d\n", led_i2c_read(i2c_side_dev, LED_I2C_ADDR1, LED_CUR_BASE_REG_ADDR + 8));

			ret = led_i2c_write(i2c_side_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 3, side_pwm[2]);
			if (ret) dprintf(CRITICAL, "LED_PWR_BCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_PWR_BCLR_REG: %d\n", led_i2c_read(i2c_side_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 3));

			ret = led_i2c_write(i2c_side_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 4, side_pwm[1]);
			if (ret) dprintf(CRITICAL, "LED_PWR_GCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_PWR_GCLR_REG: %d\n", led_i2c_read(i2c_side_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 4));

			ret = led_i2c_write(i2c_side_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 5, side_pwm[0]);
			if (ret) dprintf(CRITICAL, "LED_PWR_RCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_PWR_RCLR_REG: %d\n", led_i2c_read(i2c_side_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 5));

			ret = led_i2c_write(i2c_side_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 6, side_pwm[2]);
			if (ret) dprintf(CRITICAL, "LED_MUTE_BCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_MUTE_BCLR_REG: %d\n", led_i2c_read(i2c_side_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 6));

			ret = led_i2c_write(i2c_side_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 7, side_pwm[1]);
			if (ret) dprintf(CRITICAL, "LED_MUTE_GCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_MUTE_GCLR_REG: %d\n", led_i2c_read(i2c_side_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 7));

			ret = led_i2c_write(i2c_side_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 8, side_pwm[0]);
			if (ret) dprintf(CRITICAL, "LED_MUTE_RCLR_REG: I2C Write failure\n");
			//else dprintf(INFO, "LED_MUTE_RCLR_REG: %d\n", led_i2c_read(i2c_side_dev, LED_I2C_ADDR1, LED_CLR_BASE_REG_ADDR + 8));

			dprintf(INFO, "SIDE_LED initialization completed\n");
       	}
	}
#if PON_VIB_SUPPORT

	/* turn on vibrator to indicate that phone is booting up to end user */
		vib_timed_turn_on(VIBRATE_TIME);
#endif

	if (target_use_signed_kernel())
		target_crypto_init_params();

#if SMD_SUPPORT
	rpm_smd_init();
#endif

}

void target_serialno(unsigned char *buf)
{
	uint32_t serialno;
	if (target_is_emmc_boot()) {
		serialno = mmc_get_psn();
		snprintf((char *)buf, 13, "%x", serialno);
	}
}

unsigned board_machtype(void)
{
	return LINUX_MACHTYPE_UNKNOWN;
}

/* Detect the target type */
void target_detect(struct board_data *board)
{
	/*
	* already fill the board->target on board.c
	*/
}

void target_baseband_detect(struct board_data *board)
{
	uint32_t platform;

	platform = board->platform;
	switch(platform)
	{
	case MSM8909:
	case MSM8209:
	case MSM8208:
	case MSM8609:
		board->baseband = BASEBAND_MSM;
		break;

	case MDM9209:
	case MDM9309:
	case MDM9609:
		board->baseband = BASEBAND_MDM;
		break;

	case APQ8009:
		if ((board->platform_hw == HW_PLATFORM_MTP) &&
			((board->platform_subtype == HW_SUBTYPE_APQ_NOWGR) ||
			 (board->platform_subtype == HW_PLATFORM_SUBTYPE_SWOC_NOWGR_CIRC)))
			board->baseband = BASEBAND_APQ_NOWGR;
		else
			board->baseband = BASEBAND_APQ;
		break;

	default:
		dprintf(CRITICAL, "Platform type: %u is not supported\n", platform);
		ASSERT(0);
	};
}
uint8_t target_panel_auto_detect_enabled()
{
	uint8_t ret = 0;

	switch(board_hardware_id()) {
	default:
		ret = 0;
		break;
	}
	return ret;
}

static uint8_t splash_override;
/* Returns 1 if target supports continuous splash screen. */
int target_cont_splash_screen()
{
	uint8_t splash_screen = 0;
	if (!splash_override) {
		switch (board_hardware_id()) {
		case HW_PLATFORM_SURF:
		case HW_PLATFORM_MTP:
		case HW_PLATFORM_QRD:
		case HW_PLATFORM_RCM:
			splash_screen = 1;
			break;
		default:
			splash_screen = 0;
			break;
		}
		dprintf(SPEW, "Target_cont_splash=%d\n", splash_screen);
	}
	return splash_screen;
}

void target_force_cont_splash_disable(uint8_t override)
{
        splash_override = override;
}

/*Update this command line only for LE based builds*/
int get_target_boot_params(const char *cmdline, const char *part, char **buf)
{
	struct ptable *ptable;
	int system_ptn_index = -1;
	int le_based = -1;
	uint32_t buflen = 0;

	if (!cmdline || !part ) {
		dprintf(CRITICAL, "WARN: Invalid input param\n");
		return -1;
	}

	/*LE partition.xml will have recoveryfs partition*/
	if (target_is_emmc_boot())
		le_based = partition_get_index("recoveryfs");
	else
		/*Nand targets by default have this*/
		le_based = 1;

	if (le_based != -1)
	{
		if (!target_is_emmc_boot())
		{
			ptable = flash_get_ptable();
			if (!ptable)
			{
				dprintf(CRITICAL,
					"WARN: Cannot get flash partition table\n");
				return -1;
			}
			system_ptn_index = ptable_get_index(ptable, part);
		}
		else
			system_ptn_index = partition_get_index(part);
		if (system_ptn_index < 0) {
			dprintf(CRITICAL,
				"WARN: Cannot get partition index for %s\n", part);
			return -1;
		}
		/*
		* check if cmdline contains "root=" at the beginning of buffer or
		* " root=" in the middle of buffer.
		*/
		if (((!strncmp(cmdline, "root=", strlen("root="))) ||
			(strstr(cmdline, " root=")))) {
			dprintf(DEBUG, "DEBUG: cmdline has root=\n");
			return -1;
		}
		else
		/*in success case buf will be freed in the calling function of this*/
		{
			if (!target_is_emmc_boot())
			{
				/* Extra character is for Null termination */
				buflen = strlen(" root=/dev/mtdblock") + sizeof(int) +1;

				/* In success case, this memory is freed in calling function */
				*buf = (char *)malloc(buflen);
				if(!(*buf)) {
					dprintf(CRITICAL,"Unable to allocate memory for boot params \n");
					return -1;
				}

				snprintf(*buf, buflen, " root=/dev/mtdblock%d",system_ptn_index);
			}
			else
			{
				/* Extra character is for Null termination */
				buflen = strlen(" root=/dev/mmcblk0p") + sizeof(int) + 1;

				/* In success case, this memory is freed in calling function */
				*buf = (char *)malloc(buflen);
				if(!(*buf)) {
					dprintf(CRITICAL,"Unable to allocate memory for boot params \n");
					return -1;
				}

				/*For Emmc case increase the ptn_index by 1*/
				snprintf(*buf, buflen, " root=/dev/mmcblk0p%d",system_ptn_index + 1);
			}
		}

		/*Return for LE based Targets.*/
		return 0;
	}
	return -1;
}

unsigned target_baseband()
{
	return board_baseband();
}

int emmc_recovery_init(void)
{
	return _emmc_recovery_init();
}

void target_usb_init(void)
{
	uint32_t val;

	/* Select and enable external configuration with USB PHY */
	ulpi_write(ULPI_MISC_A_VBUSVLDEXTSEL | ULPI_MISC_A_VBUSVLDEXT, ULPI_MISC_A_SET);

	/* Enable sess_vld */
	val = readl(USB_GENCONFIG_2) | GEN2_SESS_VLD_CTRL_EN;
	writel(val, USB_GENCONFIG_2);

	/* Enable external vbus configuration in the LINK */
	val = readl(USB_USBCMD);
	val |= SESS_VLD_CTRL;
	writel(val, USB_USBCMD);
}

unsigned target_pause_for_battery_charge(void)
{
	uint8_t pon_reason = pm8x41_get_pon_reason();
	uint8_t is_cold_boot = pm8x41_get_is_cold_boot();
	dprintf(INFO, "%s : pon_reason is %d cold_boot:%d\n", __func__,
		pon_reason, is_cold_boot);
	/* In case of fastboot reboot,adb reboot or if we see the power key
	* pressed we do not want go into charger mode.
	* fastboot reboot is warm boot with PON hard reset bit not set
	* adb reboot is a cold boot with PON hard reset bit set
	*/
	if (is_cold_boot &&
			(!(pon_reason & HARD_RST)) &&
			(!(pon_reason & KPDPWR_N)) &&
			((pon_reason & USB_CHG) || (pon_reason & DC_CHG) || (pon_reason & CBLPWR_N)))
		return 1;
	else
		return 0;
}

void target_usb_stop(void)
{
	/* Disable VBUS mimicing in the controller. */
	ulpi_write(ULPI_MISC_A_VBUSVLDEXTSEL | ULPI_MISC_A_VBUSVLDEXT, ULPI_MISC_A_CLEAR);
}


void target_uninit(void)
{
#if PON_VIB_SUPPORT
	/* wait for the vibrator timer is expried */
	wait_vib_timeout();
#endif

	if (platform_boot_dev_isemmc())
	{
		mmc_put_card_to_sleep(dev);
		sdhci_mode_disable(&dev->host);
	}

	if (crypto_initialized())
		crypto_eng_cleanup();

	if (target_is_ssd_enabled())
		clock_ce_disable(CE1_INSTANCE);

#if SMD_SUPPORT
	rpm_smd_uninit();
#endif
}

/* Do any target specific intialization needed before entering fastboot mode */
void target_fastboot_init(void)
{
	/* Set the BOOT_DONE flag in PM8916 */
	pm8x41_set_boot_done();

	if (target_is_ssd_enabled()) {
		clock_ce_enable(CE1_INSTANCE);
		target_load_ssd_keystore();
	}
}

int set_download_mode(enum reboot_reason mode)
{
	int ret = 0;
	ret = scm_dload_mode(mode);

	pm8x41_clear_pmic_watchdog();

	return ret;
}

void target_load_ssd_keystore(void)
{
	uint64_t ptn;
	int      index;
	uint64_t size;
	uint32_t *buffer = NULL;

	if (!target_is_ssd_enabled())
		return;

	index = partition_get_index("ssd");

	ptn = partition_get_offset(index);
	if (ptn == 0){
		dprintf(CRITICAL, "Error: ssd partition not found\n");
		return;
	}

	size = partition_get_size(index);
	if (size == 0) {
		dprintf(CRITICAL, "Error: invalid ssd partition size\n");
		return;
	}

	buffer = memalign(CACHE_LINE, ROUNDUP(size, CACHE_LINE));
	if (!buffer) {
		dprintf(CRITICAL, "Error: allocating memory for ssd buffer\n");
		return;
	}
	if (mmc_read(ptn, buffer, size)) {
		dprintf(CRITICAL, "Error: cannot read data\n");
		free(buffer);
		return;
	}

	clock_ce_enable(CE1_INSTANCE);
	scm_protect_keystore(buffer, size);
	clock_ce_disable(CE1_INSTANCE);
	free(buffer);
}

crypto_engine_type board_ce_type(void)
{
	return CRYPTO_ENGINE_TYPE_HW;
}

/* Set up params for h/w CE. */
void target_crypto_init_params()
{
	struct crypto_init_params ce_params;

	/* Set up base addresses and instance. */
	ce_params.crypto_instance  = CE1_INSTANCE;
	ce_params.crypto_base      = MSM_CE1_BASE;
	ce_params.bam_base         = MSM_CE1_BAM_BASE;

	/* Set up BAM config. */
	ce_params.bam_ee               = CE_EE;
	ce_params.pipes.read_pipe      = CE_READ_PIPE;
	ce_params.pipes.write_pipe     = CE_WRITE_PIPE;
	ce_params.pipes.read_pipe_grp  = CE_READ_PIPE_LOCK_GRP;
	ce_params.pipes.write_pipe_grp = CE_WRITE_PIPE_LOCK_GRP;

	/* Assign buffer sizes. */
	ce_params.num_ce           = CE_ARRAY_SIZE;
	ce_params.read_fifo_size   = CE_FIFO_SIZE;
	ce_params.write_fifo_size  = CE_FIFO_SIZE;

	/* BAM is initialized by TZ for this platform.
	* Do not do it again as the initialization address space
	* is locked.
	*/
	ce_params.do_bam_init      = 0;

	crypto_init_params(&ce_params);
}

uint32_t target_get_hlos_subtype()
{
	return board_hlos_subtype();
}

void pmic_reset_configure(uint8_t reset_type)
{
	pm8x41_reset_configure(reset_type);
}

uint32_t target_get_pmic()
{
	return PMIC_IS_PM8909;
}
