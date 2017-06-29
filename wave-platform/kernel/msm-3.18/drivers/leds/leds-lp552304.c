/*
 * lp5523.c - LP5523, LP55231 LED Driver
 *
 * Copyright (C) 2010 Nokia Corporation
 * Copyright (C) 2012 Texas Instruments
 *
 * Contact: Samu Onkalo <samu.p.onkalo@nokia.com>
 *          Milo(Woogyom) Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_data/leds-lp55xx.h>
#include <linux/slab.h>

#include "leds-lp55xx-common.h"
#include "leds-lp5523.h"

//#define LED_ACTIVE(mux, led)		(!!(mux & (0x0001 << led)))

static u8 clear_pattern[LP5523_PATTERN_SIZE+1] = {0};
static u8 engine1_pattern[LP5523_PATTERN_SIZE+1] = {0};
static u8 engine2_pattern[LP5523_PATTERN_SIZE+1] = {0};
static u8 engine3_pattern[LP5523_PATTERN_SIZE+1] = {0};
static int engine1_pattern_count = 0;
static int engine2_pattern_count = 0;
static int engine3_pattern_count = 0;

static u8 engine1_reset_pattern[LP5523_PATTERN_SIZE+1] = {0};
static u8 engine2_reset_pattern[LP5523_PATTERN_SIZE+1] = {0};
static u8 engine3_reset_pattern[LP5523_PATTERN_SIZE+1] = {0};
static int engine1_reset_count = 0;
static int engine2_reset_count = 0;
static int engine3_reset_count = 0;
static u8 reg_op_mode, reg_enable;

static int lp5523_init_program_engine(struct lp55xx_chip *chip);

static inline void lp5523_wait_opmode_done(void)
{
	usleep_range(1000, 1100);
}

static void lp5523_set_led_current(struct lp55xx_led *led, u8 led_current)
{
	led->led_current = led_current;
	lp55xx_write(led->chip, LP5523_REG_LED_CURRENT_BASE + led->chan_nr,
		led_current);
}

static int lp5523_post_init_device(struct lp55xx_chip *chip)
{
	int ret;

	ret = lp55xx_write(chip, LP5523_REG_ENABLE, LP5523_ENABLE);
	if (ret)
		return ret;

	/* Chip startup time is 500 us, 1 - 2 ms gives some margin */
	usleep_range(1000, 2000);

	ret = lp55xx_write(chip, LP5523_REG_CONFIG,
			    LP5523_AUTO_INC | LP5523_PWR_SAVE |
			    LP5523_CP_AUTO | LP5523_AUTO_CLK |
			    LP5523_PWM_PWR_SAVE);
	if (ret)
		return ret;

	/* turn on all leds */
	ret = lp55xx_write(chip, LP5523_REG_ENABLE_LEDS_MSB, 0x01);
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5523_REG_ENABLE_LEDS_LSB, 0xff);
	if (ret)
		return ret;

	return lp5523_init_program_engine(chip);
}

static void lp5523_load_all_engine(struct lp55xx_chip *chip)
{
	lp55xx_update_bits(chip, LP5523_REG_OP_MODE, LP5523_MODE_M, LP5523_LOAD_ALL);
	lp5523_wait_opmode_done();
}

static void lp5523_load_engine(struct lp55xx_chip *chip)
{
	enum lp55xx_engine_index idx = chip->engine_idx;
	u8 mask[] = {
		[LP55XX_ENGINE_1] = LP5523_MODE_ENG1_M,
		[LP55XX_ENGINE_2] = LP5523_MODE_ENG2_M,
		[LP55XX_ENGINE_3] = LP5523_MODE_ENG3_M,
	};

	u8 val[] = {
		[LP55XX_ENGINE_1] = LP5523_LOAD_ENG1,
		[LP55XX_ENGINE_2] = LP5523_LOAD_ENG2,
		[LP55XX_ENGINE_3] = LP5523_LOAD_ENG3,
	};

	lp55xx_update_bits(chip, LP5523_REG_OP_MODE, mask[idx], val[idx]);

	lp5523_wait_opmode_done();
}

static void lp5523_load_engine_and_select_page(struct lp55xx_chip *chip)
{
	enum lp55xx_engine_index idx = chip->engine_idx;
	u8 page_sel[] = {
		[LP55XX_ENGINE_1] = LP5523_PAGE_ENG1,
		[LP55XX_ENGINE_2] = LP5523_PAGE_ENG2,
		[LP55XX_ENGINE_3] = LP5523_PAGE_ENG3,
	};

	lp5523_load_engine(chip);

	lp55xx_write(chip, LP5523_REG_PROG_PAGE_SEL, page_sel[idx]);
}

static void lp5523_stop_all_engines(struct lp55xx_chip *chip)
{
	lp55xx_write(chip, LP5523_REG_OP_MODE, 0);
	lp5523_wait_opmode_done();
}

static void lp5523_stop_engine(struct lp55xx_chip *chip)
{
	enum lp55xx_engine_index idx = chip->engine_idx;
	u8 mask[] = {
		[LP55XX_ENGINE_1] = LP5523_MODE_ENG1_M,
		[LP55XX_ENGINE_2] = LP5523_MODE_ENG2_M,
		[LP55XX_ENGINE_3] = LP5523_MODE_ENG3_M,
	};

	lp55xx_update_bits(chip, LP5523_REG_OP_MODE, mask[idx], 0);

	//lp5523_wait_opmode_done();
}

static void lp5523_turn_off_channels(struct lp55xx_chip *chip)
{
	int i;

	for (i = 0; i < LP5523_MAX_LEDS; i++)
		lp55xx_write(chip, LP5523_REG_LED_PWM_BASE + i, 0);
}

static void lp5523_run_engine(struct lp55xx_chip *chip, bool start)
{
	int ret;
	u8 mode;
	u8 exec;

	/* stop engine */
	if (!start) {
		lp5523_stop_engine(chip);
		lp5523_turn_off_channels(chip);
		return;
	}

	/*
	 * To run the engine,
	 * operation mode and enable register should updated at the same time
	 */

	ret = lp55xx_read(chip, LP5523_REG_OP_MODE, &mode);
	if (ret)
		return;

	ret = lp55xx_read(chip, LP5523_REG_ENABLE, &exec);
	if (ret)
		return;

	/* change operation mode to RUN only when each engine is loading */
	if (LP5523_ENG1_IS_LOADING(mode)) {
		mode = (mode & ~LP5523_MODE_ENG1_M) | LP5523_RUN_ENG1;
		exec = (exec & ~LP5523_EXEC_ENG1_M) | LP5523_RUN_ENG1;
	}

	if (LP5523_ENG2_IS_LOADING(mode)) {
		mode = (mode & ~LP5523_MODE_ENG2_M) | LP5523_RUN_ENG2;
		exec = (exec & ~LP5523_EXEC_ENG2_M) | LP5523_RUN_ENG2;
	}

	if (LP5523_ENG3_IS_LOADING(mode)) {
		mode = (mode & ~LP5523_MODE_ENG3_M) | LP5523_RUN_ENG3;
		exec = (exec & ~LP5523_EXEC_ENG3_M) | LP5523_RUN_ENG3;
	}

	lp55xx_write(chip, LP5523_REG_OP_MODE, mode);
	//lp5523_wait_opmode_done();

	lp55xx_update_bits(chip, LP5523_REG_ENABLE, LP5523_EXEC_M, exec);
}

static int lp5523_init_program_engine(struct lp55xx_chip *chip)
{
	int i;
	int j;
	int ret;
	u8 status;
	/* one pattern per engine setting LED MUX start and stop addresses */
	static const u8 pattern[][LP5523_PROGRAM_LENGTH] =  {
		{ 0x9c, 0x30, 0x9c, 0xb0, 0x9d, 0x80, 0xd8, 0x00, 0},
		{ 0x9c, 0x40, 0x9c, 0xc0, 0x9d, 0x80, 0xd8, 0x00, 0},
		{ 0x9c, 0x50, 0x9c, 0xd0, 0x9d, 0x80, 0xd8, 0x00, 0},
	};

	/* hardcode 32 bytes of memory for each engine from program memory */
	ret = lp55xx_write(chip, LP5523_REG_CH1_PROG_START, 0x00);
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5523_REG_CH2_PROG_START, 0x10);
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5523_REG_CH3_PROG_START, 0x20);
	if (ret)
		return ret;

	/* write LED MUX address space for each engine */
	for (i = LP55XX_ENGINE_1; i <= LP55XX_ENGINE_3; i++) {
		chip->engine_idx = i;
		lp5523_load_engine_and_select_page(chip);

		for (j = 0; j < LP5523_PROGRAM_LENGTH; j++) {
			ret = lp55xx_write(chip, LP5523_REG_PROG_MEM + j,
					pattern[i - 1][j]);
			if (ret)
				goto out;
		}
	}

	lp5523_run_engine(chip, true);

	/* Let the programs run for couple of ms and check the engine status */
	usleep_range(3000, 6000);
	lp55xx_read(chip, LP5523_REG_STATUS, &status);
	status &= LP5523_ENG_STATUS_MASK;

	if (status != LP5523_ENG_STATUS_MASK) {
		dev_err(&chip->cl->dev,
			"cound not configure LED engine, status = 0x%.2x\n",
			status);
		ret = -1;
	}

out:
	lp5523_stop_all_engines(chip);
	return ret;
}

static int lp5523_update_program_memory(struct lp55xx_chip *chip,
					const u8 *data, size_t size)
{
	int ret;
	enum lp55xx_engine_index idx = chip->engine_idx;

	if (!strncmp(data, "turn_on", 7)) {
		if (idx == 1) {
			ret = lp55xx_write_block(chip, LP5523_REG_PROG_PAGE_SEL, engine1_pattern_count+1, engine1_pattern);
			if (ret<0)
				goto err;
		} else if (idx == 2) {
			ret = lp55xx_write_block(chip, LP5523_REG_PROG_PAGE_SEL, engine2_pattern_count+1, engine2_pattern);
			if (ret<0)
				goto err;
		} else if (idx == 3) {
			ret = lp55xx_write_block(chip, LP5523_REG_PROG_PAGE_SEL, engine3_pattern_count+1, engine3_pattern);
			if (ret<0)
				goto err;
		}
	} else if (!strncmp(data, "turn_off", 8)) {
		if (idx == 1) {
			ret = lp55xx_write_block(chip, LP5523_REG_PROG_PAGE_SEL, engine1_reset_count+1, engine1_reset_pattern);
			if (ret<0)
				goto err;
		} else if (idx == 2) {
			ret = lp55xx_write_block(chip, LP5523_REG_PROG_PAGE_SEL, engine2_reset_count+1, engine2_reset_pattern);
			if (ret<0)
				goto err;
		} else if (idx == 3) {
			ret = lp55xx_write_block(chip, LP5523_REG_PROG_PAGE_SEL, engine3_reset_count+1, engine3_reset_pattern);
			if (ret<0)
				goto err;
		}
	} else if (!strncmp(data, "clear", 5)) {
		if (idx == 1) {
			clear_pattern[0] = 0;
			ret = lp55xx_write_block(chip, LP5523_REG_PROG_PAGE_SEL, engine1_pattern_count+1, clear_pattern);
			if (ret<0)
				goto err;
			memset(engine1_pattern, 0, sizeof(engine1_pattern));
			memset(engine1_reset_pattern, 0, sizeof(engine1_reset_pattern));
		} else if (idx == 2) {
			clear_pattern[0] = 1;
			ret = lp55xx_write_block(chip, LP5523_REG_PROG_PAGE_SEL, engine2_pattern_count+1, clear_pattern);
			if (ret<0)
				goto err;
			memset(engine2_pattern, 0, sizeof(engine2_pattern));
			memset(engine2_reset_pattern, 0, sizeof(engine2_reset_pattern));
			engine2_pattern[0] = 1;
			engine2_reset_pattern[0] = 1;
		} else if (idx == 3) {
			clear_pattern[0] = 2;
			ret = lp55xx_write_block(chip, LP5523_REG_PROG_PAGE_SEL, engine3_pattern_count+1, clear_pattern);
			if (ret<0)
				goto err;
			memset(engine3_pattern, 0, sizeof(engine3_pattern));
			memset(engine3_reset_pattern, 0, sizeof(engine3_reset_pattern));
			engine3_pattern[0] = 2;
			engine3_reset_pattern[0] = 2;
		}
	} else if (!strncmp(data, "initial", 7)) {
		engine1_pattern[0] = 0;
		engine2_pattern[0] = 1;
		engine3_pattern[0] = 2;
		engine1_reset_pattern[0] = 0;
		engine2_reset_pattern[0] = 1;
		engine3_reset_pattern[0] = 2;

		clear_pattern[0] = 0;
		ret = lp55xx_write_block(chip, LP5523_REG_PROG_PAGE_SEL, 34, clear_pattern);
		clear_pattern[0] = 1; //change page
		ret = lp55xx_write_block(chip, LP5523_REG_PROG_PAGE_SEL, 34, clear_pattern);
		clear_pattern[0] = 2;
		ret = lp55xx_write_block(chip, LP5523_REG_PROG_PAGE_SEL, 34, clear_pattern);
		if (ret<0)
			goto err;
	}

	return size;

err:
	dev_err(&chip->cl->dev, "wrong pattern format\n");
	return -EINVAL;
}

static void lp5523_control_engine(struct lp55xx_chip *chip, u8 *action)
{
	enum lp55xx_engine_index idx = chip->engine_idx;

	u8 mask[] = {
		[LP55XX_ENGINE_1] = LP5523_MODE_ENG1_M,
		[LP55XX_ENGINE_2] = LP5523_MODE_ENG2_M,
		[LP55XX_ENGINE_3] = LP5523_MODE_ENG3_M,
	};

	u8 val[] = {
		[LP55XX_ENGINE_1] = LP5523_LOAD_ENG1,
		[LP55XX_ENGINE_2] = LP5523_LOAD_ENG2,
		[LP55XX_ENGINE_3] = LP5523_LOAD_ENG3,
	};

	/*****STOP ENGINE*****/
	reg_op_mode &= ~mask[idx];
	reg_op_mode |= 0 & mask[idx];
	lp55xx_write(chip, LP5523_REG_OP_MODE, reg_op_mode);
	udelay(100);

	/*****LOAD ENGINE*****/
	reg_op_mode &= ~mask[idx];
	reg_op_mode |= val[idx] & mask[idx];
	lp55xx_write(chip, LP5523_REG_OP_MODE, reg_op_mode);
	lp5523_wait_opmode_done();

	/*****WRITE SRAM*****/
	lp5523_update_program_memory(chip, action, 7);

	/*****RUN ENGINE*****/
	/* change operation mode to RUN only when each engine is loading */
	if (LP5523_ENG1_IS_LOADING(reg_op_mode)) {
		reg_op_mode = (reg_op_mode & ~LP5523_MODE_ENG1_M) | LP5523_RUN_ENG1;
		reg_enable= (reg_enable & ~LP5523_EXEC_ENG1_M) | LP5523_RUN_ENG1;
	}
	if (LP5523_ENG2_IS_LOADING(reg_op_mode)) {
		reg_op_mode = (reg_op_mode & ~LP5523_MODE_ENG2_M) | LP5523_RUN_ENG2;
		reg_enable = (reg_enable & ~LP5523_EXEC_ENG2_M) | LP5523_RUN_ENG2;
	}
	if (LP5523_ENG3_IS_LOADING(reg_op_mode)) {
		reg_op_mode = (reg_op_mode & ~LP5523_MODE_ENG3_M) | LP5523_RUN_ENG3;
		reg_enable = (reg_enable & ~LP5523_EXEC_ENG3_M) | LP5523_RUN_ENG3;
	}
	lp55xx_write(chip, LP5523_REG_OP_MODE, reg_op_mode);
	udelay(100);
	lp55xx_write(chip, LP5523_REG_ENABLE, reg_enable);
}

#if 0
static void lp5523_firmware_loaded(struct lp55xx_chip *chip)
{
	const struct firmware *fw = chip->fw;

	if (fw->size > LP5523_PROGRAM_LENGTH) {
		dev_err(&chip->cl->dev, "firmware data size overflow: %zu\n",
			fw->size);
		return;
	}

	/*
	 * Program momery sequence
	 *  1) set engine mode to "LOAD"
	 *  2) write firmware data into program memory
	 */

	lp5523_load_engine_and_select_page(chip);
	lp5523_update_program_memory(chip, fw->data, fw->size);
}
#endif

static int switch_engine_load(struct lp55xx_chip *chip,
			     const u8 *data, size_t len, int nr)
{
	int ret;

	chip->engine_idx = nr;
	lp5523_load_engine_and_select_page(chip);
	ret = lp5523_update_program_memory(chip, data, len);

	return ret;
}

static ssize_t show_engine_mode(struct device *dev,
				struct device_attribute *attr,
				char *buf, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	enum lp55xx_engine_mode mode = chip->engines[nr - 1].mode;

	switch (mode) {
		case LP55XX_ENGINE_RUN:
			return sprintf(buf, "run\n");
		case LP55XX_ENGINE_LOAD:
			return sprintf(buf, "load\n");
		case LP55XX_ENGINE_ON:
			return sprintf(buf, "on\n");
		case LP55XX_ENGINE_OFF:
			return sprintf(buf, "off\n");
		case LP55XX_ENGINE_CLEAR:
			return sprintf(buf, "clear\n");
		case LP55XX_ENGINE_DISABLED:
		default:
			return sprintf(buf, "disabled\n");
	}
}
show_mode(1)
show_mode(2)
show_mode(3)

static ssize_t store_engine_mode(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_engine *engine = &chip->engines[nr - 1];

	mutex_lock(&chip->lock);

	chip->engine_idx = nr;

	if (!strncmp(buf, "run", 3)) {
		lp5523_run_engine(chip, true);
		engine->mode = LP55XX_ENGINE_RUN;
	} else if (!strncmp(buf, "load", 4)) {
		lp5523_stop_engine(chip);
		lp5523_load_engine(chip);
		engine->mode = LP55XX_ENGINE_LOAD;
	} else if (!strncmp(buf, "disabled", 8)) {
		lp5523_stop_engine(chip);
		engine->mode = LP55XX_ENGINE_DISABLED;
	} else if (!strncmp(buf, "on", 2)) {
		lp5523_control_engine( chip, "turn_on");
		engine->mode = LP55XX_ENGINE_ON;
	} else if (!strncmp(buf, "off", 3)) {
		lp5523_control_engine( chip, "turn_off");
		engine->mode = LP55XX_ENGINE_OFF;
	} else if (!strncmp(buf, "clear", 5)) {
		lp5523_control_engine( chip, "clear");
		engine->mode = LP55XX_ENGINE_CLEAR;
	}

	mutex_unlock(&chip->lock);

	return len;
}
store_mode(1)
store_mode(2)
store_mode(3)

#if 0
static int lp5523_mux_parse(const char *buf, u16 *mux, size_t len)
{
	u16 tmp_mux = 0;
	int i;

	len = min_t(int, len, LP5523_MAX_LEDS);

	for (i = 0; i < len; i++) {
		switch (buf[i]) {
		case '1':
			tmp_mux |= (1 << i);
			break;
		case '0':
			break;
		case '\n':
			i = len;
			break;
		default:
			return -1;
		}
	}
	*mux = tmp_mux;

	return 0;
}

static void lp5523_mux_to_array(u16 led_mux, char *array)
{
	int i, pos = 0;
	for (i = 0; i < LP5523_MAX_LEDS; i++)
		pos += sprintf(array + pos, "%x", LED_ACTIVE(led_mux, i));

	array[pos] = '\0';
}

static ssize_t show_engine_leds(struct device *dev,
			    struct device_attribute *attr,
			    char *buf, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	char mux[LP5523_MAX_LEDS + 1];

	lp5523_mux_to_array(chip->engines[nr - 1].led_mux, mux);

	return sprintf(buf, "%s\n", mux);
}
show_leds(1)
show_leds(2)
show_leds(3)

static int lp5523_load_mux(struct lp55xx_chip *chip, u16 mux, int nr)
{
	struct lp55xx_engine *engine = &chip->engines[nr - 1];
	int ret;
	u8 mux_page[] = {
		[LP55XX_ENGINE_1] = LP5523_PAGE_MUX1,
		[LP55XX_ENGINE_2] = LP5523_PAGE_MUX2,
		[LP55XX_ENGINE_3] = LP5523_PAGE_MUX3,
	};

	lp5523_load_engine(chip);

	ret = lp55xx_write(chip, LP5523_REG_PROG_PAGE_SEL, mux_page[nr]);
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5523_REG_PROG_MEM , (u8)(mux >> 8));
	if (ret)
		return ret;

	ret = lp55xx_write(chip, LP5523_REG_PROG_MEM + 1, (u8)(mux));
	if (ret)
		return ret;

	engine->led_mux = mux;
	return 0;
}

static ssize_t store_engine_leds(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_engine *engine = &chip->engines[nr - 1];
	u16 mux = 0;
	ssize_t ret;

	if (lp5523_mux_parse(buf, &mux, len))
		return -EINVAL;

	mutex_lock(&chip->lock);

	chip->engine_idx = nr;
	ret = -EINVAL;

	if (engine->mode != LP55XX_ENGINE_LOAD)
		goto leave;

	if (lp5523_load_mux(chip, mux, nr))
		goto leave;

	ret = len;
leave:
	mutex_unlock(&chip->lock);
	return ret;
}
store_leds(1)
store_leds(2)
store_leds(3)
#endif

static ssize_t store_engine_load(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len, int nr)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	int ret;

	mutex_lock(&chip->lock);

	chip->engine_idx = nr;
	lp5523_load_engine_and_select_page(chip);
	ret = lp5523_update_program_memory(chip, buf, len);

	mutex_unlock(&chip->lock);

	return ret;
}
store_load(1)
store_load(2)
store_load(3)

static ssize_t set_led_pwm(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	u8 led_color[3] = {0};
	char *token[10];
	int i=0, engine_index;
	unsigned long val;

	mutex_lock(&chip->lock);

	for (i = 0; i < 4; i++) {
		token[i] = strsep((char **)&buf, ",");

		kstrtoul(token[i], 0, &val);
		if (i==0)
			engine_index = val;
		else
			led_color[i-1] = (u8)val;
	}

	if (engine_index == 1) {
		engine1_pattern_count = 0;
		engine1_reset_count = 0;

		for (i=1; i<=3; i++) {
			engine1_pattern[++engine1_pattern_count] = 0x9D;
			engine1_pattern[++engine1_pattern_count] = (engine_index-1)*3+(4-i);
			engine1_pattern[++engine1_pattern_count] = 0x40;
			engine1_pattern[++engine1_pattern_count] = led_color[i-1];

			engine1_reset_pattern[++engine1_reset_count] = 0x9D;
			engine1_reset_pattern[++engine1_reset_count] = (engine_index-1)*3+(4-i);
			engine1_reset_pattern[++engine1_reset_count] = 0x40;
			engine1_reset_pattern[++engine1_reset_count] = 0;
		}
	} else if (engine_index == 2) {
		engine2_pattern_count= 0;
		engine2_reset_count= 0;

		for (i=1; i<=3; i++) {
			engine2_pattern[++engine2_pattern_count] = 0x9D;
			engine2_pattern[++engine2_pattern_count] = (engine_index-1)*3+(4-i);
			engine2_pattern[++engine2_pattern_count] = 0x40;
			engine2_pattern[++engine2_pattern_count] = led_color[i-1];

			engine2_reset_pattern[++engine2_reset_count] = 0x9D;
			engine2_reset_pattern[++engine2_reset_count] = (engine_index-1)*3+(4-i);
			engine2_reset_pattern[++engine2_reset_count] = 0x40;
			engine2_reset_pattern[++engine2_reset_count] = 0;
		}
	} else if (engine_index == 3) {
		engine3_pattern_count = 0;
		engine3_reset_count = 0;

		for (i=1; i<=3; i++) {
			engine3_pattern[++engine3_pattern_count] = 0x9D;
			engine3_pattern[++engine3_pattern_count] = (engine_index-1)*3+(4-i);
			engine3_pattern[++engine3_pattern_count] = 0x40;
			engine3_pattern[++engine3_pattern_count] = led_color[i-1];

			engine3_reset_pattern[++engine3_reset_count] = 0x9D;
			engine3_reset_pattern[++engine3_reset_count] = (engine_index-1)*3+(4-i);
			engine3_reset_pattern[++engine3_reset_count] = 0x40;
			engine3_reset_pattern[++engine3_reset_count] = 0;
		}
	}

	mutex_unlock(&chip->lock);

	return len;
}

static ssize_t set_led_trigger_pwm(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	u8 led_color[3] = {0};
	char *token[10];
	int i=0, engine_index;
	unsigned long val;

	mutex_lock(&chip->lock);

	for (i = 0; i < 4; i++) {
		token[i] = strsep((char **)&buf, ",");

		kstrtoul(token[i], 0, &val);
		if (i==0)
			engine_index = val;
		else
			led_color[i-1] = (u8)val;
	}

	if (engine_index == 1) {
		engine1_pattern_count = 0;
		engine1_reset_count = 0;

		engine1_pattern[++engine1_pattern_count] = 0xF0;
		engine1_pattern[++engine1_pattern_count] = 0x00;

		engine1_reset_pattern[++engine1_reset_count] = 0xF0;
		engine1_reset_pattern[++engine1_reset_count] = 0x00;

		for (i=1; i<=3; i++) {
			engine1_pattern[++engine1_pattern_count] = 0x9D;
			engine1_pattern[++engine1_pattern_count] = (engine_index-1)*3+(4-i);
			engine1_pattern[++engine1_pattern_count] = 0x40;
			engine1_pattern[++engine1_pattern_count] = led_color[i-1];

			engine1_reset_pattern[++engine1_reset_count] = 0x9D;
			engine1_reset_pattern[++engine1_reset_count] = (engine_index-1)*3+(4-i);
			engine1_reset_pattern[++engine1_reset_count] = 0x40;
			engine1_reset_pattern[++engine1_reset_count] = 0;
		}
	} else if (engine_index == 2) {
		engine2_pattern_count = 0;
		engine2_reset_count = 0;

		engine2_pattern[++engine2_pattern_count] = 0xF0;
		engine2_pattern[++engine2_pattern_count] = 0x00;

		engine2_reset_pattern[++engine2_reset_count] = 0xF0;
		engine2_reset_pattern[++engine2_reset_count] = 0x00;

		for (i=1; i<=3; i++) {
			engine2_pattern[++engine2_pattern_count] = 0x9D;
			engine2_pattern[++engine2_pattern_count] = (engine_index-1)*3+(4-i);
			engine2_pattern[++engine2_pattern_count] = 0x40;
			engine2_pattern[++engine2_pattern_count] = led_color[i-1];

			engine2_reset_pattern[++engine2_reset_count] = 0x9D;
			engine2_reset_pattern[++engine2_reset_count] = (engine_index-1)*3+(4-i);
			engine2_reset_pattern[++engine2_reset_count] = 0x40;
			engine2_reset_pattern[++engine2_reset_count] = 0;
		}
	} else if (engine_index == 3) {
		engine3_pattern_count = 0;
		engine3_reset_count= 0;

		engine3_pattern[++engine3_pattern_count] =0xF0;
		engine3_pattern[++engine3_pattern_count] =0x00;

		engine3_reset_pattern[++engine3_reset_count] = 0xF0;
		engine3_reset_pattern[++engine3_reset_count] = 0x00;

		for (i=1; i<=3; i++) {
			engine3_pattern[++engine3_pattern_count] = 0x9D;
			engine3_pattern[++engine3_pattern_count] = (engine_index-1)*3+(4-i);
			engine3_pattern[++engine3_pattern_count] = 0x40;
			engine3_pattern[++engine3_pattern_count] = led_color[i-1];

			engine3_reset_pattern[++engine3_reset_count] = 0x9D;
			engine3_reset_pattern[++engine3_reset_count] = (engine_index-1)*3+(4-i);
			engine3_reset_pattern[++engine3_reset_count] = 0x40;
			engine3_reset_pattern[++engine3_reset_count] = 0;
		}
	}

	mutex_unlock(&chip->lock);
	return len;
}

static ssize_t lp5523_selftest(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct lp55xx_led *led = i2c_get_clientdata(to_i2c_client(dev));
	struct lp55xx_chip *chip = led->chip;
	struct lp55xx_platform_data *pdata = chip->pdata;
	int i, ret, pos = 0;
	u8 status, adc, vdd;

	mutex_lock(&chip->lock);

	ret = lp55xx_read(chip, LP5523_REG_STATUS, &status);
	if (ret < 0)
		goto fail;

	/* Check that ext clock is really in use if requested */
	if (pdata->clock_mode == LP55XX_CLOCK_EXT) {
		if  ((status & LP5523_EXT_CLK_USED) == 0)
			goto fail;
	}

	/* Measure VDD (i.e. VBAT) first (channel 16 corresponds to VDD) */
	lp55xx_write(chip, LP5523_REG_LED_TEST_CTRL, LP5523_EN_LEDTEST | 16);
	usleep_range(3000, 6000); /* ADC conversion time is typically 2.7 ms */
	ret = lp55xx_read(chip, LP5523_REG_STATUS, &status);
	if (ret < 0)
		goto fail;

	if (!(status & LP5523_LEDTEST_DONE))
		usleep_range(3000, 6000); /* Was not ready. Wait little bit */

	ret = lp55xx_read(chip, LP5523_REG_LED_TEST_ADC, &vdd);
	if (ret < 0)
		goto fail;

	vdd--;	/* There may be some fluctuation in measurement */

	for (i = 0; i < LP5523_MAX_LEDS; i++) {
		/* Skip non-existing channels */
		if (pdata->led_config[i].led_current == 0)
			continue;

		/* Set default current */
		lp55xx_write(chip, LP5523_REG_LED_CURRENT_BASE + i,
			pdata->led_config[i].led_current);

		lp55xx_write(chip, LP5523_REG_LED_PWM_BASE + i, 0xff);
		/* let current stabilize 2 - 4ms before measurements start */
		usleep_range(2000, 4000);
		lp55xx_write(chip, LP5523_REG_LED_TEST_CTRL,
			     LP5523_EN_LEDTEST | i);
		/* ADC conversion time is 2.7 ms typically */
		usleep_range(3000, 6000);
		ret = lp55xx_read(chip, LP5523_REG_STATUS, &status);
		if (ret < 0)
			goto fail;

		if (!(status & LP5523_LEDTEST_DONE))
			usleep_range(3000, 6000);/* Was not ready. Wait. */

		ret = lp55xx_read(chip, LP5523_REG_LED_TEST_ADC, &adc);
		if (ret < 0)
			goto fail;

		if (adc >= vdd || adc < LP5523_ADC_SHORTCIRC_LIM)
			pos += sprintf(buf + pos, "LED %d FAIL\n", i);

		lp55xx_write(chip, LP5523_REG_LED_PWM_BASE + i, 0x00);

		/* Restore current */
		lp55xx_write(chip, LP5523_REG_LED_CURRENT_BASE + i,
			led->led_current);
		led++;
	}
	if (pos == 0)
		pos = sprintf(buf, "OK\n");
	goto release_lock;
fail:
	pos = sprintf(buf, "FAIL\n");

release_lock:
	mutex_unlock(&chip->lock);

	return pos;
}

static void lp5523_led_brightness_work(struct work_struct *work)
{
	struct lp55xx_led *led = container_of(work, struct lp55xx_led,
					      brightness_work);
	struct lp55xx_chip *chip = led->chip;

	mutex_lock(&chip->lock);
	lp55xx_write(chip, LP5523_REG_LED_PWM_BASE + led->chan_nr,
		     led->brightness);
	mutex_unlock(&chip->lock);
}

static LP55XX_DEV_ATTR_RW(engine1_mode, show_engine1_mode, store_engine1_mode);
static LP55XX_DEV_ATTR_RW(engine2_mode, show_engine2_mode, store_engine2_mode);
static LP55XX_DEV_ATTR_RW(engine3_mode, show_engine3_mode, store_engine3_mode);
/*static LP55XX_DEV_ATTR_RW(engine1_leds, show_engine1_leds, store_engine1_leds);
static LP55XX_DEV_ATTR_RW(engine2_leds, show_engine2_leds, store_engine2_leds);
static LP55XX_DEV_ATTR_RW(engine3_leds, show_engine3_leds, store_engine3_leds);*/
static LP55XX_DEV_ATTR_WO(engine1_load, store_engine1_load);
static LP55XX_DEV_ATTR_WO(engine2_load, store_engine2_load);
static LP55XX_DEV_ATTR_WO(engine3_load, store_engine3_load);
static LP55XX_DEV_ATTR_WO(set_pwm, set_led_pwm);
static LP55XX_DEV_ATTR_WO(set_trigger_pwm, set_led_trigger_pwm);
static LP55XX_DEV_ATTR_RO(selftest, lp5523_selftest);

static struct attribute *lp5523_attributes[] = {
	&dev_attr_engine1_mode.attr,
	&dev_attr_engine2_mode.attr,
	&dev_attr_engine3_mode.attr,
	&dev_attr_engine1_load.attr,
	&dev_attr_engine2_load.attr,
	&dev_attr_engine3_load.attr,
	&dev_attr_set_pwm.attr,
	&dev_attr_set_trigger_pwm.attr,
	/*&dev_attr_engine1_leds.attr,
	&dev_attr_engine2_leds.attr,
	&dev_attr_engine3_leds.attr,*/
	&dev_attr_selftest.attr,
	NULL,
};

static const struct attribute_group lp5523_group = {
	.attrs = lp5523_attributes,
};

/* Chip specific configurations */
static struct lp55xx_device_config lp5523_cfg = {
	.reset = {
		.addr = LP5523_REG_RESET,
		.val  = LP5523_RESET,
	},
	.enable = {
		.addr = LP5523_REG_ENABLE,
		.val  = LP5523_ENABLE,
	},
	.max_channel  = LP5523_MAX_LEDS,
	.post_init_device   = lp5523_post_init_device,
	.brightness_work_fn = lp5523_led_brightness_work,
	.set_led_current    = lp5523_set_led_current,
	//.firmware_cb        = lp5523_firmware_loaded,
	.run_engine         = lp5523_run_engine,
	.dev_attr_group     = &lp5523_group,
};

static struct lp55xx_led_config rx51_lp5523_led_config[] = {
	{
		.name		= "lp552304:D1B",
		.chan_nr	= 0,
		.led_current	= 100,
		.max_current    = 200,
	}, {
		.name		= "lp552304:D1G",
		.chan_nr	= 1,
		.led_current	= 100,
		.max_current    = 200,
	}, {
		.name		= "lp552304:D1R",
		.chan_nr	= 2,
		.led_current	= 100,
		.max_current    = 200,
	}, {
		.name		= "lp552304:D2B",
		.chan_nr	= 3,
		.led_current	= 100,
		.max_current    = 200,
	}, {
		.name		= "lp552304:D2G",
		.chan_nr	= 4,
		.led_current	= 100,
		.max_current    = 200,
	}, {
		.name		= "lp552304:D2R",
		.chan_nr	= 5,
		.led_current	= 100,
		.max_current    = 200,
	}, {
		.name		= "lp552304:D3B",
		.chan_nr	= 6,
		.led_current	= 100,
		.max_current    = 200,
	}, {
		.name		= "lp552304:D3G",
		.chan_nr	= 7,
		.led_current	= 100,
		.max_current    = 200,
	}, {
		.name		= "lp552304:D3R",
		.chan_nr	= 8,
		.led_current	= 100,
		.max_current    = 200,
	}
};

static struct lp55xx_platform_data rx51_lp5523_platform_data = {
	.led_config		= rx51_lp5523_led_config,
	.num_channels	= ARRAY_SIZE(rx51_lp5523_led_config),
	.clock_mode		= LP55XX_CLOCK_INT,//LP55XX_CLOCK_AUTO,
};

static int lp5523_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	struct lp55xx_chip *chip;
	struct lp55xx_led *led;
	struct lp55xx_platform_data *pdata = NULL;
	//struct device_node *np = client->dev.of_node;

	/*if (!dev_get_platdata(&client->dev)) {
		if (np) {
			ret = lp55xx_of_populate_pdata(&client->dev, np);
			if (ret < 0)
				return ret;
		} else {
			dev_err(&client->dev, "no platform data\n");
			return -EINVAL;
		}
	}
	pdata = dev_get_platdata(&client->dev);*/
	client->dev.platform_data = &rx51_lp5523_platform_data;
	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "no platform data\n");
		return -EINVAL;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	led = devm_kzalloc(&client->dev,
			sizeof(*led) * pdata->num_channels, GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	chip->cl = client;
	chip->pdata = pdata;
	chip->cfg = &lp5523_cfg;

	mutex_init(&chip->lock);

	i2c_set_clientdata(client, led);

	ret = lp55xx_init_device(chip);
	if (ret)
		goto err_init;

	dev_info(&client->dev, "%s Programmable led chip found\n", id->name);

	ret = lp55xx_register_leds(led, chip);
	if (ret)
		goto err_register_leds;

	ret = lp55xx_register_sysfs(chip);
	if (ret) {
		dev_err(&client->dev, "registering sysfs failed\n");
		goto err_register_sysfs;
	}

	lp5523_stop_all_engines(chip);
	lp5523_load_all_engine(chip);
	lp5523_update_program_memory(chip, "initial", 7);
	lp5523_run_engine(chip, true);
	lp5523_stop_all_engines(chip);
	lp55xx_read(chip, LP5523_REG_ENABLE, &reg_enable);
	lp55xx_read(chip, LP5523_REG_OP_MODE, &reg_op_mode);

	return 0;

err_register_sysfs:
	lp55xx_unregister_leds(led, chip);
err_register_leds:
	lp55xx_deinit_device(chip);
err_init:
	return -EPROBE_DEFER;
}

static int lp5523_remove(struct i2c_client *client)
{
	struct lp55xx_led *led = i2c_get_clientdata(client);
	struct lp55xx_chip *chip = led->chip;

	lp5523_stop_all_engines(chip);
	lp55xx_unregister_sysfs(chip);
	lp55xx_unregister_leds(led, chip);
	lp55xx_deinit_device(chip);

	return 0;
}

static const struct i2c_device_id lp5523_id[] = {
	{ "lp552304", lp552304 },
	{ }
};

static const struct of_device_id lp55231_dt_match[] = {
	{.compatible = "ti,lp552304",},
	{}
};

MODULE_DEVICE_TABLE(i2c, lp5523_id);

static struct i2c_driver lp5523_driver = {
	.driver = {
		.name	= "lp552304",
		.of_match_table = lp55231_dt_match,
	},
	.probe		= lp5523_probe,
	.remove		= lp5523_remove,
	.id_table	= lp5523_id,
};

module_i2c_driver(lp5523_driver);

MODULE_AUTHOR("Mathias Nyman <mathias.nyman@nokia.com>");
MODULE_AUTHOR("Milo Kim <milo.kim@ti.com>");
MODULE_DESCRIPTION("LP5523 LED engine");
MODULE_LICENSE("GPL");
