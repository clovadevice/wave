/*
 * bq2477x-charger.c -- BQ24775 Charger driver
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Andy Park <andyp@nvidia.com>
 * Author: Syed Rafiuddin <srafiuddin@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power/bq2477x-charger.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/alarmtimer.h>
#include <linux/sched/rt.h>
#include <linux/pm.h>
#include <linux/input.h>
#include <linux/wakelock.h>
struct i2c_client *client_g;
struct input_dev *bq2477x_input;

struct bq2477x_chip {
	struct device	*dev;
	struct power_supply	ac;
	struct regmap	*regmap;
	struct regmap	*regmap_word;
	struct mutex	mutex;
	int	irq;
	int	gpio;
	int	ac_online;
	int	dac_ichg;
	int	dac_v;
	int	dac_minsv;
	int	dac_iin;
	int	suspended;
	int	wdt_timer;
	int	wdt_refresh_timeout;
	int	charge_current_now;
	int	charge_voltage_now;
	struct kthread_worker	bq_kworker;
	struct task_struct	*bq_kworker_task;
	struct kthread_work	bq_wdt_work;
	struct wake_lock		bq_wakelock;
	struct timer_list timer;
};

/* Kthread scheduling parameters */
struct sched_param bq2477x_param = {
	.sched_priority = MAX_RT_PRIO - 1,
};

static const struct regmap_config bq2477x_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= BQ2477X_MAX_REGS,
};

static const struct regmap_config bq2477x_regmap_word_config = {
	.reg_bits	= 8,
	.val_bits	= 16,
	.max_register	= BQ2477X_MAX_REGS,
};

/* Charge current limit */
static const unsigned int dac_ichg[] = {
	64, 128, 256, 512, 1024, 2048, 4096,
};

/* Output charge regulation voltage */
static const unsigned int dac_v[] = {
	16, 32, 64, 128, 256, 512, 1024, 2048,
	4096, 8192, 16384,
};

/* Minimum system votlage */
static const unsigned int dac_minsv[] = {
	256, 512, 1024, 2048, 4096, 8192,
};

/* Setting input current */
static const unsigned int dac_iin[] = {
	64, 128, 256, 512, 1024, 2048, 4096,
};

static int bq2477x_read(struct bq2477x_chip *bq2477x,
	unsigned int reg, unsigned int *val)
{
		//printk(KERN_INFO "bq2477x: bq2477x_read() enter");
	return regmap_read(bq2477x->regmap, reg, val);
}

static int bq2477x_read_word(struct bq2477x_chip *bq2477x,
	unsigned int reg, unsigned int *val)
{
			//printk(KERN_INFO "bq2477x: bq2477x_read_word() enter");
	return regmap_read(bq2477x->regmap_word, reg, val);
}

static int bq2477x_write(struct bq2477x_chip *bq2477x,
	unsigned int reg, unsigned int val)
{
	//printk(KERN_INFO "bq2477x: bq2477x_write() enter");
	return regmap_write(bq2477x->regmap, reg, val);
}

static int bq2477x_write_word(struct bq2477x_chip *bq2477x,
	unsigned int reg, unsigned int val)
{
	//printk(KERN_INFO "bq2477x: bq2477x_write_word() enter");
	return regmap_write(bq2477x->regmap_word, reg, val);
}

static int bq2477x_update_bits(struct bq2477x_chip *bq2477x,
	unsigned int reg, unsigned int mask, unsigned int val)
{
	//printk(KERN_INFO "bq2477x: bq2477x_update_bits() enter");
	return regmap_update_bits(bq2477x->regmap, reg, mask, val);
}

static enum power_supply_property bq2477x_psy_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int bq2477x_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct bq2477x_chip *bq2477x;
	bq2477x = container_of(psy, struct bq2477x_chip, ac);
	if (psp == POWER_SUPPLY_PROP_ONLINE){
	  val->intval = gpio_get_value(46);
	}else{
		//printk(KERN_INFO "bq2477x: bq2477x_ac_get_property() return -EINVAL");
		return -EINVAL;
	}
	//printk(KERN_INFO "bq2477x: bq2477x_ac_get_property() return 0");

	return 0;
}

static int bq2477x_show_chip_version(struct bq2477x_chip *bq2477x)
{
	int ret;
	unsigned int val;


	ret = bq2477x_read(bq2477x, BQ2477X_DEVICE_ID_REG, &val);
	if (ret < 0) {
		dev_err(bq2477x->dev, "DEVICE_ID_REG read failed: %d\n", ret);
		return ret;
	}

	if (val == BQ24770_DEVICE_ID)
		dev_info(bq2477x->dev, "chip type BQ24770 detected\n");
	else if (val == BQ24773_DEVICE_ID)
		dev_info(bq2477x->dev, "chip type BQ24773 detected\n");
	else {
		dev_info(bq2477x->dev, "unrecognized chip type: 0x%4x\n", val);
		return -EINVAL;
	}

	return 0;
}


static int bq2477x_write_i2c(struct i2c_client *client, u8 reg, u8 data)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;
	
	buf[0] = reg;
	buf[1] = data;
	msg.flags = 0;
	msg.addr = client->addr;
	msg.len = 2;
	msg.buf = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret;
}


static int bq2477x_hw_init(struct bq2477x_chip *bq2477x,struct i2c_client *client)
{
	int ret = 0;
	unsigned int val;

	ret = bq2477x_write_i2c(client,BQ2477X_CHARGE_OPTION_0_MSB,BQ2477X_CHARGE_OPTION_POR_MSB);

	if (ret < 0) {
		printk("BQ2477X_CHARGE_OPTION_0_MSB write failed %d\n", ret);
		return ret;
	}
	ret = bq2477x_write_i2c(client,BQ2477X_CHARGE_OPTION_0_LSB,BQ2477X_CHARGE_OPTION_POR_LSB);
	if (ret < 0) {
		printk("BQ2477X_CHARGE_OPTION_0_LSB write failed %d\n", ret);
		return ret;
	}
	ret = bq2477x_write_i2c(client,0x0c, 0x00);
	if (ret < 0) {
		printk("BQ2477X_CHARGE_VOLTAGE_LSB write failed %d\n", ret);
		return ret;
	}
	ret = bq2477x_write_i2c(client,0x0d,0x11);
	if (ret < 0) {
		printk("BQ2477X_CHARGE_CURRENT_MSB write failed %d\n", ret);
		return ret;
	}
	return ret;
}


void bq2477x_update_current(int curr)
{
	int ret;
	u8 val_lsb;
	u8 val_msb;

	if(client_g!=NULL){
       val_lsb = curr & 0xFF;
	bq2477x_write_i2c(client_g,BQ2477X_CHARGE_CURRENT_LSB, val_lsb);

       val_msb = curr >> 8;
	bq2477x_write_i2c(client_g,BQ2477X_CHARGE_CURRENT_MSB,val_msb);
	}else{
	printk(KERN_INFO "bq2477x: client_g = NULL");
	}

}
EXPORT_SYMBOL(bq2477x_update_current);

void bq2477x_update_voltage(int volt)
{
	int ret;
	u8 val_lsb;
	u8 val_msb;

	if(client_g!=NULL){
       val_lsb = volt & 0xFF;

	bq2477x_write_i2c(client_g,0x0c, val_lsb);

       val_msb = volt >> 8;
	bq2477x_write_i2c(client_g,0x0d,val_msb);
	}else{
	printk(KERN_INFO "bq2477x: client_g = NULL");
	}
}
EXPORT_SYMBOL(bq2477x_update_voltage);

irqreturn_t bq2477x_isr(int irq, void *dev_instance) {
struct bq2477x_chip *bq2477x_chip = dev_instance;

        pm_stay_awake(bq2477x_input->dev.parent);

		mod_timer(&bq2477x_chip->timer,
			jiffies + msecs_to_jiffies(5));

return IRQ_NONE;
}

static int bq2477x_open(struct input_dev *input)
{
	return 0;
}

static void bq2477x_close(struct input_dev *input)
{
	return;
}


static void bq2477x_gpio_timer(unsigned long _data)
{
	int charging_insertion;
	struct bq2477x_chip *bq2477x_chip = (struct bq2477x_chip *)_data;

	charging_insertion= gpio_get_value(46);
	if(charging_insertion==1){
	  input_event(bq2477x_input, EV_KEY, KEY_NUA_CHARGER_INSERTION, 1);
	  input_sync(bq2477x_input);
	  if (!wake_lock_active(&bq2477x_chip->bq_wakelock)) {
	    wake_lock(&bq2477x_chip->bq_wakelock);
	  }
	}
	else if(charging_insertion==0)
	{	
	  input_event(bq2477x_input, EV_KEY, KEY_NUA_CHARGER_INSERTION, 0);
	  input_sync(bq2477x_input);
	  if (wake_lock_active(&bq2477x_chip->bq_wakelock)) {
	    wake_unlock(&bq2477x_chip->bq_wakelock);
	  }
	}

	pm_relax(bq2477x_input->dev.parent);
}

static int bq2477x_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	printk(KERN_INFO "bq2477x: enter probe()");
	struct bq2477x_chip *bq2477x = NULL;
	struct bq2477x_platform_data *pdata;
	int ret = 0;
	int bq2477x_irq;
	pdata = client->dev.platform_data;
	int error =0;
	int charging_insertion = 0;
//------
	bq2477x_input = input_allocate_device();
	
	if (!bq2477x_input) {
		printk("input_allocate_device error\n");
	}

	bq2477x_input->name = "bq2477x_input";

	bq2477x_input->open = bq2477x_open;
	bq2477x_input->close = bq2477x_close;
	bq2477x_input->id.bustype = BUS_HOST;
	bq2477x_input->dev.parent = &client->dev;

	__clear_bit(EV_REP, bq2477x_input->evbit);
	
	input_set_capability(bq2477x_input,EV_KEY, KEY_NUA_CHARGER_INSERTION);

	error = input_register_device(bq2477x_input);
	if (error) {
		printk("bq2477x input_allocate_device error\n");
	}
	
//------
	if (!pdata) {
		dev_err(&client->dev, "No Platform data");
		return -EINVAL;
	}

	bq2477x = devm_kzalloc(&client->dev, sizeof(*bq2477x), GFP_KERNEL);
	if (!bq2477x) {
		dev_err(&client->dev, "Memory allocation failed\n");
		ret = -ENOMEM;
		goto gpio_err;
	}
	bq2477x->dev = &client->dev;

	bq2477x->dac_v = 4352;
	bq2477x->dac_minsv = 3584;
	bq2477x->dac_iin = 2994;
	bq2477x->wdt_refresh_timeout = 60;

	i2c_set_clientdata(client, bq2477x);
	bq2477x->irq = client->irq;
	mutex_init(&bq2477x->mutex);

	bq2477x->ac_online = 0;

	bq2477x->regmap = devm_regmap_init_i2c(client, &bq2477x_regmap_config);
	if (IS_ERR(bq2477x->regmap)) {
		ret = PTR_ERR(bq2477x->regmap);
		dev_err(&client->dev, "regmap init failed with err %d\n", ret);
		goto gpio_err;
	}

	bq2477x->regmap_word = devm_regmap_init_i2c(client,
					&bq2477x_regmap_word_config);
	if (IS_ERR(bq2477x->regmap_word)) {
		ret = PTR_ERR(bq2477x->regmap_word);
		dev_err(&client->dev,
			"regmap_word init failed with err %d\n", ret);
		goto gpio_err;
	}

/*
	ret = bq2477x_show_chip_version(bq2477x);
	if (ret < 0) {
		dev_err(bq2477x->dev, "version read failed %d\n", ret);
		goto gpio_err;
	}
*/

        ret = gpio_request(46, "bq2477x_irq");
        if (ret){
                pr_err("%s :bq2477x not able to get GPIO irq_gpio\n", __func__);
                return  -ENODEV;
        	}

	  ret = gpio_direction_input(46);
        if (ret < 0) {
                pr_err("%s :bq2477x not able to set irq_gpio as input\n", __func__);
                return -ENODEV;
        }

	wake_lock_init(&bq2477x->bq_wakelock, WAKE_LOCK_SUSPEND,
			"bq_lock");

	charging_insertion= gpio_get_value(46);

	if(charging_insertion==1){
	  wake_lock(&bq2477x->bq_wakelock);
	  input_event(bq2477x_input, EV_KEY, KEY_NUA_CHARGER_INSERTION, 1);
	  input_sync(bq2477x_input);
	}

	setup_timer(&bq2477x->timer,bq2477x_gpio_timer, (unsigned long)bq2477x);
			
	bq2477x_irq=gpio_to_irq(46);
	dev_info(bq2477x->dev, "bq2477x_irq number=%d\n",bq2477x_irq);
	ret = request_threaded_irq(bq2477x_irq, bq2477x_isr, NULL, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "BQ2477x", bq2477x);
	  if (ret) {
    pr_err("%s: Failed to enable bq2477x interrupt\n",__func__);
  }

	
	bq2477x->ac.name	= "bq2477x-ac";
	bq2477x->ac.type	= POWER_SUPPLY_TYPE_MAINS;
	bq2477x->ac.get_property	= bq2477x_ac_get_property;
	bq2477x->ac.properties		= bq2477x_psy_props;
	bq2477x->ac.num_properties	= ARRAY_SIZE(bq2477x_psy_props);

	ret = power_supply_register(bq2477x->dev, &bq2477x->ac);
	if (ret < 0) {
		dev_err(bq2477x->dev,
			"AC power supply register failed %d\n", ret);
		goto gpio_err;
	}


client_g = client;

	ret = bq2477x_hw_init(bq2477x,client);
	if (ret < 0) {
		dev_err(bq2477x->dev, "Hardware init failed %d\n", ret);
		client_g = NULL;
		goto psy_err;
	}

	dev_info(bq2477x->dev, "bq2477x charger registerd\n");

	return 0;

psy_err:
	power_supply_unregister(&bq2477x->ac);
gpio_err:
	//gpio_free(bq2477x->gpio);
	return ret;
}


/*
static void bq2477x_exit(void) {
printk(KERN_ALERT "sample driver removed\n");
free_irq(bq2477x_irq, irq_dev_id);
}*/
static int bq2477x_remove(struct i2c_client *client)
{
	struct bq2477x_chip *bq2477x = i2c_get_clientdata(client);
	flush_kthread_worker(&bq2477x->bq_kworker);
	kthread_stop(bq2477x->bq_kworker_task);
	power_supply_unregister(&bq2477x->ac);
	gpio_free(bq2477x->gpio);
	return 0;
}

static const struct i2c_device_id bq2477x_id[] = {
	{.name = "bq2477x",},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2477x_id);

static struct i2c_driver bq2477x_i2c_driver = {
	.driver = {
		.name   = "bq2477x",
		.owner  = THIS_MODULE,
	},
	.probe		= bq2477x_probe,
	.remove		= bq2477x_remove,
	.id_table	= bq2477x_id,
};

static struct bq2477x_platform_data bq2477x_pdata = {
};

static struct i2c_board_info bq2477x_board_info=   {
                I2C_BOARD_INFO("bq2477x", 0x6A),
                .platform_data = &bq2477x_pdata,
};

static int __init bq2477x_module_init(void)
{
	i2c_new_device(i2c_get_adapter(5), &bq2477x_board_info);//jesse
	return i2c_add_driver(&bq2477x_i2c_driver);
}
subsys_initcall(bq2477x_module_init);

static void __exit bq2477x_cleanup(void)
{
	i2c_del_driver(&bq2477x_i2c_driver);
}
module_exit(bq2477x_cleanup);

MODULE_DESCRIPTION("BQ24775/BQ24777 battery charger driver");
MODULE_AUTHOR("Andy Park <andyp@nvidia.com>");
MODULE_AUTHOR("Syed Rafiuddin <srafiuddin@nvidia.com");
MODULE_LICENSE("GPL v2");

