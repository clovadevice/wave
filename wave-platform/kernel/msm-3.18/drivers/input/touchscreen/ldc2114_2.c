#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#if defined(CONFIG_SECURE_TOUCH)
#include <linux/pm_runtime.h>
#endif
#include <linux/regulator/consumer.h>
#include <linux/input/ldc2114.h>

#define I2C_DRIVER_NAME "ldc2114_2_2"

static int irq_gpio5=0;
static int irq_gpio6=0;

#define LDC2114_LPWR2 13

struct input_dev *ldc2114_2_input;

static int ldc2114_2_read_i2c(struct i2c_client *client, u8 reg)
{
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	ret = data[0];

	return ret;
}

static int ldc2114_2_write_i2c(struct i2c_client *client, u8 reg, u8 data)
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

static int ldc2114_2_power_on_write_i2c(struct i2c_client *client)
{
	struct i2c_msg msg;
	u8 init_reg[33]={0x0C, 0x03, 0x01, 0x2E, 0x02, 0x2E, 0x00, 0x00, 0x05, 0x00, 0x07, 0x0F, 0x03, 0x0F, 0x00, 0xF0, 0x00, 0x0F, 0x00, 0xAA, 0x00, 0xB5, 0x00, 0xB5, 0x00, 0x00, 0x06, 0x00, 0x00, 0xF0, 0x00, 0x00 ,0x03};
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg.flags = 0;
	msg.addr = client->addr;
	msg.len = 33;
	msg.buf = init_reg;

	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret;
}

static irqreturn_t ldc2114_interrupt5(int irq, void *pdata)
{
    int gpio_val = 0;

    gpio_val = gpio_get_value(irq_gpio5);

 if (gpio_val==0 && down_key == 0)  
 {
      down_key = 5;
      input_event(ldc2114_2_input, EV_KEY, KEY_NUA_BUTTON5, 1);
      input_sync(ldc2114_2_input);

 }else if (gpio_val==1 && down_key == 5){

      down_key = 0;
      input_event(ldc2114_2_input, EV_KEY, KEY_NUA_BUTTON5, 0);
      input_sync(ldc2114_2_input);
 }
 
	return IRQ_HANDLED;
}

static irqreturn_t ldc2114_interrupt6(int irq, void *pdata)
{
    int gpio_val = 0;

    gpio_val = gpio_get_value(irq_gpio6);

 if (gpio_val==0 && down_key == 0)  
 {
      down_key = 6;
      input_event(ldc2114_2_input, EV_KEY, KEY_NUA_BUTTON6, 1);
      input_sync(ldc2114_2_input);

 }else if (gpio_val==1 && down_key == 6){

      down_key = 0;
      input_event(ldc2114_2_input, EV_KEY, KEY_NUA_BUTTON6, 0);
      input_sync(ldc2114_2_input);
 }
	return IRQ_HANDLED;
}


static int ldc2114_2_open(struct input_dev *input)
{
	return 0;
}

static void ldc2114_2_close(struct input_dev *input)
{
	return;
}

static ssize_t ldc2114_2_calibration_gain_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	unsigned char gain_addr[2] = {0x0E, 0x10};
	char *token[10];
	unsigned long val = 0;
	int i, touch_index;
	u8 gain_value;

	for (i = 0; i < 2; i++) {
		token[i] = strsep((char **)&buf, ",");

		kstrtoul(token[i], 0, &val);

		if (i==0) {
			if (val == 0) {
				touch_index = 0;
			} else if (val == 5) {
				touch_index = 1;
			} else {
				printk("ldc2114_2, error touch index\n");
				return -1;
			}
		} else {
			gain_value = (u8)val;
			if (gain_value > 63 || gain_value < 1) {
				printk("ldc2114_2, error calibration gain value");
				return -1;
			}
		}
	}

	ldc2114_2_write_i2c(client, 0x0a, 0x01);
	msleep(50);
	ldc2114_2_write_i2c(client, gain_addr[touch_index], gain_value);
	ldc2114_2_write_i2c(client, 0x0a, 0x00);

	return count;
}

static DEVICE_ATTR(calibration_gain,  S_IWUSR,
			NULL, ldc2114_2_calibration_gain_store);

static struct attribute *ldc2114_2_attributes[] = {
	&dev_attr_calibration_gain.attr,
	NULL,
};

static const struct attribute_group ldc2114_2_attribute_group = {
	.attrs = ldc2114_2_attributes
};

static int ldc2114_2_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int retval;
	int ret = 0;
	int irq5 =0;
	int irq6 =0;
	int error =0;
	enum of_gpio_flags gpio_flags5;
	enum of_gpio_flags gpio_flags6;
	struct device_node *np = client->dev.of_node;
	struct regulator *regulator_vdd;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data commands not supported by host\n",
				__func__);
		return -EIO;
	}



	ldc2114_2_input = input_allocate_device();

	if (!ldc2114_2_input) {
		printk("input_allocate_device error\n");
	}

	ldc2114_2_input->name = "ldc2114_2_input";

	ldc2114_2_input->open = ldc2114_2_open;
	ldc2114_2_input->close = ldc2114_2_close;
	ldc2114_2_input->id.bustype = BUS_HOST;
	ldc2114_2_input->dev.parent = &client->dev;

	__clear_bit(EV_REP, ldc2114_2_input->evbit);
	
	input_set_capability(ldc2114_2_input,EV_KEY, KEY_NUA_BUTTON5);
	input_set_capability(ldc2114_2_input,EV_KEY, KEY_NUA_BUTTON6);

	error = input_register_device(ldc2114_2_input);
	if (error) {
		printk("ldc2114_2 input_allocate_device error\n");

	}

	regulator_vdd = regulator_get(&client->dev,
			"vdd");
	if (IS_ERR(regulator_vdd)) {
		printk("%s: Failed to get regulator vdd\n",
				__func__);
	}else{

		retval = regulator_enable(regulator_vdd);
		if (retval) {
			printk("%s: Failed to enable regulator vdd\n",
				__func__);
		}
	}


	if (gpio_is_valid(LDC2114_LPWR2)) {
		error = gpio_request_one(LDC2114_LPWR2,
				GPIOF_DIR_OUT,
				"ldc2114_lpwr2");
		if (error) {
			dev_err(&client->dev, "unable to request gpio %d\n",
				LDC2114_LPWR2);
		}

              gpio_direction_output(LDC2114_LPWR2,1);
	} else {
		dev_err(&client->dev, "lpwr2 not provided\n");
	}

	irq_gpio5 = of_get_named_gpio_flags(np,
				"qcom,irq-gpio5", 0, &gpio_flags5);
	irq_gpio6 = of_get_named_gpio_flags(np,
				"qcom,irq-gpio6", 0, &gpio_flags6);

	if (gpio_is_valid(irq_gpio5)) {
		error = gpio_request_one(irq_gpio5,
				GPIOF_DIR_IN,
				"ldc2114_gpio5");
		if (error) {
			dev_err(&client->dev, "unable to request gpio %d\n",
				irq_gpio5);
		}

	       irq5 = gpio_to_irq(irq_gpio5);

	       error = request_threaded_irq(irq5, ldc2114_interrupt5, NULL,
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "ld2114_out5", NULL);
	} else {
		dev_err(&client->dev, "irq gpio5 not provided\n");
	}

	if (gpio_is_valid(irq_gpio6)) {
		error = gpio_request_one(irq_gpio6,
				GPIOF_DIR_IN,
				"ldc2114_gpio6");
		if (error) {
			dev_err(&client->dev, "unable to request gpio %d\n",
				irq_gpio6);
		}

	       irq6 = gpio_to_irq(irq_gpio6);

	       error = request_threaded_irq(irq6, ldc2114_interrupt6, NULL,
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "ld2114_out6", NULL);
	} else {
		dev_err(&client->dev, "irq gpio6 not provided\n");
	}

	msleep(100);
	ldc2114_2_write_i2c(client,0x0a,0x01);
	msleep(50);
	ldc2114_2_power_on_write_i2c(client);

	ldc2114_2_write_i2c(client,0x0a,0x00);

	ret = sysfs_create_group(&client->dev.kobj, &ldc2114_2_attribute_group);
	if (ret)
		return ret;

	return 0;
}

static int ldc2114_2_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ldc2114_2_id_table[] = {
	{I2C_DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ldc2114_2_id_table);

#ifdef CONFIG_OF
static struct of_device_id ldc_match_table[] = {
	{ .compatible = "qcom,ldc2114_2",},
	{ },
};
#else
#define dsx_match_table NULL
#endif

static const struct i2c_device_id ldc_id_table[] = {
	{I2C_DRIVER_NAME, 0},
	{},
};

static struct i2c_driver ldc2114_2_i2c_driver = {
	.driver = {
		.name = I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ldc_match_table,
	},
	.probe = ldc2114_2_i2c_probe,
	.remove = ldc2114_2_i2c_remove,
	.id_table = ldc_id_table,
};

int ldc2114_2_init(void)
{
	return i2c_add_driver(&ldc2114_2_i2c_driver);
}

void ldc2114_2_exit(void)
{
	i2c_del_driver(&ldc2114_2_i2c_driver);

	return;
}

module_init(ldc2114_2_init);
module_exit(ldc2114_2_exit);

MODULE_AUTHOR("QUANTA, Inc.");
MODULE_DESCRIPTION("TI ldc2114_2 I2C Bus Support Module");
MODULE_LICENSE("GPL v2");
