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
#define I2C_DRIVER_NAME "ldc2114"

static int irq_gpio1=0;
static int irq_gpio2=0;
static int irq_gpio3=0;
static int irq_gpio4=0;

#define LDC2114_LPWR1 12

#define GPIO_BOARD_ID0 80
#define GPIO_BOARD_ID1 81
#define GPIO_BOARD_ID2 82

int down_key = 0;

struct input_dev *ldc2114_input;

static int ldc2114_read_i2c(struct i2c_client *client, u8 reg)
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

static int ldc2114_power_on_write_i2c_EVT(struct i2c_client *client)
{
	struct i2c_msg msg;
	u8 init_reg[33]={0x0C,0x0F,0x01, 0x30, 0x02, 0x30, 0x00, 0x30, 0x05, 0x30, 0x03, 0x0F, 0x02, 0x0F, 0x00, 0xF0, 0x00, 0x0F, 0x00, 0xAA, 0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF, 0x00,0xBF, 0x00, 0xA0, 0x00, 0x00 ,0x02};
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

static int ldc2114_power_on_write_i2c_DVT(struct i2c_client *client)
{
	struct i2c_msg msg;
	u8 init_reg[33]={0x0C, 0x0F, 0x01, 0x2E, 0x02, 0x2E, 0x00, 0x2E, 0x05, 0x2E, 0x07, 0x0F, 0x02, 0x0F, 0x00, 0xF0, 0x00, 0x0F, 0x00, 0xAA, 0x00, 0xBF, 0x00, 0xBF, 0x00, 0xBF, 0x06, 0xBF, 0x00, 0xF0, 0x00, 0x00 ,0x03};
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

static int ldc2114_write_i2c(struct i2c_client *client, u8 reg, u8 data)
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

static irqreturn_t ldc2114_interrupt1(int irq, void *pdata)
{
    int gpio_val = 0;

    gpio_val = gpio_get_value(irq_gpio1);

 if (gpio_val==0 && down_key == 0)  
 {
      down_key = 1;
      input_event(ldc2114_input, EV_KEY, KEY_NUA_BUTTON1, 1);
      input_sync(ldc2114_input);

 }else if (gpio_val==1 && down_key == 1){

      down_key = 0;
      input_event(ldc2114_input, EV_KEY, KEY_NUA_BUTTON1, 0);
      input_sync(ldc2114_input);
 }

	return IRQ_HANDLED;
}

static irqreturn_t ldc2114_interrupt2(int irq, void *pdata)
{
    int gpio_val = 0;

    gpio_val = gpio_get_value(irq_gpio2);

 if (gpio_val==0 && down_key == 0)  
 {
      down_key = 2;
      input_event(ldc2114_input, EV_KEY, KEY_NUA_BUTTON2, 1);
      input_sync(ldc2114_input);

 }else if (gpio_val==1 && down_key == 2){

      down_key = 0;
      input_event(ldc2114_input, EV_KEY, KEY_NUA_BUTTON2, 0);
      input_sync(ldc2114_input);
 }
	return IRQ_HANDLED;
}

static irqreturn_t ldc2114_interrupt3(int irq, void *pdata)
{
    int gpio_val = 0;

    gpio_val = gpio_get_value(irq_gpio3);

 if (gpio_val==0 && down_key == 0)  
 {
      down_key = 3;
      input_event(ldc2114_input, EV_KEY, KEY_NUA_BUTTON3, 1);
      input_sync(ldc2114_input);

 }else if (gpio_val==1 && down_key == 3){

      down_key = 0;
      input_event(ldc2114_input, EV_KEY, KEY_NUA_BUTTON3, 0);
      input_sync(ldc2114_input);
 }

 
	return IRQ_HANDLED;
}

static irqreturn_t ldc2114_interrupt4(int irq, void *pdata)
{
    int gpio_val = 0;

    gpio_val = gpio_get_value(irq_gpio4);

 if (gpio_val==0 && down_key == 0)  
 {
      down_key = 4;
      input_event(ldc2114_input, EV_KEY, KEY_NUA_BUTTON4, 1);
      input_sync(ldc2114_input);

 }else if (gpio_val==1 && down_key == 4){

      down_key = 0;
      input_event(ldc2114_input, EV_KEY, KEY_NUA_BUTTON4, 0);
      input_sync(ldc2114_input);
 }
 
	return IRQ_HANDLED;
}

static int ldc2114_open(struct input_dev *input)
{
	return 0;
}

static void ldc2114_close(struct input_dev *input)
{
	return;
}

static ssize_t ldc2114_calibration_gain_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	unsigned char gain_addr[4] = {0x10, 0x0E, 0x12, 0x14};
	char *token[10];
	unsigned long val;
	int i, touch_index;
	u8 gain_value = 0;

	for (i = 0; i < 2; i++) {
		token[i] = strsep((char **)&buf, ",");

		kstrtoul(token[i], 0, &val);
		if (i==0) {
			if (val >0 && val < 5) {
				touch_index = val - 1;
			} else {
				printk("ldc2114, error touch index\n");
				return -1;
			}
		} else {
			gain_value = (u8)val;
			if (gain_value > 63 || gain_value < 0) {
				printk("ldc2114, error calibration gain value");
				return -1;
			}
		}
	}

	ldc2114_write_i2c(client, 0x0a, 0x01);
	msleep(50);
	ldc2114_write_i2c(client, gain_addr[touch_index], gain_value);
	ldc2114_write_i2c(client, 0x0a, 0x00);

	return count;
}

static DEVICE_ATTR(calibration_gain,  S_IWUSR,
			NULL, ldc2114_calibration_gain_store);

static struct attribute *ldc2114_attributes[] = {
	&dev_attr_calibration_gain.attr,
	NULL,
};

static const struct attribute_group ldc2114_attribute_group = {
	.attrs = ldc2114_attributes
};

static int ldc2114_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int retval;
	int ret = 0;
	int irq1 =0;
	int irq2 =0;
	int irq3 =0;
	int irq4 =0;
	int error =0;
       int board0_val = 0;
       int board1_val = 0;
       int board2_val = 0;
       int board_id = 0;
	enum of_gpio_flags gpio_flags1;
	enum of_gpio_flags gpio_flags2;
	enum of_gpio_flags gpio_flags3;
	enum of_gpio_flags gpio_flags4;
	struct device_node *np = client->dev.of_node;
	struct regulator *regulator_vdd;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data commands not supported by host\n",
				__func__);
		return -EIO;
	}


	ldc2114_input = input_allocate_device();

	if (!ldc2114_input) {
		printk("input_allocate_device error\n");
	}

	ldc2114_input->name = "ldc2114_input";

	ldc2114_input->open = ldc2114_open;
	ldc2114_input->close = ldc2114_close;
	ldc2114_input->id.bustype = BUS_HOST;
	ldc2114_input->dev.parent = &client->dev;

	__clear_bit(EV_REP, ldc2114_input->evbit);
	
	input_set_capability(ldc2114_input,EV_KEY, KEY_NUA_BUTTON1);
	input_set_capability(ldc2114_input,EV_KEY, KEY_NUA_BUTTON2);
	input_set_capability(ldc2114_input,EV_KEY, KEY_NUA_BUTTON3);
	input_set_capability(ldc2114_input,EV_KEY, KEY_NUA_BUTTON4);

	error = input_register_device(ldc2114_input);
	if (error) {
		printk("ldc2114 input_allocate_device error\n");
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


	if (gpio_is_valid(LDC2114_LPWR1)) {
		error = gpio_request_one(LDC2114_LPWR1,
				GPIOF_DIR_OUT,
				"ldc2114_lpwr1");
		if (error) {
			dev_err(&client->dev, "unable to request gpio %d\n",
				LDC2114_LPWR1);
		}

              gpio_direction_output(LDC2114_LPWR1,1);
	} else {
		dev_err(&client->dev, "lpwr1 not provided\n");
	}

	irq_gpio1 = of_get_named_gpio_flags(np,
				"qcom,irq-gpio1", 0, &gpio_flags1);
	irq_gpio2 = of_get_named_gpio_flags(np,
				"qcom,irq-gpio2", 0, &gpio_flags2);
	irq_gpio3 = of_get_named_gpio_flags(np,
				"qcom,irq-gpio3", 0, &gpio_flags3);
	irq_gpio4 = of_get_named_gpio_flags(np,
				"qcom,irq-gpio4", 0, &gpio_flags4);

	if (gpio_is_valid(irq_gpio1)) {
		error = gpio_request_one(irq_gpio1,
				GPIOF_DIR_IN,
				"ldc2114_gpio1");
		if (error) {
			dev_err(&client->dev, "unable to request gpio %d\n",
				irq_gpio1);
		}

	       irq1 = gpio_to_irq(irq_gpio1);

	       error = request_threaded_irq(irq1, ldc2114_interrupt1, NULL,
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "ld2114_out1", NULL);
	} else {
		dev_err(&client->dev, "irq gpio1 not provided\n");
	}

	if (gpio_is_valid(irq_gpio2)) {
		error = gpio_request_one(irq_gpio2,
				GPIOF_DIR_IN,
				"ldc2114_gpio2");
		if (error) {
			dev_err(&client->dev, "unable to request gpio %d\n",
				irq_gpio2);
		}

	       irq2 = gpio_to_irq(irq_gpio2);

	       error = request_threaded_irq(irq2, ldc2114_interrupt2, NULL,
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "ld2114_out2", NULL);
	} else {
		dev_err(&client->dev, "irq gpio2 not provided\n");
	}

	if (gpio_is_valid(irq_gpio3)) {
		error = gpio_request_one(irq_gpio3,
				GPIOF_DIR_IN,
				"ldc2114_gpio3");
		if (error) {
			dev_err(&client->dev, "unable to request gpio %d\n",
				irq_gpio3);
		}

	       irq3 = gpio_to_irq(irq_gpio3);

	       error = request_threaded_irq(irq3, ldc2114_interrupt3, NULL,
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "ld2114_out3", NULL);
	} else {
		dev_err(&client->dev, "irq gpio3 not provided\n");
	}

	if (gpio_is_valid(irq_gpio4)) {
		error = gpio_request_one(irq_gpio4,
				GPIOF_DIR_IN,
				"ldc2114_gpio4");
		if (error) {
			dev_err(&client->dev, "unable to request gpio %d\n",
				irq_gpio4);
		}

	       irq4 = gpio_to_irq(irq_gpio4);

	       error = request_threaded_irq(irq4, ldc2114_interrupt4, NULL,
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "ld2114_out4", NULL);
	} else {
		dev_err(&client->dev, "irq gpio4 not provided\n");
	}
	
	msleep(100);
	ldc2114_write_i2c(client,0x0a,0x01);
	msleep(50);

/*
	board0_val = gpio_get_value(GPIO_BOARD_ID0);
	board1_val = gpio_get_value(GPIO_BOARD_ID1);
	board2_val = gpio_get_value(GPIO_BOARD_ID2);

       board_id = (board0_val<<2)|(board1_val<<1)|(board2_val);

	if(board_id==0){
	  ldc2114_power_on_write_i2c_EVT(client);
	}else{
	  ldc2114_power_on_write_i2c_DVT(client);
	}
*/
	ldc2114_power_on_write_i2c_DVT(client);

	ldc2114_write_i2c(client,0x0a,0x00);

	ret = sysfs_create_group(&client->dev.kobj, &ldc2114_attribute_group);
	if (ret)
		return ret;

	return 0;
}

static int ldc2114_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ldc2114_id_table[] = {
	{I2C_DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ldc2114_id_table);

#ifdef CONFIG_OF
static struct of_device_id ldc_match_table[] = {
	{ .compatible = "qcom,ldc2114",},
	{ },
};
#else
#define dsx_match_table NULL
#endif

static const struct i2c_device_id ldc_id_table[] = {
	{I2C_DRIVER_NAME, 0},
	{},
};

static struct i2c_driver ldc2114_i2c_driver = {
	.driver = {
		.name = I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ldc_match_table,
	},
	.probe = ldc2114_i2c_probe,
	.remove = ldc2114_i2c_remove,
	.id_table = ldc_id_table,
};

int ldc2114_init(void)
{
	return i2c_add_driver(&ldc2114_i2c_driver);
}

void ldc2114_exit(void)
{
	i2c_del_driver(&ldc2114_i2c_driver);

	return;
}

module_init(ldc2114_init);
module_exit(ldc2114_exit);

MODULE_AUTHOR("QUANTA, Inc.");
MODULE_DESCRIPTION("TI LDC2114 I2C Bus Support Module");
MODULE_LICENSE("GPL v2");
