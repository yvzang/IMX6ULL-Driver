#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <uapi/asm-generic/errno-base.h>
#include <asm-generic/bitops/non-atomic.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/miscdevice.h>
#include <asm-generic/uaccess.h>


#define OF_COMPATIBLE					"fsl,ap3216c"
#define DEVICENAME						"ap3216c"
#define DEVICEMINOR						114

#define APCMD_READ_IR					0x00
#define APCMD_READ_ALS					0X01
#define APCMD_READ_PS					0X02

#define AP3216C_CFGREG					0x00
#define AP3216C_IR_L					0X0A
#define AP3216C_IR_H					0X0B
#define AP3216C_ALS_L					0X0C
#define AP3216C_ALS_H					0X0D
#define AP3216C_PS_L					0X0E
#define AP3216C_PS_H					0X0F


struct ap3216c_data{
	struct i2c_client *client;
	struct miscdevice misc;
	struct device dev;
	struct input_dev *input;
	unsigned int input_type;
	unsigned int input_code;
	int enable;
};

static int ap3216c_open(struct inode *inode, struct file *file);
static int ap3216c_release(struct inode *inode, struct file *file);
static long ap3216c_ioctl(struct file *file, unsigned int cmd, unsigned long args);


static int i2c_read_register(struct i2c_client *client, uint8_t reg, uint8_t *data){
	struct i2c_msg msgs[2] = {
		[0] = {
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = (__u8*)&reg
		},
		[1] = {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = (__u8*)data
		}
	};
	if(2 != i2c_transfer(client->adapter, msgs, 2))
		return -1;
	return 0;
}

static int i2c_write_register(struct i2c_client *client, uint8_t reg, uint8_t data){
	uint8_t buff[2] = {reg, data};
	struct i2c_msg msgs = {
		.addr = client->addr,
		.flags = 0,
		.len = 2,
		.buf = (__u8*)buff
	};
	if(1 != i2c_transfer(client->adapter, &msgs, 1))
		return -1;
	return 0;
}

static int ap3216c_read_ir(struct i2c_client *client, uint16_t *ir){
	uint8_t data_l=0, data_h=0;
	if(i2c_read_register(client, AP3216C_IR_L, &data_l) ||
		i2c_read_register(client, AP3216C_IR_H, &data_h))
		return -1;
	if(data_l & 0x80)
		return -1;
	*ir = ((uint32_t)data_h << 2) | ((uint32_t)data_l & 0x03);
	return 0;
}

static int ap3216c_read_als(struct i2c_client *client, uint16_t *als){
	uint8_t data_l, data_h;
	if(i2c_read_register(client, AP3216C_ALS_L, &data_l)||
		i2c_read_register(client, AP3216C_ALS_H, &data_h))
		return -1;
	*als = ((uint32_t)data_h << 8) | (data_l);
	return 0;
}

static int ap3216c_read_ps(struct i2c_client *client, uint16_t *ps){
	uint8_t data_h, data_l;
	if(i2c_read_register(client, AP3216C_PS_L, &data_l) ||
		i2c_read_register(client, AP3216C_PS_H, &data_h))
		return -1;
	if(data_l & 0x40)
		return -1;
	*ps = ((uint32_t)data_h << 8) | ((uint32_t)data_l & 0x0f);
	return 0;
}


static struct file_operations ops = {
	.owner = THIS_MODULE,
	.open = ap3216c_open,
	.release = ap3216c_release,
	.unlocked_ioctl = ap3216c_ioctl
};

static ssize_t ap3216c_show_helper(struct i2c_client *client, 
									int (*read_fn)(struct i2c_client *, uint16_t *), 
									char *buf){
	int ret = 0;
	uint16_t data;
	ret = read_fn(client, &data);
	if(ret){
		return sprintf(buf, "-1\r\n");
	}
	return sprintf(buf, "%d\n", data);
}

#define AP3216C_SHOW_FN(name) 										\
ssize_t ap3216c_show_##name(struct device *dev,						\
									struct device_attribute *attr,	\
									char *buff){					\
	struct ap3216c_data *ap3216c = dev_get_drvdata(dev);			\
	return ap3216c_show_helper(ap3216c->client,ap3216c_read_##name, buff);			\
}

static AP3216C_SHOW_FN(ir);
static AP3216C_SHOW_FN(als);
static AP3216C_SHOW_FN(ps);

static DEVICE_ATTR(ir, S_IRUGO, ap3216c_show_ir, NULL);
static DEVICE_ATTR(als, S_IRUGO, ap3216c_show_als, NULL);
static DEVICE_ATTR(ps, S_IRUGO, ap3216c_show_ps, NULL);

static struct attribute *ap3216c_attrs[] = {
	&dev_attr_ir.attr,
	&dev_attr_als.attr,
	&dev_attr_ps.attr,
	NULL
};

static struct attribute_group ap3216c_attr_group = {
	.attrs = ap3216c_attrs
};

static struct attribute_group *ap3216c_attr_groups[] = {
	&ap3216c_attr_group,
	NULL
};

static struct ap3216c_data ap3216c = {
	.misc = {
		.minor = DEVICEMINOR,
		.name = DEVICENAME,
		.groups = (const struct attribute_group**)ap3216c_attr_groups,
		.fops = &ops,
	},
	.client = NULL
};

static int ap3216c_open(struct inode *inode, struct file *file){
	file->private_data = (void*)&ap3216c;
	return 0;
}

static int ap3216c_release(struct inode *inode, struct file *file){
	file->private_data = NULL;
	return 0;
}

static long ap3216c_ioctl(struct file *file, unsigned int cmd, unsigned long args){
	struct ap3216c_data *ap = (struct ap3216c_data*)file->private_data;
	uint16_t data=-1;
	int ret = 0;
	switch (cmd)
	{
	case APCMD_READ_IR:
		ret = ap3216c_read_ir(ap->client, &data);
		break;
	
	case APCMD_READ_ALS:
		ret = ap3216c_read_als(ap->client, &data);
		break;

	case APCMD_READ_PS:
		ret = ap3216c_read_ps(ap->client, &data);
		break;
	default:
		break;
	}
	if(copy_to_user((void*)args, &data, sizeof(data)))
		return -1;
	return ret;
}

static int ap3216c_report_data(struct ap3216c_data *ap3216c, 
								struct input_dev *input){
	uint16_t ir;
	int ret = 0;
	ret = ap3216c_read_ir(ap3216c->client, &ir);
	if(ret){
		dev_err(&ap3216c->client->dev, "fails to read ir data.");
		return -EINTR;
	}
	input_event(input, ap3216c->input_type, ap3216c->input_code, ir);
	input_sync(input);
	return 0;
}

static int ap3216c_input_open(struct input_dev *dev){
	struct ap3216c_data *ap3216c = (struct ap3216c_data*)input_get_drvdata(dev);
	if(!ap3216c->enable)
		return -EBUSY;
	return ap3216c_report_data(ap3216c, dev);
}

static void ap3216c_input_close(struct input_dev *dev){

}

static int ap3216c_reset(struct i2c_client *client){
	int ret = 0;
	uint8_t data;
	/* software reset */
	ret = i2c_write_register(client, AP3216C_CFGREG, 0x04);
	if(ret < 0){
		goto fail;
	}
	mdelay(50);
	ret = i2c_write_register(client, AP3216C_CFGREG, 0x03);
	if(ret < 0){
		goto fail;
	}
	mdelay(50);
	ret = i2c_read_register(client, AP3216C_CFGREG, &data);
	if(data != 0x03){
		goto fail;
	}
	printk("success reset ap3216c!\r\n");
	return 0;

fail:
	dev_err(&client->dev, "fails to software reset ap3216c.");
	return -1;
}


static int ap3216c_probe(struct i2c_client *client, const struct i2c_device_id *ids){
	int ret = 0;
	struct input_dev *input;
	ap3216c.client = client;
	/* software reset */
	ret = ap3216c_reset(client);
	if(ret < 0)
		return ret;

	ret = misc_register(&ap3216c.misc);
	if(ret)
		return ret;
	
	input = devm_input_allocate_device(&client->dev);
	if(!input){
		dev_err(&client->dev, "failst to allocate input device!\r\n");
		return -ENOMEM;
	}
	dev_set_drvdata(ap3216c.misc.this_device, (void*)&ap3216c);
	input_set_drvdata(input, (void*)&ap3216c);

	ap3216c.input = input;
	ap3216c.input_type = EV_MSC;
	ap3216c.input_code = 0x01;

	input->name = DEVICENAME;
	input->phys = "ap3216c/input0";
	input->dev.parent = &client->dev;
	input->open = ap3216c_input_open;
	input->close = ap3216c_input_close;

	input->id.bustype = BUS_HOST;
	__set_bit(EV_REP, input->evbit);
	__set_bit(ap3216c.input_type, input->evbit);
	input_set_capability(input, ap3216c.input_type, ap3216c.input_code);

	ret = input_register_device(input);
	if(ret){
		dev_err(&client->dev, "Unable to register input device.");

	}
	ap3216c.enable = 1;
	return 0;
}

static int ap3216c_remove(struct i2c_client *client){
	input_unregister_device(ap3216c.input);
	misc_deregister(&ap3216c.misc);
	return 0;
}

static struct i2c_device_id device_id_tables[] = {
	{.name = DEVICENAME, .driver_data = 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, device_id_tables);


static struct of_device_id of_match[] = {
	{.compatible = OF_COMPATIBLE},
	{}
};
MODULE_DEVICE_TABLE(of, of_match);


static struct i2c_driver ap3216c_driver = {
	.driver = {
		.of_match_table = of_match,
		.owner = THIS_MODULE,
		.name = DEVICENAME
	},
	.id_table = device_id_tables,
	.probe = ap3216c_probe,
	.remove = ap3216c_remove
};


module_i2c_driver(ap3216c_driver);
MODULE_AUTHOR("YUZANG");
MODULE_LICENSE("GPL");