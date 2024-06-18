#include <linux/module.h>
#include <linux/printk.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/types.h>
#include <asm/io.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <uapi/asm-generic/errno-base.h>
#include <linux/timer.h>
#include <linux/spi/spi.h>
#include <linux/miscdevice.h>
#include "icm20608.h"


#define CHRDEV_NAME							"icm20608"
#define CHRDEV_MINOR						20
#define DTSCOMPATIBLE						"yuzang,icm20608"

#define SPI_SET_REG(dev, value)				spi_write_reg(dev, value)

struct chrdev_t{
	struct miscdevice misc;
	struct spi_device *spi;
}

static struct chrdev_t acm20608_dev;

/**
 * spi_read_regs-SPI read registers.
 * spi_dev: device with which data will be reaad.
 * addr: address of register.
 * buff: buffer into which data will be read.
 * size: length of read. in bytes
*/
int spi_read_regs(struct spi_device *spi_dev, uint8_t addr, 
				uint8_t *buff, size_t size){

	struct spi_transfer txfer[2];
	struct spi_message msg = {};
	uint8_t txdata;
	uint8_t *rxdata;
	int ret = -EINTR;

	rxdata = kzalloc(sizeof(uint8_t) * size + 1, GFP_KERNEL);
	if(rxdata == NULL){
		printk("fails to allocate memery.\r\n");
		goto out1;
	}

	spi_message_init(&msg);
	txdata = addr | 0x80;
	txfer[0].tx_buf = &txdata;
	txfer[0].len = 1;
	spi_message_add_tail(&txfer[0], &msg);
	txfer[1].rx_buf = rxdata;
	txfer[1].len = size;
	spi_message_add_tail(&txfer[1], &msg);
	ret = spi_sync(spi_dev, &msg);

	mdelay(6);
	memcpy(buff, rxdata, size);
out1:
	return ret;
}

/**
 * spi_write_regs-SPI write register.
 * addr: adress of register.
 * buff: data to be writen.
 * size: size of buff, in bytes.
*/
int spi_write_regs(struct spi_device *spi_dev, uint8_t addr, 
					const uint8_t *buff, size_t size){
	struct spi_transfer txfer = {};
	struct spi_message msg = {};
	uint8_t *data;
	int ret = 0;
	data = kzalloc(sizeof(uint8_t)*size+1, GFP_KERNEL);
	if(data == NULL){
		printk("fails to allocate memory!\r\n");
		return -EINTR;
	}
	data[0] = addr & ~(0x80);
	memcpy(data+1, buff, size);
	txfer.tx_buf = data;
	txfer.len = size + 1;
	spi_message_init(&msg);
	spi_message_add_tail(&txfer, &msg);
	ret = spi_sync(spi_dev, &msg);
	mdelay(6);
	return ret;
}

/***
 * spi_read_reg-SPI read one register
*/
int spi_read_reg(struct spi_device *spi_dev, uint8_t reg, uint8_t *buff){
	return spi_read_regs(spi_dev, reg, buff, 1);
}

/**
 * spi_write_reg-SPI write one register;
*/
int spi_write_reg(struct spi_device *spi_dev, uint8_t reg, const uint8_t data){
	uint8_t buf = data;
	return spi_write_regs(spi_dev, reg, &buf, 1);
}


#define acm20608_read(name) 								\
uint16_t acm20608_read_##name(struct spi_device *spi_dev){	\
	uint8_t data[2];										\
	uint16_t result;										\
	if(spi_read_reg(spi_dev, ACM20_##name##_H, &data[0])||	\
	spi_read_reg(spi_dev, ACM20_##name##_H, &data[1])){		\
		return -1;											\
	}														\
	result = ((uint16_t)data[0]<<8) | (data[1]);			\
	return result;											\
}

acm20608_read(ACCEL_X);
acm20608_read(ACCEL_Y);
acm20608_read(ACCEL_Z);
acm20608_read(TEMP);
acm20608_read(GYRO_X);
acm20608_read(GYRO_Y);
acm20608_read(GYRO_Z);

static int acm20608_open(struct inode *dev_inode, struct file *dev_file){
	dev_file->private_data = (void *)&acm20608_dev;
	return 0;
}

static int acm20608_release(struct inode *dev_inode, struct file *dev_file){

	return 0;
}

static long acm20608_ioctl(struct file *file, unsigned int cmd,
						unsigned long arg){
	struct chrdev_t *acm20608 = (struct chrdev_t*)file->private_data;
	uint16_t data;
	switch(cmd){
		case ACM20608_READ_ACCEL_X: 
			data = acm20608_read_ACCEL_X(acm20608->spi);
			break;
		case ACM20608_READ_ACCEL_Y:
			data = acm20608_read_ACCEL_Y(acm20608->spi);
			break;
		case ACM20608_READ_ACCEL_Z:
			data = acm20608_read_ACCEL_Z(acm20608->spi);
			break;
		case ACM20608_READ_TEMP:
			data = acm20608_read_TEMP(acm20608->spi);
			break;
		case ACM20608_READ_GYRO_X:
			data = acm20608_read_GYRO_Y(acm20608->spi);
			break;
		case ACM20608_READ_GYRO_Y:
			data = acm20608_read_GYRO_Z(acm20608->spi);
			break;
		case ACM20608_READ_GYRO_Z:
			data = acm20608_read_GYRO_Z(acm20608->spi);
			break;
		default:
			data = -1;
			break;
	}
	return copy_to_user((void*)arg, (void*)&data, sizeof(data));
}


#define show(name) 													\
ssize_t show_##name(struct device *dev, 							\
					struct device_attribute *attr,					\
					char *buf){										\
	struct chrdev_t *acm20608 = dev_get_drvdata(dev);				\
	struct spi_device *spi = acm20608->spi;							\
	uint16_t result = acm20608_read_##name(spi);					\
	memcpy((void*)buf, (void*)&result, sizeof(result));				\
	return sizeof(result);											\
}

show(ACCEL_X);
show(ACCEL_Y);
show(ACCEL_Z);
show(TEMP);
show(GYRO_X);
show(GYRO_Y);
show(GYRO_Z);

DEVICE_ATTR(accel_x, S_IRUSR, show_ACCEL_X, NULL);
DEVICE_ATTR(accel_y, S_IRUSR, show_ACCEL_Y, NULL);
DEVICE_ATTR(accel_z, S_IRUSR, show_ACCEL_Z, NULL);
DEVICE_ATTR(temper, S_IRUSR, show_TEMP, NULL);
DEVICE_ATTR(gyro_x, S_IRUSR, show_GYRO_X, NULL);
DEVICE_ATTR(gyro_y, S_IRUSR, show_GYRO_Y, NULL);
DEVICE_ATTR(gyro_z, S_IRUSR, show_GYRO_Z, NULL);

static struct attribute *acm20608_attrs[] = {
	&dev_attr_accel_x.attr,
	&dev_attr_accel_y.attr,
	&dev_attr_accel_z.attr,
	&dev_attr_temper.attr,
	&dev_attr_gyro_x.attr,
	&dev_attr_gyro_y.attr,
	&dev_attr_gyro_z.attr,
	NULL
};

static const struct attribute_group acm20608_attr_group = {
	.attrs = acm20608_attrs
};

static const struct attribute_group *acm20608_attr_groups[] = {
	&acm20608_attr_group,
	NULL
};

static struct file_operations ops = {
	.owner = THIS_MODULE,
	.open = acm20608_open,
	.release = acm20608_release,
	.unlocked_ioctl = acm20608_ioctl,
};

static int acm20608_init(struct spi_device *spi_dev_ptr){
	uint8_t temp;
	int ret = 0;
	/* 软复位 */
	ret = SPI_SET_REG(spi_dev_ptr, ACM20_PWR_MGMT_1_RST);
	mdelay(50);
	ret = SPI_SET_REG(spi_dev_ptr, ACM20_PWR_MGMT_1_CLK);
	mdelay(50);

	/* who and i*/
	ret = spi_read_reg(spi_dev_ptr, ACM20_WHO_AM_I, &temp);
	printk("who am i: %#x\r\n", temp);

	/* 配置acm20608 */
	ret = SPI_SET_REG(spi_dev_ptr, ACM20_SAMPLE_DEV);
	ret = SPI_SET_REG(spi_dev_ptr, ACM20_CONFIG);
	ret = SPI_SET_REG(spi_dev_ptr, ACM20_GYRO_CONFIG);
	ret = SPI_SET_REG(spi_dev_ptr, ACM20_ACCEL_CONFIG);
	ret = SPI_SET_REG(spi_dev_ptr, ACM20_ACCEL_CONFIG2);
	ret = SPI_SET_REG(spi_dev_ptr, ACM20_LP_MOD_CFG);
	ret = SPI_SET_REG(spi_dev_ptr, ACM20_FIFO_EN);
	ret = SPI_SET_REG(spi_dev_ptr, ACM20_PWR_MGMT_2);
	return 0;

}

static int acm20608_probe(struct spi_device *spi_dev){
	int ret = 0;
	struct miscdevice *misc_dev = &acm20608_dev.misc;
	/* 初始化spi_device */
	spi_dev->mode = SPI_MODE_0;
	spi_setup(spi_dev);
	acm20608_dev.spi = spi_dev;
	ret = acm20608_init(spi_dev);
	if(ret){
		dev_err(&spi_dev->dev, "fails to initialize acm20608.\r\n");
		return ret;
	}

	misc_dev->name = CHRDEV_NAME;
	misc_dev->minor = CHRDEV_MINOR;
	misc_dev->fops = &ops;
	misc_dev->groups = acm20608_attr_groups;
	ret = misc_register(&acm20608_dev.misc);
	if(ret){
		dev_err(&spi_dev->dev, "fails to register acm20608 miscdevice.");
		goto fail;
	}

	dev_set_drvdata(misc_dev->this_device, (void*)&acm20608_dev);
	return 0;
fail:
	return ret;
}

static int acm20608_remove(struct spi_device *spi_dev){
	misc_deregister(&acm20608_dev.misc);
	return 0;
}

static struct of_device_id match_table[] = {
	{.compatible = DTSCOMPATIBLE},
	{}
};

static struct spi_device_id spi_match_table[] = {
	{.name = DTSCOMPATIBLE, .driver_data = 0},
	{}
};

static struct spi_driver acm20608_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = CHRDEV_NAME,
		.of_match_table = match_table
	},
	.id_table = spi_match_table,
	.probe = acm20608_probe,
	.remove = acm20608_remove,
};

module_spi_driver(acm20608_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("yvzangjiang");