/**
 * GT9147 mult-touchscreen driver.
 * See Documentation/devicetree/bindings/input/touchscreen/goodix.txt
 * to set devicetree.
 * Date: 2024/05
 * Author: yuzangjiang
*/

#include <linux/module.h>
#include <linux/printk.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/types.h>
#include <asm/io.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <uapi/asm-generic/errno-base.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/blkdev.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <linux/byteorder/generic.h>
#include <linux/unaligned/le_byteshift.h>
#include <linux/timer.h>
#include "touchscreen.h"

#define GT_DEBUG_ON							1
#define GT_INFO_ON							1

#define GT_INFO(fmt,arg...)					{											\
												if(GT_INFO_ON)							\
												printk("<<-GT-INFO->>"fmt"\n",##arg);	\
											}
#define GT_DEBUG(fmt,arg...)				{											\
												if(GT_DEBUG_ON)							\
												printk("<<-GT-DEBUG->>"fmt"\n",##arg);	\
											}
#define GT_ERROR(fmt,arg...)				{											\
												printk("<<-GT-ERROR->>"fmt"\n",##arg);	\
											}


#define DTSCOMPATIBLE						"goodix,gt9147"
#define GT_NAME								"GT9147"
#define GT_INPUT_PHY						"input/gt"
#define REG_SIZE							4
#define GT_CONFIG_MAX_LENGTH				0x8100-GT_CFGS_REG+1
#define GT_DEFAULT_X_MAX					4096
#define GT_DEFAULT_Y_MAX					4096
#define GT_DEFAULT_TRIGGER					1
#define RESULTION_LOC						3
#define TRIGGER_LOC							8
#define GT_TIMER_EXPIRES					jiffies+msecs_to_jiffies(20)

#define IRQ_TABLE				{IRQ_TYPE_EDGE_RISING, 	\
								IRQ_TYPE_EDGE_FALLING,	\
								IRQ_TYPE_LEVEL_LOW, 	\
								IRQ_TYPE_LEVEL_HIGH}


struct gt9147_data{
	struct gt9147_pri_data *gt_data;
	struct i2c_client *client;
	struct input_dev *input;
	struct spinlock irq_lock;
	struct work_struct work;
	struct timer_list timer;
};

static uint8_t config[GT_CONFIG_MAX_LENGTH];

static struct gt9147_data gt9147_dev;


int gt_i2c_read(struct i2c_client *client, uint16_t reg, 
				uint8_t *buf, uint32_t size){
	int ret = 0;
	__be16 bebuf = cpu_to_be16(reg);
	struct i2c_msg msgs[2];

	msgs[0].flags = 0;
	msgs[0].addr = client->addr;
	msgs[0].len = 2;
	msgs[0].buf = (void*)&bebuf;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].len = size;
	msgs[1].buf = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	return ret < 0 ? ret : (ret == ARRAY_SIZE(msgs) ? 0 : -EIO);
}

int gt_i2c_write(struct i2c_client *client, uint16_t reg,
				uint8_t *buf, uint32_t size){
	int ret = 0;
	int retried = 0;
	struct i2c_msg msg;
	uint8_t *data = devm_kzalloc(&client->dev, size+GT9147_REG_WIDTH,
								GFP_KERNEL);
	__be16 bereg = cpu_to_be16(reg);
	data[0] = bereg & 0xff;
	data[1] = (bereg >> 8) & 0xff;
	memcpy(&data[GT9147_REG_WIDTH], buf, size);

	msg.flags = 0;
	msg.addr = client->addr;
	msg.len = size;
	msg.buf = data;

	while(retried < 5){
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)break;
		retried++;
	}
	if(retried >= 5){
		GT_ERROR("I2C write: 0x%04x, %d bytes failed error code: %d.",
				reg, size, ret);
	}
	return ret;
}

void gt_touch_down(struct gt9147_data *gt, int32_t id, int32_t x,
					int32_t y, int32_t w){
	input_mt_slot(gt->input, id);
	input_report_abs(gt->input, ABS_MT_TRACKING_ID, id);
	input_report_abs(gt->input, ABS_MT_POSITION_X, x);
	input_report_abs(gt->input, ABS_MT_POSITION_Y, y);
	input_report_abs(gt->input, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(gt->input, ABS_MT_WIDTH_MAJOR, w);
	GT_DEBUG("Touch id: [%2d] down.", id);
}

void gt_touch_up(struct gt9147_data *gt, int32_t id){
	input_mt_slot(gt->input, id);
	input_report_abs(gt->input, ABS_MT_TRACKING_ID, -1);
	GT_DEBUG("Touch id: [%2d] release.", id);
}

void gt_enable_irq(struct gt9147_data *gt){
	unsigned long irqflags;
	spin_lock_irqsave(&gt->irq_lock, irqflags);
	if(!gt->gt_data->is_irq_disable){
		gt->gt_data->is_irq_disable = 0;
		enable_irq(gt->client->irq);
	}
	spin_unlock_irqrestore(&gt->irq_lock, irqflags);
}

void gt_disable_irq(struct gt9147_data *gt){
	unsigned long irqflags;
	spin_lock_irqsave(&gt->irq_lock, irqflags);
	if(!gt->gt_data->is_irq_disable){
		gt->gt_data->is_irq_disable = 1;
		disable_irq_nosync(gt->client->irq);
	}
	spin_unlock_irqrestore(&gt->irq_lock, irqflags);
}

void gt_work_func(struct work_struct *work){
	struct gt9147_data *gt = container_of(work, struct gt9147_data, work);
	static uint16_t pre_index;
	uint16_t cur_index=0;
	uint8_t point_data;
	uint8_t touch_num, id, report_num=0, temp=0;
	uint16_t tp_x, tp_y, tp_w;
	uint8_t buf[GT_TP_LEN * MAX_SUPORT_FINGER], *buf_off;
	int ret = 0;
	int i;

	ret = gt_i2c_read(gt->client, GT_GSTID_REG, &point_data, 1);
	if(ret<0){
		GT_ERROR("I2C transfer error. ERRNO: %d.",ret);
	}
	if(point_data == 0)
		return ;
	
	if(point_data & 0x80)
		goto work_exit;

	touch_num = point_data & 0x0f;
	if(touch_num > MAX_SUPORT_FINGER){
		GT_ERROR("Unsuported number of finger: %d.", touch_num);
		goto work_exit;
	}

	if(touch_num || pre_index){
		ret = gt_i2c_read(gt->client, GT_TP1_REG, buf, sizeof(buf));
		if(ret){
			GT_ERROR("I2C transfer failed. ERRNO: %d.", ret);
			goto work_exit;
		}

		buf_off = buf;
		id = buf_off[0] & 0x0f;
		cur_index |= (0x01 << id);
		for(i = 0; i < MAX_SUPORT_FINGER; i++){
			if(cur_index & (0x01 << i)){
				tp_x = ((uint16_t)buf_off[2] << 8) | (buf_off[1]);
				tp_y = ((uint16_t)buf_off[4] << 8) | (buf_off[3]);
				tp_w = ((uint16_t)buf_off[6] << 8) | (buf_off[5]);
				gt_touch_down(gt, id, tp_x, tp_y, tp_w);

				pre_index |= (0x01<<id);
				report_num++;
				if(report_num < touch_num){
					buf_off += GT_TP_LEN;
					id = buf_off[0] & 0x0f;
					cur_index |= (0x01 << id);
				}
			}
			else{
				gt_touch_up(gt, id);
				pre_index &= ~(0x01 << id);
			}
		}
	}
	input_sync(gt->input);

work_exit:
	ret = gt_i2c_write(gt->client, GT_GSTID_REG, &temp, 1);
	if(ret){
		GT_ERROR("I2C write GSTID ERROR. ERRNO: %d.", ret);
	}
	if(gt->gt_data->use_irq){
		gt_enable_irq(gt);
	}
}


irqreturn_t gt_irq_handler(int irq, void *data){
	struct gt9147_data *gt = data;
	gt_disable_irq(gt);
	schedule_work(&gt->work);
	return IRQ_HANDLED;
}

void gt_timer_handler(unsigned long data){
	struct gt9147_data *gt = (struct gt9147_data *)data;
	schedule_work(&gt->work);
	mod_timer(&gt->timer, GT_TIMER_EXPIRES);
}

int gt_i2c_test(struct i2c_client *client){
	int ret = 0;
	int retried = 0;
	uint8_t buf;
	while(retried++ < 5){
		ret = gt_i2c_read(client, GT_CFGS_REG, &buf, 1);
		if(ret<0){
			GT_ERROR("GT I2C test failed time: %d.", retried);
			msleep(10);
		}
	}
	return ret;
}

int gt_read_version(struct i2c_client *client){
	int ret = 0;
	struct gt9147_data *gt9147_dev = i2c_get_clientdata(client);
	uint8_t buf[6];
	uint8_t id_str[5];
	ret = gt_i2c_read(client, GT_PID_REG, buf, sizeof(buf));
	if(ret<0){
		GT_ERROR("Failed to read gt version.");
		return ret;
	}
	memcpy(id_str, buf, 4);
	id_str[4] = '\0';
	ret = kstrtou16(id_str, 10, &gt9147_dev->gt_data->id);

	gt9147_dev->gt_data->version = get_unaligned_le16(&buf[4]);

	GT_INFO("ID: %d, Version: %d", 
			gt9147_dev->gt_data->id, gt9147_dev->gt_data->version);
	return 0;
}

int gt_get_chip_data(struct i2c_client *client, 
					struct gt9147_pri_data *gt){
	int ret = 0;
	ret = gt_i2c_read(client, GT_CFGS_REG, config, GT_CONFIG_MAX_LENGTH);
	if(ret){
		GT_ERROR("Read config data failed. Using default info.");
		gt->abs_x_max = GT_DEFAULT_X_MAX;
		gt->abs_y_max = GT_DEFAULT_Y_MAX;
		gt->irq_trigger = GT_DEFAULT_TRIGGER;
	}
	else{
		gt->abs_x_max = (uint16_t)config[RESULTION_LOC+1]<<8 | config[RESULTION_LOC];
		gt->abs_x_max = (uint16_t)config[RESULTION_LOC+3]<<8 | config[RESULTION_LOC+2];
		gt->irq_trigger = config[TRIGGER_LOC] & 0x03;
	}
	GT_INFO("abs_x_max: %d, abs_y_max: %d, trigger: %d.",
			gt->abs_x_max, gt->abs_y_max, gt->irq_trigger);
	return 0;
}

int gt_request_input_dev(struct gt9147_data *gt_dev){
	int ret = 0;
	struct input_dev *input = input_allocate_device();
	if(!gt_dev->input){
		GT_ERROR("Allocate input device failed.");
		return -ENOMEM;
	}
	gt_dev->input = input;

	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_SYN, input->evbit);
	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	ret = input_mt_init_slots(gt_dev->input, MAX_SUPORT_FINGER, 0);

	input_set_abs_params(gt_dev->input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(gt_dev->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(gt_dev->input, ABS_MT_POSITION_X, 0, gt_dev->gt_data->abs_x_max, 0, 0);
	input_set_abs_params(gt_dev->input, ABS_MT_POSITION_Y, 0, gt_dev->gt_data->abs_y_max, 0, 0);

	input->name = GT_NAME;
	input->phys = GT_INPUT_PHY;
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0XDEAD;
	input->id.product = 0xBEEF;
	input->id.version = 10427;

	ret = input_register_device(gt_dev->input);
	if(ret){
		GT_ERROR("Register %s input device failed.", input->name);
		return ret;
	}
	return 0;
}


int gt_request_irq(struct gt9147_data *gt){
	int ret = 0;
	const uint8_t irq_table[] = IRQ_TABLE;
	ret = devm_request_threaded_irq(&gt->client->dev, gt->client->irq, 
									NULL, gt_irq_handler,
									irq_table[gt->gt_data->irq_trigger],
									gt->client->name, (void*)gt);
	if(ret){
		GT_ERROR("Request irq failed. ERRNO: %d.", ret);
		init_timer(&gt->timer);
		gt->timer.expires = GT_TIMER_EXPIRES;
		gt->timer.data = (unsigned long)gt;
		gt->timer.function = gt_timer_handler;
		add_timer(&gt->timer);
		return -1;
	}
	else{
		gt_disable_irq(gt);
		gt->gt_data->use_irq = 1;
	}
	return 0;
}

void gt_reset_guitar(struct gt9147_pri_data *gt, int ms){
	gpio_set_value(gt->gt_res_gpio, 0);		//begin select i2c slave
	mdelay(ms);								//T2>10ms
	//HIGH: 0x28/0x29 	LOW:0xBA/0xBB
	gpio_set_value(gt->gt_int_gpio, gt->addr == 0x14);		
	mdelay(2);								//T3>100um
	gpio_set_value(gt->gt_res_gpio, 1);
	mdelay(6);								//T4>5ms
	gpio_direction_input(gt->gt_res_gpio);	//end select i2c slave
	gpio_set_value(gt->gt_res_gpio, 1);		//pull up reset gpio

	gpio_set_value(gt->gt_int_gpio, 1);
	mdelay(50);
	gpio_direction_input(gt->gt_int_gpio);
}

int gt_parse_dt(struct gt9147_pri_data *gt, struct device *dev){
	gt->gt_int_gpio = of_get_named_gpio(dev->of_node, "goodix,irq-gpio",0);
	if(!gpio_is_valid(gt->gt_int_gpio)){
		GT_ERROR("Fails to get int gpio.");
		return -1;
	}
	gt->gt_res_gpio = of_get_named_gpio(dev->of_node, "goodix,rst-gpio",0);
	if(!gpio_is_valid(gt->gt_int_gpio)){
		GT_ERROR("Fails to get reset gpio.");
		return -1;
	}
	return 0;
}

int gt_request_io_port(struct gt9147_pri_data *gt, struct device *dev){
	int ret = 0;
	ret = devm_gpio_request_one(dev, gt->gt_int_gpio, 
							GPIOF_OUT_INIT_HIGH, "Goodix-TS,IRQ-PIN");
	if(ret){
		GT_ERROR("Failed to request GPIO: %d. ERRNO: %d",
				gt->gt_int_gpio, ret);
		return -ENODEV;
	}
	ret = devm_gpio_request_one(dev, gt->gt_res_gpio,
							GPIOF_OUT_INIT_HIGH, "Goodix-TS,RET-PIN");
	if(ret){
		GT_ERROR("Failed to request GPIO: %d. ERRNO: %d",
				gt->gt_res_gpio, ret);
		return -1;
	}
	gt_reset_guitar(gt, 20);
	return 0;
}

static int gt9147_probe(struct i2c_client *client, const struct i2c_device_id *device_id_table){
	int ret = 0;
	struct gt9147_pri_data *gt;
	
	gt = devm_kzalloc(&client->dev, sizeof(*gt), GFP_KERNEL);
	if(gt == NULL){
		GT_ERROR("Allocate GFP_KERNEL memery failed.");
		return -ENOMEM;
	}
	gt9147_dev.gt_data = gt;
	gt9147_dev.client = client;

	if(client->dev.of_node){
		ret = gt_parse_dt(gt, &client->dev);
	}
	if(ret){
		GT_ERROR("Failed to parse device tree.");
		goto free_gt;
	}
	
	ret = gt_request_io_port(gt, &client->dev);
	if(ret){
		GT_ERROR("Failed to request io port.");
		goto free_gt;
	}
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
		GT_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}

	INIT_WORK(&gt9147_dev.work, gt_work_func);
	spin_lock_init(&gt9147_dev.irq_lock);
	i2c_set_clientdata(client, &gt9147_dev);

	ret = gt_read_version(client);
	if(ret){
		GT_ERROR("Read version failed.");
	}
	ret = gt_i2c_test(client);
	if(ret){
		GT_ERROR("I2C communication ERROR.");
		goto free_gt;
	}

	ret = gt_get_chip_data(client, gt);
	
	ret = gt_request_input_dev(&gt9147_dev);
	if(ret){
		GT_ERROR("GT request input device failed.\r\n");
		goto free_input_dev;
	}
	/* 申请中断 */
	ret = gt_request_irq(&gt9147_dev);
	if(ret){
		GT_INFO("GT9147 work int polling mode.");
	}
	else{
		GT_INFO("GT9147 work int interrupt mode.");
	}
	if(gt->use_irq)
		gt_enable_irq(&gt9147_dev);

	return 0;

free_input_dev:
	input_free_device(gt9147_dev.input);
free_gt:
	devm_kfree(&client->dev, gt);

	return ret;
}

static int gt9147_remove(struct i2c_client *client){
	input_unregister_device(gt9147_dev.input);
	input_free_device(gt9147_dev.input);
	devm_kfree(&client->dev, gt9147_dev.gt_data);
	return 0;
}

static struct of_device_id match_table[] = {
	{.compatible = DTSCOMPATIBLE},
	{}
};

static struct i2c_device_id i2c_match_table[] = {
	{.name = DTSCOMPATIBLE, .driver_data = 0},
	{}
};

static struct i2c_driver gt9147_driver = {
	.driver = {
		.name = GT_NAME,
		.of_match_table = match_table
	},
	.id_table = i2c_match_table,
	.probe = gt9147_probe,
	.remove = gt9147_remove,
};

module_i2c_driver(gt9147_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("yuzangjiang");