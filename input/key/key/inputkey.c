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
#include <linux/of_address.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <uapi/asm-generic/errno-base.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/poll.h>
#include <uapi/asm-generic/ioctl.h>
#include <asm/signal.h>
#include <linux/input.h>
#include <uapi/linux/input.h>
#include <linux/blkdev.h>


#define CHRDEVKEY_NAME						"alphkey"
#define CHRDEVKEY_NUM						1
#define KEYVALLID							0
#define KEYINVAL							0xff
#define JFDELAYMS(ms)						(jiffies + msecs_to_jiffies(ms))

#define CMD_READ_KEY0						_IOR('k', 0x00, 0x01);

struct irqkey_t{
	int gpio;
	char name[10];
	atomic_t value;
	int irq;
	struct timer_list timer;
	irqreturn_t (*irq_handler)(int, void *);
	void (*timer_handler)(unsigned long);
	int read_flag;
	struct input_dev *keyinput_dev;
};

struct chrdev_t{
	dev_t devnum;
	int major;
	int minor;
	struct cdev chrdev;
	struct class *devclass;
	struct device *device;
	struct device_node* node;
	struct irqkey_t keys[CHRDEVKEY_NUM];
};

static struct chrdev_t chrdevkey;



static irqreturn_t keyirq_handler(int irq, void *irqkey_ptr){
	struct irqkey_t *irqkey = (struct irqkey_t*)irqkey_ptr;
	irqkey->timer.data = (unsigned long)irqkey;
	mod_timer(&irqkey->timer, JFDELAYMS(10));
	return IRQ_RETVAL(1);
}

static void timer_handler(unsigned long arg){
	struct irqkey_t *irqkey = (struct irqkey_t*)arg;
	/* 按键按下 */
	if(!gpio_get_value(irqkey->gpio)){
		atomic_set(&irqkey->value, KEYVALLID);
		printk("key pressed!\r\n");
		/* 上报输入事件 */
		input_report_key(irqkey->keyinput_dev, KEY_0, 1);
		input_sync(irqkey->keyinput_dev);
	}
	else{
		atomic_set(&irqkey->value, KEYINVAL);
		printk("key released!\r\n");
		/* 上报输入事件 */
		input_report_key(irqkey->keyinput_dev, KEY_0, 0);
		input_sync(irqkey->keyinput_dev);
	}
	irqkey->read_flag = 1;
}

int alphkey_init(struct chrdev_t *dev){
	int ret = 0;
	int i;
	struct irqkey_t *curirqkey;
	for(i = 0; i < CHRDEVKEY_NUM; i++){
		curirqkey = &dev->keys[i];
		/* 初始化按键值 */
		atomic_set(&curirqkey->value, 1);
		curirqkey->irq_handler = keyirq_handler;
		curirqkey->timer_handler = timer_handler;
		curirqkey->read_flag = 0;
		/* 申请gpio */
		curirqkey->gpio = of_get_named_gpio(dev->node, "gpios", i);
		if(curirqkey->gpio < 0){
			printk("fails to get gpio!\r\n");
			return -ENXIO;
		}
		memset(curirqkey->name, 0, sizeof(curirqkey->name));
		sprintf(curirqkey->name, "KEY%d", i);
		ret = gpio_request(curirqkey->gpio, curirqkey->name);
		if(ret < 0){
			printk("fails to request gpio!\r\n");
			return -ENXIO;
		}
		/* 设置gpio属性 */
		gpio_direction_input(curirqkey->gpio);
		/* 获取中断号 */
		curirqkey->irq = irq_of_parse_and_map(dev->node, i);
#if 0
		curirqkey->irq = gpio_to_irq(curirqkey->gpio);
#endif
		if(curirqkey < 0){
			printk("fails to parse interrupt node!\r\n");
			return -ENXIO;
		}
		printk("get interrupt number: %d\r\n", curirqkey->irq);
		/* 注册中断 */
		ret = request_irq(curirqkey->irq, curirqkey->irq_handler, 
							IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, 
							curirqkey->name, curirqkey);
		if(ret){
			printk("fails to request interrupt %d!\r\n", curirqkey->irq);
			return ret;
		}
		/* 初始化定时器 */
		curirqkey->timer.function = curirqkey->timer_handler;
		curirqkey->timer.data = (unsigned long)curirqkey;
		init_timer(&curirqkey->timer);
		/* 注册input_dev */
		curirqkey->keyinput_dev = input_allocate_device();
		__set_bit(EV_KEY, curirqkey->keyinput_dev->evbit);
		__set_bit(EV_REP, curirqkey->keyinput_dev->evbit);
		__set_bit(KEY_0, curirqkey->keyinput_dev->keybit);
		ret = input_register_device(curirqkey->keyinput_dev);
	}
	return 0;
}

static int __init keydev_init(void){
	int ret = 0;
	struct property *proper;
	const char *str;
	printk("key character device init..\r\n");
	/* 从设备树获取节点 */
	chrdevkey.node = of_find_node_by_path("/alphkey");
	if(chrdevkey.node < 0){
		printk("fails to find node!\r\n");
		return -ENXIO;
	}
	/* 获取节点compatible属性 */
	proper = of_find_property(chrdevkey.node, "compatible", NULL);
	if(proper == NULL){
		printk("fails to find compatible property!\r\n");
		return -ENXIO;
	}
	else{
		printk("property: %s has been found!\r\n", proper->name);
	}
	/* 获取节点status属性 */
	ret = of_property_read_string(chrdevkey.node, "status", &str);
	if(ret < 0){
		printk("fails to get status of property!\r\n");
		return -ENXIO;
	}
	else{
		printk("get status: %s\r\n", str);
	}
	/* 初始化按键 */
	return alphkey_init(&chrdevkey);
}

static void __exit keydev_exit(void){
	int i;
	for(i = 0; i < CHRDEVKEY_NUM; i++){
		/* 注销input_dev */
		input_unregister_device(chrdevkey.keys[i].keyinput_dev);
		input_free_device(chrdevkey.keys[i].keyinput_dev);
		/* 删除gpio号 */
		gpio_free(chrdevkey.keys[i].gpio);
		free_irq(chrdevkey.keys[i].irq, &chrdevkey.keys[i]);
	}
}


module_init(keydev_init);
module_exit(keydev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("江雨藏");