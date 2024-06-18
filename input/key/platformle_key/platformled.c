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
#include <uapi/asm-generic/param.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <uapi/asm-generic/ioctl.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>


#define CHRDEVLED_NAME						"alphled"
#define DTSCOMPATIBLE						"atcyuzang-led"
#define CHRDEVLED_NUM						1
#define REG_SIZE							4
#define LEDON								1
#define LEDOFF								0
#define JFDELAYTIME(exp)					(jiffies + msecs_to_jiffies(exp))

#define CMD_TIMER_START						_IO(0XFE, 0X01)
#define CMD_TIMER_STOP						_IO(0XFE, 0X02)
#define CMD_TIMER_RESTART					_IO(0XFE, 0X03)

struct chrdevled_t{
	dev_t devnum;
	int major;
	int minor;
	struct cdev chrdev;
	struct class *devclass;
	struct device *leddevice;
	struct device_node* node;
	int led_gpio;
	struct timer_list timer;
	struct mutex mutex;
};

static struct chrdevled_t chrdevled;


void led_switch(int led_gpio, const char turn){
	if(turn){
		gpio_set_value(led_gpio, LEDOFF);
	}
	else{
		gpio_set_value(led_gpio, LEDON);
	}
}

void timer_handler(unsigned long arg){
	static int status = 1;
	status = !status;
	led_switch(chrdevled.led_gpio, status);
	mod_timer(&chrdevled.timer, JFDELAYTIME(arg));
}


static int led_open(struct inode *dev_inode, struct file *dev_file){
	dev_file->private_data = (void*)(&chrdevled);

	return 0;	
}

static int led_release(struct inode *dev_inode, struct file *dev_file){

	return 0;
}

static long led_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg){
	struct chrdevled_t *dev = (struct chrdevled_t*)file->private_data;
	/* 加锁 */
	if(mutex_lock_interruptible(&dev->mutex)){
		return -EINTR;
	}
	switch (cmd)
	{
	/* 启动定时器 */
	case CMD_TIMER_START:
		dev->timer.data = arg;
		mod_timer(&dev->timer, JFDELAYTIME(arg));
		break;
	/* 删除定时器 */
	case CMD_TIMER_STOP:
		del_timer_sync(&dev->timer);
		break;
	/* 重新设置定时器 */
	case CMD_TIMER_RESTART:
		dev->timer.data = arg;
		mod_timer(&dev->timer, JFDELAYTIME(arg));
		break;
	default:
		break;
	}
	/* 解锁 */
	mutex_unlock(&dev->mutex);
	return 0;
}


static struct file_operations ops = {
	.open = led_open,
	.release = led_release,
	.unlocked_ioctl = led_unlocked_ioctl
};


static int key_probe(struct platform_device *pltf_device){
	int ret = 0;
	struct property *proper;
	const char *str;
	printk("chrdevbase init..\r\n");
	/* 初始化锁 */
	mutex_init(&chrdevled.mutex);
	/* 初始化定时器 */
	chrdevled.timer.function = timer_handler;
	init_timer(&chrdevled.timer);
	/* 查找节点 */
	chrdevled.node = of_find_node_by_path("/alphled");
	if(chrdevled.node == NULL){
		printk("fials to find device_node in /alphled/r/n");
		return -ENXIO;
	}
	else{
		printk("alphnode has been found!\r\n");
	}
	/* 获取gpio号 */
	chrdevled.led_gpio = of_get_named_gpio(chrdevled.node, "gpios", 0);
	if(chrdevled.led_gpio < 0){
		printk("fails to get gpio number!\r\n");
		return -ENXIO;
	}
	/* 设置gpio输出 */
	ret = gpio_direction_output(chrdevled.led_gpio, 1);
	if(ret < 0){
		printk("fails to set gpio output!\r\n");
		return -EINTR;
	}
	/* 获取节点compatible属性 */
	proper = of_find_property(chrdevled.node, "compatible", NULL);
	if(proper == NULL){
		printk("fails to find compatible property!\r\n");
		return -ENXIO;
	}
	else{
		printk("property: %s has been found!\r\n", proper->name);
	}
	/* 获取节点status属性 */
	ret = of_property_read_string(chrdevled.node, "status", &str);
	if(ret < 0){
		printk("fails to get status of property!\r\n");
		return -ENXIO;
	}
	else{
		printk("get status: %s\r\n", str);
	}

	/* 分配设备号 */
	if(chrdevled.major){
		ret = register_chrdev_region(chrdevled.devnum, CHRDEVLED_NUM, CHRDEVLED_NAME);
	}
	else{
		ret = alloc_chrdev_region(&chrdevled.devnum, 0, CHRDEVLED_NUM, CHRDEVLED_NAME);
		chrdevled.major =  MAJOR(chrdevled.devnum);
		chrdevled.minor = MINOR(chrdevled.devnum);
	}
	if(ret < 0){
		printk("fails to allocate chrdev_region./r/n");
		return -EINTR;
	}
	else{
		printk("major = %d, minor = %d\r\n", chrdevled.major, chrdevled.minor);
	}
	/* 添加cdev */
	chrdevled.chrdev.owner = THIS_MODULE;
	cdev_init(&chrdevled.chrdev, &ops);
	ret = cdev_add(&chrdevled.chrdev, chrdevled.devnum, CHRDEVLED_NUM);
	if(ret < 0){
		printk("fails to add cdev.\r\n");
		return -EINTR;
	}
	/* 创建class */
	chrdevled.devclass = class_create(THIS_MODULE, CHRDEVLED_NAME);
	if(IS_ERR(chrdevled.devclass)){
		return PTR_ERR(chrdevled.devclass);;
	}
	/* 创建device */
	chrdevled.leddevice = device_create(chrdevled.devclass, NULL,\
							chrdevled.devnum, NULL, CHRDEVLED_NAME);
	if(IS_ERR(chrdevled.leddevice)){
		return PTR_ERR(chrdevled.leddevice);
	}
	return 0;
}

static int key_remove(struct platform_device *pltf_device){
	/* 删除定时器 */
	del_timer_sync(&chrdevled.timer);
	/* 删除cdev */
	cdev_del(&chrdevled.chrdev);
	/* 注销设备号 */
	unregister_chrdev_region(chrdevled.devnum, CHRDEVLED_NUM);
	/* 摧毁device */
	device_destroy(chrdevled.devclass, chrdevled.devnum);
	/* 摧毁class */
	class_destroy(chrdevled.devclass);
	return 0;
}

static struct of_device_id match_table[] = {
	{.compatible = DTSCOMPATIBLE},
	{}
};

static struct platform_driver pltf_driver = {
	.driver = {
		.name = CHRDEVLED_NAME,
		.of_match_table = match_table
	},
	.probe = key_probe,
	.remove = key_remove,
};

static int __init leddev_init(void){
	platform_driver_register(&pltf_driver);
	return 0;
}

static void __exit leddev_exit(void){
	platform_driver_unregister(&pltf_driver);
}

module_init(leddev_init);
module_exit(leddev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("江雨藏");