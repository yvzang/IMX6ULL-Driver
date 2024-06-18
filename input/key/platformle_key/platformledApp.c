#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm-generic/ioctl.h>


#define CMD_TIMER_START							_IO(0XFE, 0X01)
#define CMD_TIMER_STOP							_IO(0XFE, 0X02)
#define CMD_TIMER_RESTART						_IO(0XFE, 0X03)

int main(int argc, char *args[]){
	int fd;
	int ret = 0;
	const char *filename = args[1];
	char buff[1];
	if(argc < 3){
		printf("number of arguments less 1.\r\n");
		return -1;
	}
	/* 打开设备 */
	fd = open(filename, O_RDWR);
	if(fd < 0){
		printf("faild to open %s\r\n", filename);
		return -1;
	}
	/* 控制设备 */
	int args2 = atoi(args[2]);
	unsigned long arg;
	if(args2 ==1 || args2 == 2){
		if(argc < 4){
			printf("number of arguments less 1.\r\n");
			return -1;
		}
		arg = atoi(args[3]);
	}
	switch(args2){
		case 1:
			ret = ioctl(fd, CMD_TIMER_START, arg);
			break;
		case 2:
			ret = ioctl(fd, CMD_TIMER_RESTART, arg);
			break;
		case 3:
			ret = ioctl(fd, CMD_TIMER_STOP);
			break;
		default:
			printf("unknown ioctl command!\r\n");;
			break;
	}
	if(ret < 0){
		printf("fails to ctrl device!\r\n");
		return -1;
	}
	
	/* 关闭设备 */
	ret = close(fd);
	if(ret < 0){
		printf("fails to close %s\r\n", filename);
		return -1;
	}
}