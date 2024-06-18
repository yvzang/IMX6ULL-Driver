#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <linux/input.h>


#define CMD_READ_KEY0						_IOR('k', 0x00, 0x01)

static int fd;
static struct input_event inputevent;


int main(int argc, char *args[]){
	int ret = 0;
	const char *filename = args[1];
	int flag;
	if(argc < 2){
		printf("number of arguments less 1.\r\n");
		return -1;
	}
	/* 打开设备 */
	fd = open(filename, O_RDWR);
	if(fd < 0){
		printf("faild to open %s\r\n", filename);
		return -1;
	}
	while(1){
		ret = read(fd, &inputevent, sizeof(inputevent));
		if(ret>0){
			switch(inputevent.type){
				case EV_KEY:
					printf("key %d : %s\r\n", inputevent.code,
							inputevent.value ? "press": "release");
					break;
				case EV_SYN:
					break;
				default:
					printf("unknown event: %d\r\n", inputevent.type);
					break;
			}
		}
		else{
			printf("fails to read input event.\r\n");
			goto close;
		}
	}
close:
	/* 关闭设备 */
	ret = close(fd);
	if(ret < 0){
		printf("fails to close %s\r\n", filename);
		return -1;
	}
}