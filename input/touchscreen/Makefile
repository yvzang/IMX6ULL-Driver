KERNELDIR ?= "/home/yuzang/linux/linux/linux-imx-rel_imx_4.1.15_2.1.0_ga/"

CURRENT_DIR := $(shell pwd)
obj-m := gt9147.o

build: kernel_modules

kernel_modules:
	@$(MAKE) -C $(KERNELDIR) M=$(CURRENT_DIR) modules

clean:
	@$(MAKE) -C $(KERNELDIR) M=$(CURRENT_DIR) clean