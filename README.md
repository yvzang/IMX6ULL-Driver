# IMX6ULL Driver
## Introduction
These are the drivers for the IMX6ULL-14x14-EMMC series chips. If you want to use them on your device, please modify the device tree according to your hardware. Refer to the Linux kernel documentation for device tree bindings, such as Documentation/devicetree/bindings. For example, if you want to use the GT9147 multi-touch screen, refer to Documentation/devicetree/bindings/input/touchscreen/goodix.txt. For devices with the same chip, you do not need to modify imx6ull.dtsi.
## Usage
To compile all drivers:
```
make
```
To compile specific drivers, the driver name must be the directory name where the driver file is located:
```
make modules=xxx
```
To compile all drivers except xxx:
```
make EXCLUDE_DIR=xxx
```
After the drivers are compiled, place the xxx.ko file in the Linux directory and use modprobe xxx.ko to load the driver, or compile it into the Linux kernel.
