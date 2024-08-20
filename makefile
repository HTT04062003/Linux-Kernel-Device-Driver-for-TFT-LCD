obj-m += tft_driver.o

all: module dt
        echo Builded Device Tree Overlay and kernel module

module:
        $(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules EXTRA_CFLAGS=-I/usr/include

dt: testoverlay_2.dts
        dtc -@ -I dts -O dtb -o testoverlay_2.dtbo testoverlay_2.dts

clean:
        $(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
        rm -rf testoverlay_2.dtbo
