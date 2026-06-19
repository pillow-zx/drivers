ifneq ($(KERNELRELEASE),)

MODULE_NAME := $(notdir $(M))

obj-m += $(MODULE_NAME).o
$(MODULE_NAME)-y := uart.o

else

ARCH ?= arm
CROSS_COMPILE ?= arm-linux-gnueabihf-
KDIR ?= /home/pillow/code/kernel/linux-imx
PWD := $(CURDIR)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) \
		ARCH=$(ARCH) \
		CROSS_COMPILE=$(CROSS_COMPILE) \
		modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

endif
