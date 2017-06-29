
KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

ifeq ($(CONFIG_ARCH_MDMCALIFORNIUM), y)
obj-m += alx.o
CDEFINES += -DMDM_PLATFORM
else
obj-$(CONFIG_ALX) += alx.o
endif
alx-objs := alx_main.o alx_ethtool.o alc_cb.o alc_hw.o alf_cb.o alf_hw.o
ccflags-y += -D__CHECK_ENDIAN__

CDEFINES += -D__CHECK_ENDIAN__

KBUILD_CPPFLAGS += $(CDEFINES)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(shell pwd) modules
modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(shell pwd) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean
