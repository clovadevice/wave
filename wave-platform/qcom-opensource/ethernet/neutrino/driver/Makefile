CURRENT	= $(shell uname -r)
#OBJS	= usbnet.o
TARGET	= ntn_usb_class_drv
ntn_usb_class_drv-objs	= ntn_usb_class_main.o ntn_gmac.o ntn_reg_rw.o ntn_ptp.o
MDIR	= drivers/net/usb
KDIR	= /lib/modules/$(CURRENT)/build
SUBLEVEL= $(shell uname -r | cut -d '.' -f 3 | cut -d '.' -f 1 | cut -d '-' -f 1 | cut -d '_' -f 1)
USBNET	= $(shell find $(KDIR)/include/linux/usb/* -name usbnet.h)

ifneq (,$(filter $(SUBLEVEL),14 15 16 17 18 19 20 21))
MDIR = drivers/usb/net
endif

EXTRA_CFLAGS += -DEXPORT_SYMTAB
PWD = $(shell pwd)
DEST = /lib/modules/$(CURRENT)/kernel/$(MDIR)

obj-m      := $(TARGET).o

default:
	modprobe usbnet
	make -C $(KDIR) SUBDIRS=$(PWD) modules

$(TARGET).o: $(OBJS)
	$(LD) $(LD_RFLAG) -r -o $@ $(OBJS)

install:
	modprobe usbnet
	insmod $(TARGET).ko

remove:
	rmmod $(TARGET)

clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean

.PHONY: modules clean

-include $(KDIR)/Rules.make
