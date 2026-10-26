# Makefile for tbs6281_drv

KVER ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVER)/build
PWD := $(shell pwd)
INSTALL_DIR := /lib/modules/$(KVER)/misc

all: modules

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install: modules
	install -D -v -m 644 tbs6281_drv.ko $(INSTALL_DIR)/tbs6281_drv.ko
	depmod -a $(KVER)

uninstall:
	rm -fv $(INSTALL_DIR)/tbs6281_drv.ko
	depmod -a $(KVER)

.PHONY: all modules clean install uninstall
