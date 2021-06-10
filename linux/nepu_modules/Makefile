obj-m := test_msi_mode.o
KDIR := /home/nepuko/research/gem5_share/linux
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean