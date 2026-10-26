# Kbuild for tbs6281_drv

obj-m := tbs6281_drv.o
tbs6281_drv-y := src/tbsecp3-core.o \
                 src/tbsecp3-cards.o \
                 src/tbsecp3-i2c.o \
                 src/tbsecp3-dma.o \
                 src/cxd2878.o \
                 src/tbs6281_chrdev.o

ccflags-y += -I$(src)/include -I$(src)/src
