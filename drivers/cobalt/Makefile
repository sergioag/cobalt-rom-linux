#
# Makefile for the Sun/Cobalt device drivers
#

O_TARGET := cobalt.o

export-objs := init.o systype.o wdt.o i2c.o

obj-$(CONFIG_COBALT)		+= init.o systype.o
obj-$(CONFIG_COBALT_RAQ)	+= i2c.o wdt.o
obj-$(CONFIG_COBALT_ACPI)	+= acpi.o
obj-$(CONFIG_COBALT_SERNUM)	+= serialnum.o
obj-$(CONFIG_COBALT_LCD)	+= lcd.o
obj-$(CONFIG_COBALT_LED)	+= net.o led.o
obj-$(CONFIG_COBALT_SENSORS)	+= sensors.o
obj-$(CONFIG_COBALT_FANS)	+= fans.o
obj-$(CONFIG_COBALT_RAMINFO)	+= raminfo.o
obj-$(CONFIG_COBALT_RULER)	+= ruler.o

include $(TOPDIR)/Rules.make
