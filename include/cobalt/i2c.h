/*
 * $Id: cobalt-i2c.h,v 1.3 2001/08/22 05:48:04 asun Exp $
 * cobalt-i2c.h : I2C support for LCD/Front Panel
 *
 * Copyright 2000 Cobalt Networks, Inc.
 * Copyright 2001 Sun Microsystems, Inc.
 */
#ifndef COBALT_I2C_H
#define COBALT_I2C_H

#include <linux/types.h>
#include <cobalt/cobalt.h>

#define COBALT_I2C_DEV_LED_I		0x40
#define COBALT_I2C_DEV_LED_II		0x42
#define COBALT_I2C_DEV_LCD_DATA		0x4a
#define COBALT_I2C_DEV_LCD_INST		0x48
#define COBALT_I2C_DEV_FP_BUTTONS	0x41
#define COBALT_I2C_DEV_DRV_SWITCH	0x45
#define COBALT_I2C_DEV_RULER		0x46
#define COBALT_I2C_DEV_LM77		0x90
#define COBALT_I2C_DEV_ADM1029		0x5e
#define COBALT_I2C_DEV_AT24C02		0xae

#define COBALT_I2C_READ			0x01
#define COBALT_I2C_WRITE		0x00

extern int cobalt_i2c_reset(void);
extern int cobalt_i2c_read_byte(const int dev, const int index);
extern int cobalt_i2c_read_word(const int dev, const int index);
extern int cobalt_i2c_read_block(const int dev, const int index,
				 unsigned char *data, int count);
extern int cobalt_i2c_write_byte(const int dev, const int index,
				 const u8 val);
extern int cobalt_i2c_write_word(const int dev, const int index,
				 const u16 val);
extern int cobalt_i2c_write_block(const int dev, const int index,
				  unsigned char *data, int count);

#endif /* COBALT_I2C_H */
