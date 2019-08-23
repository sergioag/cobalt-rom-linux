/*
 * $Id: i2c.c,v 1.19 2002/09/17 23:41:29 sparker Exp $
 * i2c.c : Cobalt I2C driver support
 *
 * Copyright (C) 2000 Cobalt Networks, Inc. 
 * Copyright (C) 2001 Sun Microsystems, Inc. 
 *
 * This should be SMP safe.  All the exported functions lock on enter and
 * unlock on exit.  These exported functions may be called at interupt time,
 * so we have to use the IRQ safe locks.  NOTE: no function herein may call 
 * any exported function herein. --TPH
 */
#include <stddef.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/config.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <asm/io.h>

#include <cobalt/cobalt.h>
#include <cobalt/i2c.h>
#include <cobalt/systype.h>

#define I2C_3K_STATUS			0x00
#define I2C_3K_CMD			0x01
#define I2C_3K_START			0x02
#define I2C_3K_ADDR			0x03
#define I2C_3K_LOW_DATA			0x04
#define I2C_3K_HIGH_DATA		0x05
#define I2C_3K_BLOCK_DATA		0x06
#define I2C_3K_INDEX			0x07
#define I2C_3K_STATUS_IDLE		0x04
#define I2C_3K_CMD_RW_BYTE		0x20
#define I2C_3K_CMD_RW_WORD		0x30
#define I2C_3K_CMD_RW_BLOCK		0xC0
#define I2C_3K_CMD_RESET_PTR		0x80

#define I2C_5K_HOST_STATUS		0x00
#define I2C_5K_SLAVE_STATUS		0x01
#define I2C_5K_HOST_CONTROL		0x02
#define I2C_5K_HOST_COMMAND		0x03
#define I2C_5K_HOST_ADDR		0x04
#define I2C_5K_DATA_0			0x05
#define I2C_5K_DATA_1			0x06
#define I2C_5K_BLOCK_DATA		0x07
#define I2C_5K_SLAVE_CONTROL		0x08
#define I2C_5K_SHADOW_COMMAND		0x09
#define I2C_5K_SLAVE_EVENT		0x0a
#define I2C_5K_SLAVE_DATA		0x0c
#define I2C_5K_HOST_STATUS_BUSY		0x01
#define I2C_5K_HOST_CMD_START		0x40
#define I2C_5K_HOST_CMD_QUICK_RW	(0 << 2)
#define I2C_5K_HOST_CMD_BYTE_RW		(1 << 2)
#define I2C_5K_HOST_CMD_BYTE_DATA_RW	(2 << 2)
#define I2C_5K_HOST_CMD_WORD_DATA_RW	(3 << 2)
#define I2C_5K_HOST_CMD_BLOCK_DATA_RW	(5 << 2)

#define I2C_WRITE			0
#define I2C_READ			1

/* this delay was determined empirically */
#define I2C_WRITE_UDELAY                1000

struct cobalt_i2c_data {
	const unsigned char status;
	const unsigned char addr;
	const unsigned char index;
	const unsigned char data_low;
	const unsigned char data_high;
	const unsigned char data_block;
	const unsigned char rw_byte;
	const unsigned char rw_word;
	const unsigned char rw_block;
	unsigned int io_port;
};

struct cobalt_i2c_data cobalt_i2c_3k = {
	I2C_3K_STATUS,
	I2C_3K_ADDR,
	I2C_3K_INDEX,
	I2C_3K_LOW_DATA,
	I2C_3K_HIGH_DATA,
	I2C_3K_BLOCK_DATA,
	I2C_3K_CMD_RW_BYTE,
	I2C_3K_CMD_RW_WORD,
	I2C_3K_CMD_RW_BLOCK,
	0L
};

struct cobalt_i2c_data cobalt_i2c_5k = {
	I2C_5K_HOST_STATUS,
	I2C_5K_HOST_ADDR,
	I2C_5K_HOST_COMMAND,
	I2C_5K_DATA_0,
	I2C_5K_DATA_1,
	I2C_5K_BLOCK_DATA,
	I2C_5K_HOST_CMD_BYTE_DATA_RW,
	I2C_5K_HOST_CMD_WORD_DATA_RW,
	I2C_5K_HOST_CMD_BLOCK_DATA_RW,
	0L
};

/* a global pointer for our i2c data */
struct cobalt_i2c_data *i2c_data;

#define I2C_REG(r)			(i2c_data->io_port + i2c_data->r)
#define I2C_CMD(c)			(i2c_data->c)

#define I2C_LOCK	(1 << 0)
#define I2C_DEAD	(1 << 1)
static int i2c_state;

static int initialized;

static inline int 
do_i2c_lock(void)
{
	int i = 0;

	if (test_bit(I2C_DEAD, &i2c_state))
		return -1;

	while (test_and_set_bit(I2C_LOCK, &i2c_state)) {
		if (i++ > 5)
			return -1;
		udelay(10);
	}
	udelay(1);
	return 0;
}

static inline void 
do_i2c_unlock(void)
{
	clear_bit(I2C_LOCK, &i2c_state);
}

/* do a little squelching */
#define NOISE_RATE (5*HZ)
static int 
i2c_noisy(void)
{
	static unsigned long last_time;
	static unsigned int messages;

	if ((long) (jiffies - last_time) > NOISE_RATE) {
		last_time = jiffies;
		if (messages) {
			WPRINTK("skipped %u kernel messages\n", messages);
			messages = 0;
		}
		return 0;
	}
	messages++;
	return 1;
}

static int 
i2c_wait_for_smi(void)
{
	static unsigned int shutup = 0;
	int timeout=10;
	int status;

	while (timeout--) {
		udelay(100); /* wait */
		status = inb_p(I2C_REG(status));

		if (cobt_is_3k()) {
			if (status & I2C_3K_STATUS_IDLE) {
				return 0;
			}
		} else if (cobt_is_5k()) {
			if (!(status & I2C_5K_HOST_STATUS_BUSY)) {
				return 0;
			}
		}
		outb_p(status, I2C_REG(status));
	}

	/* still busy - complain */
	if (!i2c_noisy()) {
		if (++shutup > 2) {
			EPRINTK("i2c seems to be dead - sorry\n");
			set_bit(I2C_DEAD, &i2c_state);
		} else {
			WPRINTK("i2c timeout: status busy (0x%x), resetting\n",
				status);
		}
	}

	/* punch the abort bit */
	if (cobt_is_3k()) {
		outb_p(4, i2c_data->io_port + I2C_3K_CMD);
	} else if (cobt_is_5k()) {
		outb_p(2, i2c_data->io_port + I2C_5K_HOST_CONTROL);
		outb_p(1, i2c_data->io_port + I2C_5K_HOST_CONTROL);
	}

	return -1;
}

static inline int 
i2c_setup(const int dev, const int index, const int r)
{
	if (i2c_wait_for_smi() < 0)
		return -1;

	/* clear status */
	outb_p(0xff, I2C_REG(status));

	/* device address */
	outb_p((dev|r) & 0xff, I2C_REG(addr));

	/* I2C index */
	outb_p(index & 0xff, I2C_REG(index));

	return 0;
}
	
static inline int 
i2c_cmd(const unsigned char command)
{
	if (cobt_is_3k()) {
		outb_p(command, i2c_data->io_port + I2C_3K_CMD); 
		outb_p(0xff, i2c_data->io_port + I2C_3K_START);
	} else if (cobt_is_5k()) {
		outb_p(I2C_5K_HOST_CMD_START | command,
			i2c_data->io_port + I2C_5K_HOST_CONTROL);
	}

	if (i2c_wait_for_smi() < 0)
		return -1;

	return 0;
}

int 
cobalt_i2c_init(void)
{
	struct pci_dev *i2cdev = NULL;

        if( ! initialized ) {
		if (cobt_is_3k()) {
			i2c_data = &cobalt_i2c_3k;
			i2cdev = pci_find_device(PCI_VENDOR_ID_AL, 
				PCI_DEVICE_ID_AL_M7101, NULL);
			if (!i2cdev) {
				EPRINTK("can't find PMU for i2c access\n");
				return -1;
			}
			pci_read_config_dword(i2cdev, 0x14, &i2c_data->io_port);
		} else if (cobt_is_5k()) {
			i2c_data = &cobalt_i2c_5k;
			i2cdev = pci_find_device(PCI_VENDOR_ID_SERVERWORKS,
				PCI_DEVICE_ID_SERVERWORKS_OSB4, i2cdev);
			if (!i2cdev) {
			    i2cdev = pci_find_device(PCI_VENDOR_ID_SERVERWORKS,
			    PCI_DEVICE_ID_SERVERWORKS_CSB5, i2cdev);
			    if (!i2cdev) {
				EPRINTK("can't find OSB4 or CSB5 for i2c access\n");
				return -1;
			    }
			}
			pci_read_config_dword(i2cdev, 0x90, &i2c_data->io_port);
		}

		i2c_data->io_port &= 0xfff0;
		if (!i2c_data->io_port) {
			EPRINTK("i2c IO port not found\n");
           	}
		initialized = 1;
	}

	return 0;
}

int 
cobalt_i2c_reset(void)
{
	int r;

	if( !initialized ) {
		if( cobalt_i2c_init() < 0 )
			return -1;
	}

       	if (do_i2c_lock() < 0)
		return -1;

	if (cobt_is_3k()) {
		/* clear status */
		outb_p(0xff, i2c_data->io_port + I2C_3K_STATUS);
		/* reset SMB devs */
		outb_p(0x08, i2c_data->io_port + I2C_3K_CMD);
		/* start command */
		outb_p(0xff, i2c_data->io_port + I2C_3K_START);
	} else if (cobt_is_5k()) {
		/* clear status */
		outb_p(0x2, i2c_data->io_port + I2C_5K_HOST_CONTROL);
		outb_p(0x1, i2c_data->io_port + I2C_5K_HOST_CONTROL);
		outb_p(0xff, i2c_data->io_port + I2C_5K_HOST_STATUS);
		outb_p(I2C_5K_HOST_CMD_START | 0x08, 
			i2c_data->io_port + I2C_5K_HOST_CONTROL);
	}

	r = i2c_wait_for_smi();

	do_i2c_unlock();

	return r;
}

int 
cobalt_i2c_read_byte(const int dev, const int index)
{
	int val = 0;

	if( !initialized ) {
		if( cobalt_i2c_init() < 0 )
			return -1;
	}

	if (do_i2c_lock() < 0)
		return -1;

	if (i2c_setup(dev, index, I2C_READ) < 0 
	 || i2c_cmd(I2C_CMD(rw_byte)) < 0) {
		val = -1;
	}

	if (val == 0) {
		val = inb_p(I2C_REG(data_low));
	}

	do_i2c_unlock();

	return val;
}

int 
cobalt_i2c_read_word(const int dev, const int index)
{
	int val = 0;

	if( !initialized ) {
		if( cobalt_i2c_init() < 0 )
			return -1;
	}

	if (do_i2c_lock() < 0)
		return -1;
	
	if (i2c_setup(dev, index, I2C_READ) < 0 
	 || i2c_cmd(I2C_CMD(rw_word)) < 0) {
		val = -1;
	}

	if (val == 0) {
		val = inb_p(I2C_REG(data_low));
		val += inb_p(I2C_REG(data_high)) << 8;
	}

	do_i2c_unlock();

	return val;
}

int 
cobalt_i2c_read_block(const int dev, const int index, 
	unsigned char *data, int count)
{
	if( !initialized ) {
		if( cobalt_i2c_init() < 0 )
			return -1;
	}

	if (do_i2c_lock() < 0)
		return -1;
	
	if (i2c_setup(dev, index, I2C_READ) < 0) { 
		do_i2c_unlock();
		return -1;
	}

	outb_p(count & 0xff, I2C_REG(data_low));
	outb_p(count & 0xff, I2C_REG(data_high));

	if (i2c_cmd(I2C_CMD(rw_block)) < 0) {
		do_i2c_unlock();
		return -1;
	}

	while (count) {
		/* read a byte of block data */
		*data = inb_p(I2C_REG(data_block));
		data++;
		count--;
	}

	do_i2c_unlock();

	return 0;	
}

int 
cobalt_i2c_write_byte(const int dev, const int index, const u8 val)
{
	int r = 0;

	if( !initialized ) {
		if( cobalt_i2c_init() < 0 )
			return -1;
	}

	if (do_i2c_lock() < 0)
		return -1;

	if (i2c_setup(dev, index, I2C_WRITE) < 0) {
		r = -1;
	}

	if (r == 0) {
		outb_p(val & 0xff, I2C_REG(data_low));

		if (i2c_cmd(I2C_CMD(rw_byte)) < 0) {
			r = -1;
		}
	}

 	udelay(I2C_WRITE_UDELAY);

	do_i2c_unlock();

	return r;	
}

int 
cobalt_i2c_write_word(const int dev, const int index, const u16 val)
{
	int r = 0;

	if( !initialized ) {
		if( cobalt_i2c_init() < 0 )
			return -1;
	}

	if (do_i2c_lock() < 0)
		return -1;

	if (i2c_setup(dev, index, I2C_WRITE) < 0) {
		r = -1;
	}

	if (r == 0) {
		outb_p(val & 0xff, I2C_REG(data_low));
		outb_p((val >> 8) & 0xff, I2C_REG(data_high));

		if (i2c_cmd(I2C_CMD(rw_word)) < 0) {
			r = -1;
		}
	}
        
 	udelay(I2C_WRITE_UDELAY);

	do_i2c_unlock();

	return r;	
}

int 
cobalt_i2c_write_block(int dev, int index, unsigned char *data, int count)
{
	if( !initialized ) {
		if( cobalt_i2c_init() < 0 )
			return -1;
	}

	if (do_i2c_lock() < 0)
		return -1;

	if (i2c_setup(dev, index, I2C_WRITE) < 0) {
		do_i2c_unlock();
		return -1;
	}

	outb_p(count & 0xff, I2C_REG(data_low));
	outb_p(count & 0xff, I2C_REG(data_high));

	if (i2c_cmd(I2C_CMD(rw_block)) < 0) {
		do_i2c_unlock();
		return -1;
	}

	while (count) {
		/* write a byte of block data */
		outb_p(*data, I2C_REG(data_block));
		data++;
		count--;
	}

 	udelay(I2C_WRITE_UDELAY);

	do_i2c_unlock();

	return 0;	
}

EXPORT_SYMBOL(cobalt_i2c_reset);
EXPORT_SYMBOL(cobalt_i2c_read_byte);
EXPORT_SYMBOL(cobalt_i2c_read_word);
EXPORT_SYMBOL(cobalt_i2c_read_block);
EXPORT_SYMBOL(cobalt_i2c_write_byte);
EXPORT_SYMBOL(cobalt_i2c_write_word);
EXPORT_SYMBOL(cobalt_i2c_write_block);
