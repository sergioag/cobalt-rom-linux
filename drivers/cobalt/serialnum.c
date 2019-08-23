/* $Id: serialnum.c,v 1.15 2001/10/23 20:15:27 thockin Exp $ */
/*
 *
 *   Author: Philip Gladstone, Axent Technologies
 *           modified for Nat Semi PC[89]7317 by asun@cobalt.com
 *           ported to 2.4.x by thockin@sun.com
 *           alpine serial eeprom by erik.glling@sun.com
 *   Copyright (c) 2000  Axent Technologies, Cobalt Networks
 *   Copyright (c) 2001  Axent Technologies, Sun Microsystems
 *
 *   This module interrogates the DS2401 Silicon Serial Number chip
 *   that is attached to all x86 Cobalt systems.
 *
 *   It exports /proc/cobalt/hostid which is four bytes generated from of 
 *   the id. It can be linked to /var/adm/hostid or /etc/hostid for the 
 *   hostid command to use.
 *
 *   It exports /proc/cobalt/serialnumber which is the entire 64 bit value 
 *   read back (in ascii).
 *
 *   For the guts of the 1 wire protocol used herein, please see the DS2401
 *   specification.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This driver is SMP safe by nature. --TPH
 */
#include <linux/config.h>
#if defined (CONFIG_COBALT_SERNUM) || defined(CONFIG_COBALT_SERNUM_MODULE)

#include <linux/module.h>
#include <linux/stddef.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <asm/io.h>

#include <cobalt/cobalt.h>
#include <cobalt/systype.h>
#include <cobalt/superio.h>
#include <cobalt/serialnum.h>
#include <cobalt/i2c.h>

#define SERNUM_VER	"1.6"

/* dependent on systype */
static unsigned int sn_direction;
static unsigned int sn_output;
static unsigned int sn_input;
static unsigned int sn_mask;

/* 3k style systems */
#define III_SN_DIRECTION	0x7d
#define III_SN_OUTPUT		0x7e
#define III_SN_INPUT		0x7f
#define III_SN_MASK		0x08
static struct pci_dev *id_dev;

/* 5k style systems */
#define V_SN_DIRECTION		(sn_io_base + 0x01)
#define V_SN_OUTPUT		(sn_io_base + 0x00)
#define V_SN_INPUT		(sn_io_base + 0x00)
#define V_SN_MASK		(sn_io_base + 0x01)
static unsigned int sn_io_base;

#define SSN_SIZE	8	/* bytes */
static char ssn_string[SSN_SIZE * 2 + 1];
static unsigned long hostid;
static int debug;
#ifdef CONFIG_PROC_FS
#ifdef CONFIG_COBALT_OLDPROC
static struct proc_dir_entry *proc_hostid;
static struct proc_dir_entry *proc_serialnum;
#endif
static struct proc_dir_entry *proc_chostid;
static struct proc_dir_entry *proc_cserialnum;
#endif

static int
hostid_read(char *buf, char **start, off_t pos, int len, int *eof, void *x)
{
	int plen = sizeof(hostid);
	memcpy(buf, &hostid, sizeof(hostid));
	return cobalt_gen_proc_read(buf, plen, start, pos, len, eof);
}

static int
serialnum_read(char *buf, char **start, off_t pos, int len, int *eof, void *x)
{
	int plen = sizeof(ssn_string);
	sprintf(buf, "%s\n", ssn_string);
	return cobalt_gen_proc_read(buf, plen, start, pos, len, eof);
}

/* set up the requisite IO bits */
static int __init 
io_init(void)
{
	unsigned char data;

	if (cobt_is_3k()) {
		/* The GPIO tied to the ID chip is on the PMU */
		id_dev = pci_find_device(PCI_VENDOR_ID_AL, 
			PCI_DEVICE_ID_AL_M7101, NULL);
		if (!id_dev) {
			EPRINTK("can't find PMU for serialnumber access\n");
			return -ENXIO;
		}

		/* Set input mode on GPIO3 */
		pci_read_config_byte(id_dev, sn_direction, &data);
		if (debug > 1) {
			WPRINTK("read of register 0x%x = 0x%x\n",
				sn_direction, data);
		}
		if (data & sn_mask) {
			pci_write_config_byte(id_dev, sn_direction, 
				data & ~sn_mask);
		}

		/* Set the output value to be 0 */
		pci_read_config_byte(id_dev, sn_output, &data);
		if (debug > 1) {
			WPRINTK("read of register 0x%x = 0x%x\n",
				sn_output, data);
		}
		if (data & sn_mask) {
			pci_write_config_byte(id_dev, sn_output, 
				data & ~sn_mask);
		}
	} else if (cobt_is_5k()) {
		u16 addr;

		addr = superio_ldev_base(PC87317_DEV_GPIO);
		if (addr) {
			u8 val;

			sn_io_base = addr;

			/* set output value to 0 */
			val = inb(sn_direction);
			outb(val | sn_mask, sn_direction);
			data = inb(sn_output);
			if (data & sn_mask) {
				outb(data & ~sn_mask, sn_output);
			}
			/* set to input */
			outb(val & ~sn_mask, sn_direction);
		}
	} else {
		return -ENXIO;
	}

	/* pick proper variables */
	if (cobt_is_3k()) {
		sn_direction = III_SN_DIRECTION;
		sn_output = III_SN_OUTPUT;
		sn_input = III_SN_INPUT;
		sn_mask = III_SN_MASK;
	} else if (cobt_is_5k()) {
		sn_direction = V_SN_DIRECTION;
		sn_output = V_SN_OUTPUT;
		sn_input = V_SN_INPUT;
		sn_mask = V_SN_MASK;
	} else {
		return -1;
	}

	/* Let things calm down */
	udelay(500);
	return 0;
}

/* write out a bit */
static void __init
io_write(int delay)
{
	if (cobt_is_3k()) {
		unsigned char data;
		/* Set output mode on GPIO3 */
		pci_read_config_byte(id_dev, sn_direction, &data);
		pci_write_config_byte(id_dev, sn_direction, data | sn_mask);
		udelay(delay);

		/* Set input mode */
		pci_write_config_byte(id_dev, sn_direction, data & ~sn_mask);
	} else if (cobt_is_5k()) {
		unsigned char direction;

		/* change to output and back */
		direction = inb(sn_direction); 
		outb(direction | sn_mask, sn_direction);
		udelay(delay);
		outb(direction & ~sn_mask, sn_direction);
	}
}

/* read in a bit */
static int __init
io_read(void)
{
	unsigned char data = 0;

	/* Get the input value */
	if (cobt_is_3k()) {
		pci_read_config_byte(id_dev, sn_input, &data);
	} else if (cobt_is_5k()) {
		data = inb(sn_input);
	}

	return (data & sn_mask) ? 1 : 0;
}

static void __init
io_write_byte(unsigned char c)
{
	int i;
	unsigned long flags;

	save_flags(flags);

	for (i = 0; i < 8; i++, c >>= 1) {
		cli();
		if (c & 1) {
			/* Transmit a 1 */
			io_write(5);
			udelay(80);
		} else {
			/* Transmit a 0 */
			io_write(80);
			udelay(10);
		}
		restore_flags(flags);
	}
}

static int __init
io_read_byte(void)
{
	int i;
	int c = 0;
	unsigned long flags;

	save_flags(flags);

	for (i = 0; i < 8; i++) {
		cli();
		io_write(1);	/* Start the read */
		udelay(2);
		if (io_read()) {
			c |= 1 << i;
		}
		udelay(60);
		restore_flags(flags);
	}

	return c;
}

static int __init
get_ssn(unsigned char *buf)
{
	int i;
	unsigned long flags;

	/* 
	 * Alpine does not have a dallas chip.  Instead
	 * we read from an eprom.
	 */
	if (cobt_is_alpine()) {
		for (i = 0; i < 8; i++) {
			buf[i] = cobalt_i2c_read_byte(COBALT_I2C_DEV_AT24C02, 
				12 + i);
		}
		return 0;
	}

	/*
	 * bit-bang the Dallas 2401
	 */

	save_flags(flags);
	cli();

	/* Master Reset Pulse */
	for (i = 0; i < 600; i += 30) {
		if (io_read()) {
			break;
		}
	}

	if (i >= 600) {
		if (debug) {
			EPRINTK("the data line seems to be held low\n");
		}
		restore_flags(flags);
		return -ENXIO;
	}

	io_write(600);

	for (i = 0; i < 300; i += 15) {
		udelay(15);
		if (io_read() == 0) {
			/* We got a presence pulse */
			udelay(600);	/* Wait for things to quiet down */
			break;
		}
	}
	restore_flags(flags);

	if (i >= 300) {
		if (debug)
			EPRINTK("no presence pulse detected\n");
		return -ENXIO;
	}

	io_write_byte(0x33);

	for (i = 0; i < 8; i++) {
		int rc;

		rc = io_read_byte();
		if (rc < 0) {
			return rc;
		}

		*buf++ = rc;
	}

	return 0;
}

int __init
cobalt_serialnum_init(void)
{
	unsigned char ssn[SSN_SIZE];
	int rc;
	int i;

	/* set up for proper IO */
	rc = io_init();
	if (rc) {
		return rc;
	}

	/*
	 * NOTE: the below algorithm CAN NOT be changed.  We have many systems
	 * out there registered with the serial number AS DERIVED by this
	 * algorithm.
	 */

	rc = get_ssn(ssn);
	if (rc) {
		return rc;
	}

	/* Convert to ssn_string */
	for (i = 7; i >= 0; i--) {
		sprintf(ssn_string + (7 - i) * 2, "%02x", ssn[i]);
	}

	/* get four bytes for a pretty unique (not guaranteed) hostid */
	hostid = *(unsigned long *)ssn ^ *(unsigned long *)(ssn+4);

#ifdef CONFIG_PROC_FS
#ifdef CONFIG_COBALT_OLDPROC
	proc_hostid = create_proc_read_entry("hostid", 0, NULL, 
		hostid_read, NULL);
	if (!proc_hostid) {
		EPRINTK("can't create /proc/hostid\n");
	}
	proc_serialnum = create_proc_read_entry("serialnumber", 0, NULL,
		serialnum_read, NULL);
	if (!proc_serialnum) {
		EPRINTK("can't create /proc/serialnumber\n");
	}
#endif
	proc_chostid = create_proc_read_entry("hostid", 0, proc_cobalt, 
		hostid_read, NULL);
	if (!proc_chostid) {
		EPRINTK("can't create /proc/cobalt/hostid\n");
	}
	proc_cserialnum = create_proc_read_entry("serialnumber", 0, 
		proc_cobalt, serialnum_read, NULL);
	if (!proc_cserialnum) {
		EPRINTK("can't create /proc/cobalt/serialnumber\n");
	}
#endif

	return 0;
}

char *
cobalt_serialnum_get(void)
{
	return ssn_string;
}

unsigned long
cobalt_hostid_get(void)
{
	return hostid;
}

#if defined(CONFIG_COBALT_SERNUM_MODULE)
MODULE_PARM(debug, "i");

int
init_module(void)
{
	return cobalt_serialnum_init();
}

void
cleanup_module(void)
{
#ifdef CONFIG_PROC_FS
#ifdef CONFIG_COBALT_OLDPROC
	remove_proc_entry("hostid", NULL);
	remove_proc_entry("serialnumber", NULL);
#endif
	remove_proc_entry("hostid", proc_cobalt);
	remove_proc_entry("serialnumber", proc_cobalt);
#endif
}

module_init(init_module);
module_exit(cleanup_module);
#endif /* MODULE */

#endif /* CONFIG_COBALT_SERNUM */
