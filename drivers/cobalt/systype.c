/*
 * $Id: systype.c,v 1.33 2002/11/04 17:54:15 thockin Exp $
 * systype.c : routines for figuring out which Cobalt system this is
 *
 * Copyright 2001-2002 Sun Microsystems, Inc.
 *
 * By:  Tim Hockin
 *	Adrian Sun
 *	Duncan Laurie
 *
 * This driver is SMP safe by nature. --TPH
 */

#include <linux/config.h>

#include <linux/pci.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/module.h>

#include <cobalt/cobalt.h>
#include <cobalt/systype.h>
#include <cobalt/i2c.h>

/* for easy first-pass analysis */
#if defined(CONFIG_COBALT_GEN_III)
int COBALT_GENERATION_III_DEFINED;
#endif
#if defined(CONFIG_COBALT_GEN_V)
int COBALT_GENERATION_V_DEFINED;
#endif

cobt_sys_t cobt_type = COBT_UNINITIALIZED;
EXPORT_SYMBOL(cobt_type);
unsigned long cobt_rev;
EXPORT_SYMBOL(cobt_rev);

#ifdef CONFIG_PROC_FS
static struct proc_dir_entry *proc_systype;
#endif
static int systype_read_proc(char *buf, char **start, off_t pos, int len,
	int *eof, void *x);
static char *systype_str(cobt_sys_t type);
static unsigned long boardrev_read(void);

void __init
cobalt_boardrev_init(void)
{
	cobt_rev = boardrev_read();
}

int __init 
cobalt_systype_init(void)
{
	cobalt_systype_probe();

#ifdef CONFIG_PROC_FS
	proc_systype = create_proc_read_entry("systype", 0, 
		proc_cobalt, systype_read_proc, NULL);
	if (!proc_systype) {
		EPRINTK("can't create /proc/cobalt/systype\n");
	}
#endif

	if (cobt_type == COBT_UNKNOWN) {
		printk("Cobalt system type is unknown (trouble will ensue)\n");
		return -1;
	}

	return 0;
}

#if defined(CONFIG_COBALT_GEN_III)
static cobt_sys_t
systype_probe_3k(void)
{
	struct pci_dev *pdev;
	cobt_sys_t retval = COBT_UNKNOWN;

	/* board identifier for RaQ3/4 vs Qube3 is on the PMU @ 0x7f */
	pdev = pci_find_device(PCI_VENDOR_ID_AL, PCI_DEVICE_ID_AL_M7101, NULL);
	if (pdev) {
		/* 
		 * check to see what board we are on
		 * ( RaQ 3, RaQ 4, Qube 3 )
		 */
		unsigned char val;

		/* momentarily set DOGB# to input */
		pci_read_config_byte(pdev, 0x7d, &val);
		pci_write_config_byte(pdev, 0x7d, val & ~0x20);

		/* read the GPIO register */
		pci_read_config_byte(pdev, 0x7f, &val);
		/* RaQ3/4 boards have DOGB (0x20) high, 
		 * Qube3 has DOGB low */ 
		retval = (val & 0x20) ? COBT_PACIFICA : COBT_CARMEL;

		/* change DOGB back to output */
		pci_read_config_byte(pdev, 0x7d, &val);
		pci_write_config_byte(pdev, 0x7d, val | 0x20);
	}

	/* assign to this, so the compiler shuts up */
	COBALT_GENERATION_III_DEFINED = 1;

	return retval;
}
#else
#define systype_probe_3k()	(COBT_UNKNOWN)
#endif

#if defined(CONFIG_COBALT_GEN_V)
static cobt_sys_t
systype_probe_5k(void)
{
	struct pci_dev *pdev;
	cobt_sys_t retval = COBT_UNKNOWN;

	/* is it a gen V ? */
	pdev = pci_find_device(PCI_VENDOR_ID_SERVERWORKS,
		PCI_DEVICE_ID_SERVERWORKS_OSB4, NULL);
	if (pdev) {
		retval = COBT_MONTEREY;
		goto out;
	}

	pdev = pci_find_device(PCI_VENDOR_ID_SERVERWORKS,
		PCI_DEVICE_ID_SERVERWORKS_CSB5, NULL);
	if (pdev) {
		pdev = pci_find_device(PCI_VENDOR_ID_SERVERWORKS,
			PCI_DEVICE_ID_SERVERWORKS_LE, NULL);
		if (pdev) {
			retval = COBT_ALPINE;
			goto out;
		}
	}

out:
	/* assign to this, so the compiler shuts up */
	COBALT_GENERATION_V_DEFINED = 1;

	return retval;
}
#else
#define systype_probe_5k()	(COBT_UNKNOWN)
#endif

static cobt_sys_t
systype_probe_gp(void)
{
	struct pci_dev *pdev;
	cobt_sys_t retval = COBT_UNKNOWN;

	/* is it a GP system? */
	pdev = pci_find_device(PCI_VENDOR_ID_SERVERWORKS,
		PCI_DEVICE_ID_SERVERWORKS_CSB5, NULL);
	if (pdev) {
		pdev = pci_find_device(PCI_VENDOR_ID_SERVERWORKS,
			PCI_DEVICE_ID_SERVERWORKS_HE, NULL);
		if (pdev) {
			retval = COBT_BIGBEAR;
		}	
	}

	return retval;
}

cobt_sys_t
cobalt_systype_probe(void)
{
	static int init_done = 0;

	if (init_done) {
		return cobt_type;
	}

	/* check for 3k family systems */
	cobt_type = systype_probe_3k();
	if (cobt_type != COBT_UNKNOWN)
		goto out;

	/* check for 5k family systems */
	cobt_type = systype_probe_5k();
	if (cobt_type != COBT_UNKNOWN)
		goto out;

	/* it's a GP system or unknown */
	cobt_type = systype_probe_gp();

out:
	if (cobt_type != COBT_UNKNOWN) {
		init_done = 1;
	}

	return cobt_type;
}
EXPORT_SYMBOL(cobalt_systype_probe);

#ifdef CONFIG_PROC_FS
static int 
systype_read_proc(char *buf, char **start, off_t pos, int len,
	int *eof, void *x)
{
	int plen = sprintf(buf, "%s\n", systype_str(cobt_type));
	return cobalt_gen_proc_read(buf, plen, start, pos, len, eof);
}
#endif

static char *
systype_str(cobt_sys_t type)
{
	switch (type) {
		case COBT_PACIFICA:
			return "Pacifica";
			break;
		case COBT_CARMEL:
			return "Carmel";
			break;
		case COBT_MONTEREY:
			return "Monterey";
			break;
		case COBT_ALPINE:
			return "Alpine";
			break;
		case COBT_BIGBEAR:
			return "BigBear";
			break;
		case COBT_UNKNOWN:
		default:
			return "unknown";
			break;
	}
}

static unsigned long
boardrev_read(void)
{
	unsigned long rev;

	switch (cobt_type) {
#ifdef CONFIG_COBALT_RAQ
	case COBT_PACIFICA:
	case COBT_CARMEL:
		/* No usable board rev on these systems */
		return 0;
	case COBT_MONTEREY:
		/*
		 * the boardrev on monterey is strapped off of GPM[3:0]
		 * and is read from port 0xc52
		 */
		return inb(0xc52);
	case COBT_ALPINE:
		/*
		 * the boardrev on alpine in stored in the i2c eeprom
		 * location 4
		 */
		rev = cobalt_i2c_read_byte(COBALT_I2C_DEV_AT24C02, 0x04);
		rev |= cobalt_i2c_read_byte(COBALT_I2C_DEV_AT24C02, 0x05) << 8;
		rev |= cobalt_i2c_read_byte(COBALT_I2C_DEV_AT24C02, 0x06) << 16;
		rev |= cobalt_i2c_read_byte(COBALT_I2C_DEV_AT24C02, 0x07) << 24;
		if (rev == 0xffffffff)
			rev = 0;
		return rev;
#endif
	case COBT_BIGBEAR:
		/* No board revs at this time */
		return 0;
	case COBT_UNKNOWN:
	case COBT_UNINITIALIZED:
		return 0;
	}
	return 0;
}
