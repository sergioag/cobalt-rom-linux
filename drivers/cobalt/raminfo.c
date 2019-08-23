/* $Id: raminfo.c,v 1.7 2001/10/29 22:21:36 thockin Exp $
 *
 * Copyright (c) 2000-2001 Sun Microsystems, Inc.
 * All Rights Reserved.
 *
 * This is SMP safe - the init runs once on load, and the rest is just
 * printing information. --TPH
 */
#include <linux/config.h>

#if defined(CONFIG_COBALT_RAMINFO) || defined(CONFIG_COBALT_RAMINFO_MODULE)

#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/proc_fs.h>

#include <cobalt/cobalt.h>
#include <cobalt/systype.h>

#define MAX_DIMM_SLOTS	4

enum dimm_t {
	DIMM_TYPE_FPM_DRAM,
	DIMM_TYPE_EDO_DRAM,
	DIMM_TYPE_REG_SDRAM,
	DIMM_TYPE_SDRAM
};

static char *dimm_desc[] = {
	"Fast-page Mode DRAM",
	"EDO DRAM",
	"Registered SDRAM",
	"SDRAM",
};

struct dimm_slot {
	int num;
	enum dimm_t type;
	uint16_t size;
	int ecc;
};

struct raminfo {
	int total;
	int (*query)(struct dimm_slot *);
	struct pci_dev *dev;
	struct dimm_slot *dimm;
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry *proc;
#endif /* CONFIG_PROC_FS */
};

/*########################################################################*/

static int serverworks_le_dimm_info(struct dimm_slot *);
static int ali_1541_dimm_info(struct dimm_slot *);
static int raminfo_read_proc(char*, char**, off_t, int, int*, void*);

/* RaQ-3, RaQ-4, Qube-3
 * - uses ALI M1541 for memory controller
 * - has 2 dimm slots */
static struct raminfo gen3_raminfo = {
	total: 2,
	query: ali_1541_dimm_info
};
/* RaQ-XTR (Monterey)
 * - uses ServerWorks CNB30LE for Memory Controller
 * - has 4 dimm slots */
static struct raminfo gen5_monterey_raminfo = {
	total: 4,
	query: serverworks_le_dimm_info
};
/* RaQ (Alpine)
 * - uses ServerWorks CNB30LE for Memory Controller
 * - has 2 dimm slots */
static struct raminfo gen5_alpine_raminfo = {
	total: 2,
	query: serverworks_le_dimm_info
};

static struct raminfo *sys_raminfo;

/*########################################################################*/

#define SERVERWORKS_DRAM_MRPR		(0x90)
#define SERVERWORKS_DRAM_MRAR(slot)	(0x7c + (slot))
#define SERVERWORKS_DRAM_ECCR		(0xe0)

static int
serverworks_le_dimm_info(struct dimm_slot *dimm)
{
	int row;
	uint8_t rar, active, eccr;
	uint16_t ma_map[] = {
		32, 16, 32, 256, 512, 128, 128, 64, 256, 128, 64, 64, 128,
	};

	if (!sys_raminfo || !sys_raminfo->dev || !dimm)
		return -ENOSYS;

	pci_read_config_byte(sys_raminfo->dev,
			     SERVERWORKS_DRAM_MRPR, &active);
	pci_read_config_byte(sys_raminfo->dev,
			     SERVERWORKS_DRAM_MRAR(dimm->num), &rar);

	/* serverworks uses only registered sdram */
	dimm->type = DIMM_TYPE_REG_SDRAM;
	dimm->size = 0;

	/* check to see if ECC is enabled (bit 4 of reg 0xE0) */
	pci_read_config_byte(sys_raminfo->dev,
			     SERVERWORKS_DRAM_ECCR, &eccr);
	dimm->ecc = (eccr & (1<<2)) ? 1 : 0;

	/* two rows for each dimm slot */
	for (row=2*dimm->num; row<=(2*dimm->num+1); row++) {
		/* each active row will have corresponding bit
		 * set in the Memory Row Presence Register */
		if (active & (1 << row)) {
			/* lookup size ma_map table */
			dimm->size += ma_map[ rar & 0xf ];
		}
		/* two rows per RAR register, bits 7-4 and bits 3-0 */
		rar >>= 4;
	}

	return 0;
}

#define ALI_DRAM_CONF_1(row)		(0x60 + ((row) * 2))
#define ALI_DRAM_CONF_2(row)		(0x61 + ((row) * 2))
#define ALI_DIMM_TYPE(d2)		(((d2) >> 4) & 0x3)
#define ALI_DIMM_MMAP(d2)		(((d2) >> 6) & 0x3)
#define ALI_DIMM_SIZE(d1, d2)		(((((d2) & 0xf) << 8) | (d1)) + 1)

static int
ali_1541_dimm_info(struct dimm_slot *dimm)
{
	int row;
	uint8_t dbc1, dbc2;

	if (!sys_raminfo || !sys_raminfo->dev || !dimm)
		return -ENOSYS;

	dimm->size = 0;
	dimm->ecc  = 0;

	/* read two rows per dimm (for double-side) */
	for (row=2*dimm->num; row<=(2*dimm->num + 1); row++) {
		pci_read_config_byte(sys_raminfo->dev,
				     ALI_DRAM_CONF_2(row), &dbc2);

		/* row is empty iff dimm type and ma_map are both 0 */
		if (!ALI_DIMM_TYPE(dbc2) && !ALI_DIMM_MMAP(dbc2))
			continue;

		pci_read_config_byte(sys_raminfo->dev,
				     ALI_DRAM_CONF_1(row), &dbc1);

		/* type is bits 4-5 of dimm conf reg 2 */
		dimm->type = ALI_DIMM_TYPE(dbc2);

		/* A27-A20 address lines are bits 7-0 of dimm conf reg 1
		 * A31-A28 address lines are bits 3-0 of dimm conf reg 2 */
		dimm->size = ALI_DIMM_SIZE(dbc1, dbc2);
	}

	/* the M1541 uses "not less than" policy to determine which row a
	 * memory address resides in.  the top address boundary for each
	 * row is the maximum memory value minus 1.  so to determine the 
	 * size of a row you must subtract the size of the previous row.
	 * (unless this is slot 0 or the first populated slot) */
	if (dimm->num > 0 && dimm->size > 0) {
		uint16_t sz;
		pci_read_config_byte(sys_raminfo->dev,
				     ALI_DRAM_CONF_1(2*dimm->num - 1), &dbc1);
		pci_read_config_byte(sys_raminfo->dev,
				     ALI_DRAM_CONF_2(2*dimm->num - 1), &dbc2);
		sz = ALI_DIMM_SIZE(dbc1, dbc2);
		dimm->size -= (sz > 1) ? sz : 0;
	}
	
	return 0;
}

int __init
cobalt_raminfo_init(void)
{
	int j;

	/* determine system type and find memory controller pci dev
	 * so we don't have to do pci lookup for each proc read */
	if (cobt_is_3k()) {
		sys_raminfo = &gen3_raminfo;
		sys_raminfo->dev = pci_find_device(PCI_VENDOR_ID_AL,
				   PCI_DEVICE_ID_AL_M1541, NULL);
	} else if (cobt_is_5k()) {
		if (cobt_is_monterey()) {
			sys_raminfo = &gen5_monterey_raminfo;
		} else if (cobt_is_alpine()) {
			sys_raminfo = &gen5_alpine_raminfo;
		} else {
			EPRINTK("unable to identify gen5 board\n");
			return -ENOSYS;
		}
		sys_raminfo->dev = pci_find_device(PCI_VENDOR_ID_SERVERWORKS,
				   PCI_DEVICE_ID_SERVERWORKS_LE, NULL);
	}

	if (!sys_raminfo || !sys_raminfo->dev) {
		EPRINTK("unable to identify system type\n");
		return -ENOSYS;
	}

#ifdef CONFIG_PROC_FS
	/* add entry to /proc filesytem */
	sys_raminfo->proc = create_proc_entry("raminfo",
			    S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH, proc_cobalt);
	if (!sys_raminfo->proc) {
		EPRINTK("can't create /proc/cobalt/raminfo\n");
		return -ENOENT;
	}
	sys_raminfo->proc->owner = THIS_MODULE;
	sys_raminfo->proc->write_proc = NULL;
	sys_raminfo->proc->read_proc = raminfo_read_proc;
#endif /* CONFIG_PROC_FS */

	/* create arrary of dimm slots to store info */
	sys_raminfo->dimm = kmalloc(
		sys_raminfo->total * sizeof(struct dimm_slot), GFP_ATOMIC);
	if (!sys_raminfo->dimm) {
		EPRINTK("unable to allocate memory\n");
#ifdef CONFIG_PROC_FS
		if (sys_raminfo->proc) {
			remove_proc_entry("raminfo", proc_cobalt);
			sys_raminfo->proc = NULL;
		}
#endif /* CONFIG_PROC_FS */
		return -ENOMEM;
	}

	{
		struct dimm_slot *ds = sys_raminfo->dimm;
		for (j=0; j<sys_raminfo->total; j++, ds++) {
			if (!ds) continue;
			ds->num = j;
			if (sys_raminfo->query(ds) < 0) {
				EPRINTK("unable to read dimm %d\n", j);
				ds->num = -1;
			}
		}
	}

	return 0;
}

static void __exit
cobalt_raminfo_exit(void)
{
#ifdef CONFIG_PROC_FS
	if (sys_raminfo->proc) {
		remove_proc_entry("raminfo", proc_cobalt);
		sys_raminfo->proc = NULL;
	}
#endif /* CONFIG_PROC_FS */

	if (sys_raminfo->dimm) {
		kfree(sys_raminfo->dimm);
		sys_raminfo->dimm = NULL;
	}

	sys_raminfo->dev = NULL;
	sys_raminfo = NULL;
}

#ifdef CONFIG_PROC_FS
static int
raminfo_read_proc(char *buf, char **st, off_t off, int len, int *eof, void *x)
{
	int rlen, i;
	struct dimm_slot *ds;

	if (!sys_raminfo)
		return -ENOSYS;

	MOD_INC_USE_COUNT;

	ds = sys_raminfo->dimm;
	for (rlen=i=0; i<sys_raminfo->total; i++, ds++) {
		if (!ds || ds->num < 0)
			continue;
		rlen += sprintf(buf+rlen, "%d [%s%s]: %u MB\n", i,
				ds->size ? dimm_desc[ds->type] : "Empty",
				ds->size ? ds->ecc ? "+ECC" : "" : "",
				ds->size);
	}

	MOD_DEC_USE_COUNT;

	return cobalt_gen_proc_read(buf, rlen, st, off, len, eof);
}
#endif /* CONFIG_PROC_FS */

module_init(cobalt_raminfo_init);
module_exit(cobalt_raminfo_exit);

MODULE_AUTHOR("Sun Cobalt");
MODULE_DESCRIPTION("DIMM Information");
MODULE_LICENSE("GPL");

#endif /* CONFIG_COBALT_RAMINFO || CONFIG_COBALT_RAMINFO_MODULE */
