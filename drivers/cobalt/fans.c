/* $Id: fans.c,v 1.18 2002/03/16 21:33:02 duncan Exp $
 * Copyright (c) 2000-2001 Sun Microsystems, Inc 
 *
 * This should be SMP safe.  The critical data (the info list) and the
 * critical code (inb()/outb() calls) are protected by fan_lock.  It is 
 * locked at the only external access points - the proc read()/write() 
 * methods. --TPH
 */
#include <linux/config.h>
#if defined(CONFIG_COBALT_FANS) || defined(CONFIG_COBALT_FANS_MODULE)

#include <stdarg.h>
#include <stddef.h>

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/time.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include <cobalt/cobalt.h>
#include <cobalt/systype.h>

/* GPIO base is assigned by BIOS, perhaps we should probe it */
#define GPIO_BASE		0x600
#define FAN_GPIO_MAX		8
#define FAN_RPM(fn,ms)		((fn).hcyl * (60000000 / (fn).poles) / (ms))
#define FAN_VALID(f)		((f)->mask && (f)->poles)
#define FAN_CACHE_TIME		2 /* seconds */
#define FAN_SAMPLE_LEN		50 /* milliseconds */

/* 
 * fans are attached to GPIO pins
 * each pin is part of a port, multiple fans are controlled by a port
 */
struct fan_info {
	int id;			/* fan number */
	uint8_t mask;		/* mask within the port */
	int poles;		/* # of magnetic poles (divisor) */
	int hcyl;		/* # of half cycles */
	unsigned rpm;		/* calculated fan speed */
	char *type;		/* FAN description */
};

struct fan_gpio {
	int port;		/* GPIO Port */
	uint16_t base;		/* GPDI (data in) base address */
	uint8_t latch;		/* latched 'data in' value */
	long tcache;		/* latched 'epoch' value */
	struct fan_info fan[FAN_GPIO_MAX];
};

/* the current fanlist */
static struct fan_gpio *sys_fanlist;
static spinlock_t fan_lock = SPIN_LOCK_UNLOCKED;

static struct fan_gpio fan_gpio_raqxtr[] = {
	{
		port: 1,
		base: GPIO_BASE,
		fan: {
			{ mask: 0x2, poles: 4, type: "processor" },
			{ mask: 0x4, poles: 4, type: "processor" },
			{ mask: 0 },
		},
	},
	{
		port: 2,
		base: GPIO_BASE+4,
		fan: {
			{ mask: 0x10, poles: 4 },
			{ mask: 0x20, poles: 4 },
			{ mask: 0x40, poles: 4 },
			{ mask: 0x80, poles: 4 },
			{ mask: 0 },
		},
	},
	{ port: -1 }
};

static struct fan_gpio fan_gpio_alpine[] = {
	{
		port: 2,
		base: GPIO_BASE+7,
		fan:  {
			{ mask: 0x4, poles: 4 },
			{ mask: 0x8, poles: 4 },
			{ mask: 0x10, poles: 4 },
			{ mask: 0x20, poles: 4, type: "power supply" },
			{ mask: 0x40, poles: 4, type: "processor" },
			{ mask: 0 },
		},
	},
	{ port: -1 }
};

#ifdef CONFIG_PROC_FS
#ifdef CONFIG_COBALT_OLDPROC
static struct proc_dir_entry *proc_faninfo;
#endif /* CONFIG_COBALT_OLDPROC */
static struct proc_dir_entry *proc_cfaninfo;
#endif /* CONFIG_PROC_FS */

static struct fan_info *fan_info_find(int id);
static int fan_control(struct fan_info *fi, int todo);
static int fan_info_print(char *buffer);
static int fan_read_proc(char *buf, char **start, off_t pos,
			 int len, int *eof, void *x);
static int fan_write_proc(struct file *file, const char *buf,
			  unsigned long len, void *x);

int __init 
cobalt_fan_init(void)
{
	if (cobt_is_monterey()) {
		sys_fanlist = (struct fan_gpio *)fan_gpio_raqxtr;
	} else if (cobt_is_alpine()) {
		sys_fanlist = (struct fan_gpio *)fan_gpio_alpine;
	} else {
		sys_fanlist = NULL;
		return -ENOSYS;
	}

#ifdef CONFIG_PROC_FS
#ifdef CONFIG_COBALT_OLDPROC
	proc_faninfo = create_proc_entry("faninfo", S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH, NULL);
	if (!proc_faninfo) {
		EPRINTK("can't create /proc/faninfo\n");
		return -ENOENT;
	}
	proc_faninfo->owner = THIS_MODULE;
	proc_faninfo->read_proc = fan_read_proc;
	proc_faninfo->write_proc = fan_write_proc;
#endif /* CONFIG_COBALT_OLDPROC */
	proc_cfaninfo = create_proc_entry("faninfo", S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH, proc_cobalt);
	if (!proc_cfaninfo) {
		EPRINTK("can't create /proc/cobalt/faninfo\n");
		return -ENOENT;
	}
	proc_cfaninfo->owner = THIS_MODULE;
	proc_cfaninfo->read_proc = fan_read_proc;
	proc_cfaninfo->write_proc = fan_write_proc;
#endif /* CONFIG_PROC_FS */

	return 0;
}

static void __exit
cobalt_fan_exit(void)
{
#ifdef CONFIG_PROC_FS
#ifdef CONFIG_COBALT_OLDPROC
	if (proc_faninfo) {
		remove_proc_entry("faninfo", NULL);
	}
#endif /* CONFIG_COBALT_OLDPROC */
	if (proc_cfaninfo) {
		remove_proc_entry("faninfo", proc_cobalt);
	}
#endif /* CONFIG_PROC_FS */

	sys_fanlist = NULL;
}

/*
 * Samples fan tachometer square wave to calculate and report RPM
 */
static int 
get_faninfo(char *buffer)
{
	struct fan_gpio *fg;
	struct timeval utime;
	unsigned long elapsed, start;
	int i, val, len;

	if (!sys_fanlist || !cobt_is_5k()) {
		/* software is keyed off this string - do not change it ! */
		return sprintf(buffer, "Fan monitoring not supported.\n");
	}

	/* save start timestamp */
	do_gettimeofday(&utime);
	start = utime.tv_usec;

	/* initialize 'previous' values. we do edge detection by
	 * looking for transitions from previous values */
	for (fg = sys_fanlist; fg->port >= 0; fg++) {
		if (fg->tcache && utime.tv_sec < fg->tcache+FAN_CACHE_TIME) {
			return fan_info_print(buffer);
		}
		fg->tcache = utime.tv_sec;
		fg->latch = inb(fg->base);
		for (i = 0; i < FAN_GPIO_MAX; i++) {
			fg->fan[i].hcyl = 0;
			fg->fan[i].rpm = 0;
		}
	}

	/* We are counting the number of halfcycles in a square wave
	 * that pass in a given amount of time to determine frequency */
	do {
		for (fg=sys_fanlist; fg->port>=0; fg++) {
			val = inb(fg->base);
			for (i=0; i<FAN_GPIO_MAX; i++) {
				struct fan_info *p = &fg->fan[i];
				if (FAN_VALID(p)) {
					if ((val ^ fg->latch) & p->mask) {
						p->hcyl++;
					}
				}
			}
			fg->latch = val;
		}

		do_gettimeofday(&utime);
		if (utime.tv_usec > start) {
			elapsed = utime.tv_usec - start;
		} else {
			elapsed = utime.tv_usec + 1000001 - start;
		}

	} while (elapsed < (FAN_SAMPLE_LEN) * 1000);

	/* Fan rpm = 60 / ( t * poles )
	 *  where t is 1/2 the period and poles are the number of 
	 *  magnetic poles for the fan.
	 *  
	 * For the Sunon KDE1204PKBX fans on Raq XTR, poles = 4
	 * So, in terms of cycles,
	 *
	 *  rpm = 60 s/m    halfcycles       
	 *        ------ *  -------------- * 1,000,000 us/s * 2
	 *        4         2 * elapsed us
	 *
	 *      = (60,000,000 / 4 poles) * halfcycles / elapsed
	 *      = 15,000,000 * halfcycles / elapsed
	 *
	 * Note, by this method and sampling for 50ms, our accuracy
	 *  is +/- 300 rpm.  The fans are spec'ed for +/- 1000 rpm 
	 */
	for (val=len=0, fg=sys_fanlist; fg->port>=0; fg++) {
		for (i=0; i<FAN_GPIO_MAX; i++) {
			struct fan_info *p = &fg->fan[i];
			if (FAN_VALID(p)) {
				p->id = val++;
				p->rpm = FAN_RPM(fg->fan[i], elapsed);
				len += sprintf(buffer+len, "fan %d     : %u\n",
					p->id, p->rpm);
			}
		}
	}

	return len;
}

static int
fan_info_print(char *buffer)
{
	struct fan_gpio *fg;
	int i, len=0;

	if (!sys_fanlist) {
		return -1;
	}

	for (fg=sys_fanlist; fg->port>=0; fg++) {
		for (i=0; i<FAN_GPIO_MAX; i++) {
			struct fan_info *p = &fg->fan[i];
			if (FAN_VALID(p)) {
				len += sprintf(buffer+len, "fan %d     : %u\n",
					p->id, p->rpm);
			}
		}
	}

	return len;
}

/* FIXME: generify */
static int
fan_control(struct fan_info *fi, int todo)
{
	if (fi && cobt_is_alpine()) {
		switch (fi->id) {
		case 4:	{
			/* CPU FAN */
			uint8_t gpdo = inb(GPIO_BASE+6);

			if (todo) {
				gpdo &= ~fi->mask; /* 0 = on */
			} else {
				gpdo |= fi->mask;  /* 1 = off */
			}
			outb(gpdo, GPIO_BASE+6);
			return 0;
		}
		default:
			return -ENODEV;
		}
	}

	return -ENOSYS;
}

static struct fan_info *
fan_info_find(int id)
{
	struct fan_gpio *fg;
	int i;

	if (!sys_fanlist) {
		return NULL;
	}

	for (fg=sys_fanlist; fg->port>=0; fg++) {
		for (i=0; i<FAN_GPIO_MAX; i++) {
			if (FAN_VALID(&fg->fan[i])) {
				if (fg->fan[i].id == id) {
					return &fg->fan[i];
				}
			}
		}
	}

	return NULL;
}

#ifdef CONFIG_PROC_FS
static int 
fan_read_proc(char *buf, char **start, off_t pos, int len, int *eof, void *x)
{
	int plen;

	MOD_INC_USE_COUNT;

	spin_lock(&fan_lock);
	plen = get_faninfo(buf);
	spin_unlock(&fan_lock);

	MOD_DEC_USE_COUNT;

	return cobalt_gen_proc_read(buf, plen, start, pos, len, eof);
}

static int
fan_write_proc(struct file *file, const char *buf, unsigned long len, void *x)
{
	char *page;
	int retval = -EINVAL;

	MOD_INC_USE_COUNT;

	if (len > PAGE_SIZE) {
		MOD_DEC_USE_COUNT;
		return -EOVERFLOW;
	}

	page = (char *)__get_free_page(GFP_KERNEL);
	if (!page) {
		MOD_DEC_USE_COUNT;
		return -ENOMEM;
	}

	if (copy_from_user(page, buf, len)) {
		free_page((unsigned long)page);
		MOD_DEC_USE_COUNT;
		return -EFAULT;
	}
	page[len] = '\0';

	/* format: `fan ID COMMAND' */
	if (len>5 && !strncmp("fan ", page, 4)) {
		if (*(page+4) != '\0') {
			struct fan_info *finf;
			char *nextpg = NULL;

			spin_lock(&fan_lock);
			finf = fan_info_find(simple_strtoul(page+4,&nextpg,0));
			if (!finf) {
				retval = -ENOENT;
			} else if (nextpg != '\0') {
				if (!strncmp("on", nextpg+1, 2)) {
					retval = fan_control(finf, 1);
				}
				else if (!strncmp("off", nextpg+1, 3)) {
					retval = fan_control(finf, 0);
				}
			}
			spin_unlock(&fan_lock);
		}
	}

	free_page((unsigned long)page);
	MOD_DEC_USE_COUNT;
	
	return (retval < 0) ? retval : len;
}
#endif /* CONFIG_PROC_FS */

#if defined(CONFIG_COBALT_FANS_MODULE)
module_init(cobalt_fan_init);
module_exit(cobalt_fan_exit);

MODULE_AUTHOR("Sun Cobalt");
MODULE_DESCRIPTION("Sun Cobalt fan tachometers");
#endif

#endif /* CONFIG_COBALT_FANS || CONFIG_COBALT_FANS_MODULE */
