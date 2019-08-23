/* $Id: init.c,v 1.22 2002/11/04 17:54:15 thockin Exp $ */
/*
 *   Copyright (c) 2001  Sun Microsystems
 *   Generic initialization, to reduce pollution of other files
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/stddef.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <cobalt/cobalt.h>

static int cobalt_proc_init(void);
extern int cobalt_i2c_init(void);
extern int cobalt_net_init(void);
extern int cobalt_systype_init(void);
extern void cobalt_boardrev_init(void);
extern int cobalt_led_init(void);
extern int cobalt_lcd_init(void);
extern int cobalt_serialnum_init(void);
extern int cobalt_wdt_init(void);
extern int cobalt_sensors_init(void);
extern int cobalt_fan_init(void);
extern int cobalt_acpi_init(void);
extern int cobalt_ruler_init(void);

#ifdef CONFIG_PROC_FS
struct proc_dir_entry *proc_cobalt;
EXPORT_SYMBOL(proc_cobalt);
#endif
spinlock_t cobalt_superio_lock = SPIN_LOCK_UNLOCKED;

/* initialize all the cobalt specific stuff */
int __init
cobalt_init(void)
{
	cobalt_proc_init();
	cobalt_systype_init();
#ifdef CONFIG_COBALT_RAQ
	/* we might keep the boardrev on an i2c chip */
	cobalt_i2c_init();
#endif
	cobalt_boardrev_init();
#ifdef CONFIG_COBALT_ACPI
	cobalt_acpi_init();
#endif
#ifdef CONFIG_COBALT_LED
	cobalt_net_init();
	cobalt_led_init();
#endif
#ifdef CONFIG_COBALT_LCD
	cobalt_lcd_init();
#endif
#ifdef CONFIG_COBALT_RULER
	cobalt_ruler_init();
#endif
#ifdef CONFIG_COBALT_SERNUM
	cobalt_serialnum_init();
#endif
#ifdef CONFIG_COBALT_RAQ
	/* some systems use WDT it for reboot */
	cobalt_wdt_init();
#endif
#ifdef CONFIG_COBALT_SENSORS
	cobalt_sensors_init();
#endif
#ifdef CONFIG_COBALT_FANS
	cobalt_fan_init();
#endif

	return 0;
}

static int __init
cobalt_proc_init(void)
{
#ifdef CONFIG_PROC_FS
	proc_cobalt = proc_mkdir("cobalt", 0);
	if (!proc_cobalt) {
		EPRINTK("can't create /proc/cobalt\n");
		return -1;
	}
#endif

	return 0;
}

/* a function that handles the blah stuff in a simple proc read function */
int
cobalt_gen_proc_read(char *buf, int plen, char **start, off_t pos, 
	int len, int *eof)
{
	/* trying to read a bad offset? */
	if (pos >= plen) {
		*eof = 1;
		return 0;
	}

	/* did we write everything we wanted to? */
	if (len >= (plen-pos)) {
		*eof = 1;
	}

	*start = buf + pos;
	plen -= pos;

	return (len > plen) ? plen : len;
}
EXPORT_SYMBOL(cobalt_gen_proc_read);
