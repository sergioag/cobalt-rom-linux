/* $Id: wdt.c,v 1.21 2002/07/02 00:38:17 asun Exp $ */
/* 
 * Cobalt kernel WDT timer driver
 * Tim Hockin <thockin@cobaltnet.com>
 * Adrian Sun <asun@cobalt.com>
 * Chris Johnson <cjohnson@cobalt.com>
 * Copyright (c)1999-2000, Cobalt Networks
 * Copyright (c)2001, Sun Microsystems
 *
 * This should be SMP safe.  Every external function (except trigger_reboot)
 * grabs the wdt lock.  No function in this file may call any exported
 * function (excepting trigger_reboot).  The disable counter is an atomic, so
 * there should be no issues there. --TPH
 */
#include <linux/config.h>

#include <linux/module.h>
#include <stdarg.h>
#include <stddef.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/config.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/msr.h>

#include <cobalt/cobalt.h>
#include <cobalt/systype.h>
#include <cobalt/wdt.h>
#include <cobalt/superio.h>

#define DOGB			0x20
#define ALI_7101_WDT		0x92
#define ALI_WDT_ARM		0x01
#define WDT_3K_TIMEOUT 		(HZ >> 4)	/* 1/16 second */

#define WDT_SUPERIO_TIMEOUT	(0x01)		/* 1 minute */
#define WDT_5K_TIMEOUT 		(HZ << 3)	/* 8 seconds */

static unsigned long wdt_timeout;
static unsigned long long tsc_per_wdt;
static int initialized;

#ifdef CONFIG_COBALT_WDT
struct timer_list cobalt_wdt_timer;
static atomic_t cobalt_wdt_disable_count = ATOMIC_INIT(0);
static spinlock_t wdt_lock = SPIN_LOCK_UNLOCKED;
#endif

/* gen III */
static struct pci_dev *cobalt_pmu;
static int use_pic;
/* gen V */
static u16 superio_pm_port;

#ifdef CONFIG_COBALT_WDT
static void do_refresh(void);
static void do_cleardog(void);
static void do_disable(void);
static void do_reenable(void);
#endif
	
static unsigned long __init 
chipset_setup(void)
{
	if (cobt_is_3k()) {
		/* 
		 * Set up the PMU for 3k boards. It has a max
		 * of a 1 second timeout. 
		 */
		struct pci_dev *south;
		char tmp;

		/* PMU (1543 ver A1-E) has a built-in WDT.  Set it to 1 sec */
		cobalt_pmu = pci_find_device(PCI_VENDOR_ID_AL, 
			PCI_DEVICE_ID_AL_M7101, NULL);
		if (!cobalt_pmu) {
			EPRINTK("can't find south bridge for WDT\n");
			return 0;
		}
		pci_write_config_byte(cobalt_pmu, ALI_7101_WDT, 0x02);
	
		/* why it is called 1543, but DevId is 1533 I'll never know */
		south = pci_find_device(PCI_VENDOR_ID_AL, 
			PCI_DEVICE_ID_AL_M1533, NULL);
		if (!south) {
			EPRINTK("can't find south bridge for WDT\n");
			use_pic = 1;
		} else {
			/* reversion # is here - must match ???1001?(b)
			 * else use PIC for WDT */
			pci_read_config_byte(south, 0x5e, &tmp);
			use_pic = ((tmp & 0x1e) != 0x12);
		}

		if (!use_pic) {
			/* set DOGB GPIO pin to OUTPUT - JIC */
			pci_read_config_byte(cobalt_pmu, 0x7d, &tmp);
			pci_write_config_byte(cobalt_pmu, 0x7d, tmp | DOGB);
		}
		return WDT_3K_TIMEOUT;
	} else if (cobt_is_monterey()) {
		/* 
		 * Set up the Nat. Semi SuperI/O for XTR. It has a 
		 * minimum of a 1 minute timeout. 
		 */
	
		/* superi/o -- select pm logical device and get base address */
		superio_pm_port = superio_ldev_base(PC87317_DEV_PM);
#ifdef CONFIG_COBALT_WDT
		if (!superio_pm_port) {
			return 0;
		}
		outb(PC87317_PMDEV_WDTO, superio_pm_port);
		outb(WDT_SUPERIO_TIMEOUT, superio_pm_port + 1); 
#endif
		return WDT_5K_TIMEOUT;
	} else if (cobt_is_alpine()) {
		/* 
		 * Set up the Nat. Semi SuperI/O for Alpine. It has a 
		 * minimum of a 1 minute timeout. 
		 */
		unsigned char tmp;
	
		/* superi/o -- select pm logical device and get base address */
		superio_pm_port = superio_ldev_base(PC87417_DEV_SWC);
#ifdef CONFIG_COBALT_WDT
		if (!superio_pm_port) {
			return 0;
		}
		/* select the WDT bank of SWC */
		outb(PC87417_SWCBANK_WDT, superio_pm_port+PC87417_SWC_BANK);
		/* clear event config... */
		tmp = inb(superio_pm_port+PC87417_WDT_CONFIG);
		outb(0, superio_pm_port+PC87417_WDT_CONFIG);
		/* ...before mucking with timeout */
		outb(WDT_SUPERIO_TIMEOUT, 
			superio_pm_port+PC87417_WDT_TIMEOUT); 
		/* restore the event config */
		outb(tmp, superio_pm_port+PC87417_WDT_CONFIG);
		/* enable the counter */
		outb(1, superio_pm_port+PC87417_WDT_CONTROL);
#endif
		return WDT_5K_TIMEOUT;
	}

	return 0;
}

void __init 
cobalt_wdt_init(void)
{
	unsigned long long start, stop;

	wdt_timeout = chipset_setup();

	/* figure out time */
	rdtscll(start);
	udelay(100);
	rdtscll(stop);

	/*
	 * (int) (stop - start) * 10 == tsc per msec
	 * 1000 / HZ == msec per tick
	 * wdt_timeout == ticks per watchdog rearm
	 */
	tsc_per_wdt = (int) (stop - start) * 10 * (1000 * wdt_timeout / HZ);

#ifdef CONFIG_COBALT_WDT
	/* set the timer going */
	init_timer(&cobalt_wdt_timer);
	cobalt_wdt_timer.function = cobalt_wdt_refresh;
	cobalt_wdt_timer.data = 1;
	cobalt_wdt_timer.expires = jiffies + wdt_timeout;
	add_timer(&cobalt_wdt_timer);

	/* the first timer tick will set it going */

	if (cobt_is_3k() && use_pic) {
		WPRINTK("Cobalt WDT - old board, using PIC controller\n");
	}
#endif /* CONFIG_COBALT_WDT */

	initialized = 1;
}

static inline void
hw_disable(void)
{
	if (cobt_is_3k()) {
		char tmp;
		/* read the value, disable (reset) WDT */
		pci_read_config_byte(cobalt_pmu, ALI_7101_WDT, &tmp);
		pci_write_config_byte(cobalt_pmu, ALI_7101_WDT, 
			(tmp & ~ALI_WDT_ARM));
	} else if (cobt_is_monterey()) {
		outb(PC87317_PMDEV_WDTO, superio_pm_port);
		outb(0, superio_pm_port + 1);
	} else if (cobt_is_alpine()) {
		unsigned char tmp;
		/* select the WDT bank of SWC */
		outb(PC87417_SWCBANK_WDT, superio_pm_port + PC87417_SWC_BANK);
		/* clear event config before mucking with timeout */
		tmp = inb(superio_pm_port + PC87417_WDT_CONFIG);
		outb(0, superio_pm_port + PC87417_WDT_CONFIG);
		/* 
		 * Disable it by setting a 0 time-out.
		 * The spec says 00h is reserved, but NSC confirms this is the
		 * way to disable the device.
		 */
		outb(0, superio_pm_port + PC87417_WDT_TIMEOUT);
		/* restore the event config */
		outb(tmp, superio_pm_port + PC87417_WDT_CONFIG);
	}
}

static inline void
hw_enable(void)
{
	if (cobt_is_3k()) {
		unsigned char tmp;
		/* read the value, disable (reset) WDT, enable WDT */
		pci_read_config_byte(cobalt_pmu, ALI_7101_WDT, &tmp);
		pci_write_config_byte(cobalt_pmu, ALI_7101_WDT, 
			(tmp | ALI_WDT_ARM));
		if (use_pic) {
			/* transition GPIO 5 (DOGB) to arm/clear timer */
			pci_read_config_byte(cobalt_pmu, 0x7e, &tmp);
			pci_write_config_byte(cobalt_pmu, 0x7e, tmp ^ DOGB);
		}
	} else if (cobt_is_monterey()) {
		outb(PC87317_PMDEV_WDTO, superio_pm_port);
		outb(WDT_SUPERIO_TIMEOUT, superio_pm_port + 1);
	} else if (cobt_is_alpine()) {
		unsigned char tmp;
		/* select the WDT bank of SWC */
		outb(PC87417_SWCBANK_WDT, superio_pm_port + PC87417_SWC_BANK);
		/* clear event config before mucking with timeout */
		tmp = inb(superio_pm_port + PC87417_WDT_CONFIG);
		outb(0, superio_pm_port + PC87417_WDT_CONFIG);
		/* enable and refresh the timer */
		outb(WDT_SUPERIO_TIMEOUT, 
			superio_pm_port + PC87417_WDT_TIMEOUT);
		outb(0x80, superio_pm_port + PC87417_WDT_CONTROL);
		/* restore event config */
		outb(tmp, superio_pm_port + PC87417_WDT_CONFIG);
	}
}

#ifdef CONFIG_COBALT_WDT
static void
do_refresh(void)
{
	if (!initialized) {
		return;
	}

	do_cleardog();
	
	/* re-arm the timer - this is locked in mod_timer() */
	mod_timer(&cobalt_wdt_timer, jiffies + wdt_timeout);
}
#endif

EXPORT_SYMBOL(cobalt_wdt_refresh);
void 
cobalt_wdt_refresh(unsigned long refresh_timer)
{
#ifdef CONFIG_COBALT_WDT
	unsigned long flags;
	spin_lock_irqsave(&wdt_lock, flags);
	do_refresh();
	spin_unlock_irqrestore(&wdt_lock, flags);
#endif
}

#ifdef CONFIG_COBALT_WDT
static void
do_cleardog(void)
{
	static unsigned long long last_tsc = 0;
	unsigned long long tmp;

	if (!initialized || (atomic_read(&cobalt_wdt_disable_count) > 0)) {
		return;
	}

	/* only bother if we're due */
	rdtscll(tmp);
	if ((int)(tmp - last_tsc) < tsc_per_wdt) {
		return;
	}

	if (cobt_is_3k() || cobt_is_monterey()) {
		/* this is how we re-start the clock */
		hw_disable();
		hw_enable();
	} else if (cobt_is_alpine()) {
		/* select the WDT bank of SWC */
		outb(PC87417_SWCBANK_WDT, superio_pm_port + PC87417_SWC_BANK);
		/* refresh the timer */
		outb(0x80, superio_pm_port + PC87417_WDT_CONTROL);
	}

	rdtscll(last_tsc);
}
#endif

EXPORT_SYMBOL(cobalt_wdt_cleardog);
void 
cobalt_wdt_cleardog(void)
{
#ifdef CONFIG_COBALT_WDT
	unsigned long flags;

	spin_lock_irqsave(&wdt_lock, flags);
	do_cleardog();
	spin_unlock_irqrestore(&wdt_lock, flags);
#endif
}

/* 
 * this is called from machine_restart. it should not be used on
 * 5k machines. 
 */
EXPORT_SYMBOL(cobalt_wdt_trigger_reboot);
void 
cobalt_wdt_trigger_reboot(void)
{
	if (cobt_is_3k()) {
		if (!cobalt_pmu) {
			WPRINTK("no PMU found!\n");
			WPRINTK("reboot not possible!\n");
			return;
		}

#ifdef CONFIG_COBALT_WDT
		/* stop feeding it */
		del_timer_sync(&cobalt_wdt_timer);
#endif

		/* kiss your rear goodbye... */
		initialized = 0;
		hw_disable();
		hw_enable();
	}
}

#ifdef CONFIG_COBALT_WDT
static void
do_disable(void)
{
	if (!initialized) {
		return;
	}

	if (atomic_read(&cobalt_wdt_disable_count) == 0) {
		atomic_inc(&cobalt_wdt_disable_count);
		del_timer_sync(&cobalt_wdt_timer);
		hw_disable();
	}
}
#endif

EXPORT_SYMBOL(cobalt_wdt_disable);
void 
cobalt_wdt_disable(void)
{
#ifdef CONFIG_COBALT_WDT
	unsigned long flags;

	if (cobt_is_3k() && use_pic) {
		WPRINTK("in PIC mode - cannot disable\n");
		return;
	}

	spin_lock_irqsave(&wdt_lock, flags);
	do_disable();
	spin_unlock_irqrestore(&wdt_lock, flags);
#endif
}

#ifdef CONFIG_COBALT_WDT
static void
do_reenable(void)
{
	int dcnt;

	if (!initialized) { 
		return;
	}

	atomic_dec(&cobalt_wdt_disable_count);
	dcnt = atomic_read(&cobalt_wdt_disable_count);

	if (dcnt == 0) {
		do_refresh();
	} else if (dcnt < 0) {
		WPRINTK("too many enables\n");
		atomic_set(&cobalt_wdt_disable_count, 0);
	}
}
#endif


EXPORT_SYMBOL(cobalt_wdt_reenable);
void 
cobalt_wdt_reenable(void)
{
#ifdef CONFIG_COBALT_WDT
	unsigned long flags;

	spin_lock_irqsave(&wdt_lock, flags);
	do_reenable();
	spin_unlock_irqrestore(&wdt_lock, flags);
#endif
}
