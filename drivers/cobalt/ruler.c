/* 
 * cobalt ruler driver 
 * Copyright (c) 2000, Cobalt Networks, Inc.
 * Copyright (c) 2001, Sun Microsystems, Inc.
 * $Id: ruler.c,v 1.23 2002/08/29 00:33:01 uzi Exp $
 *
 * author: asun@cobalt.com, thockin@sun.com
 *
 * This should be SMP safe.  There is one critical piece of data, and thus
 * one lock.  The ruler_lock protects the arrays of channels(hwifs) and
 * busproc function pointers.  These are only ever written in the
 * register/unregister functions but read in several other places.  A
 * read/write lock is appropriate. The global switches and sled_leds are 
 * atomic_t. --TPH
 */

#include <stdarg.h>
#include <stddef.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/config.h>
#include <linux/pci.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/ide.h>
#include <linux/hdreg.h>
#include <linux/notifier.h>
#include <linux/sysctl.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <asm/io.h>

#include <cobalt/cobalt.h>
#include <cobalt/systype.h>
#include <cobalt/i2c.h>
#include <cobalt/acpi.h>
#include <cobalt/led.h>


#define RULER_TIMEOUT		(HZ >> 1)  /* .5s */
#define MAX_COBT_DRIVES		4

/* all of this is for gen V */
static struct timer_list cobalt_ruler_timer;
static rwlock_t ruler_lock = RW_LOCK_UNLOCKED;
static ide_hwif_t *channels[MAX_COBT_DRIVES];
static ide_busproc_t *busprocs[MAX_COBT_DRIVES];
/* NOTE: switches is a bitmask of DETACHED sleds */
static atomic_t switches = ATOMIC_INIT(0);	
static atomic_t sled_leds = ATOMIC_INIT(0);
static int sled_led_map[] = {LED_SLED0, LED_SLED1, LED_SLED2, LED_SLED3};
static int ruler_detect;
static int initialized;

static void ruler_hwif_added(ide_hwif_t *hwif, int idx);

static inline u8
read_switches(void)
{
	u8 state = 0;
	if (cobt_is_monterey()) {
		int tries = 3;

		/* i2c can be busy, and this can read wrong - try a few times */
		while (tries--) {
			state = cobalt_i2c_read_byte(COBALT_I2C_DEV_DRV_SWITCH, 
				0);
			if ((state & 0xf0) != 0xf0) {
				break;
			}
		}
	}

	return state;
}

static inline unsigned int
get_sled_leds(void)
{
	return atomic_read(&sled_leds);
}

/*
 * deal with sled leds: LED on means OK to remove
 * NOTE: all the reset lines are kept high. 
 * NOTE: the reset lines are in the reverse order of the switches. 
 */
static void
set_sled_leds(unsigned int leds)
{
	if (cobt_is_monterey()) {
		unsigned int offed = get_sled_leds();

		offed &= ~leds;
		atomic_set(&sled_leds, leds);
#ifdef CONFIG_COBALT_LED
		cobalt_led_clear_bits_lazy(offed);
		cobalt_led_set_bits_lazy(leds);
#endif
	}
}

/* this must be called with the ruler_lock held for read */
static int
do_busproc(int idx, ide_hwif_t *hwif, int arg)
{
	if (cobt_is_monterey()) {
		/* sed sled LEDs */
		switch (arg) {
			case BUSSTATE_ON:
				set_sled_leds(get_sled_leds() & 
					~sled_led_map[idx]);
				break;
			case BUSSTATE_OFF:
			case BUSSTATE_TRISTATE:
				set_sled_leds(get_sled_leds() | 
					sled_led_map[idx]);
				break;
			default:
				WPRINTK("unknown busproc argument (%d)\n", arg);
		}
	}

	/* do the real work */
	return busprocs[idx](hwif, arg);
}

static void 
ruler_timer_fn(unsigned long data)
{
	if (cobt_is_monterey()) {
		u8 state;
		int i;
		unsigned int now, expected, bit, swcur;

		state = read_switches();
		if ((state & 0xf0) == 0xf0) {
			return;
		}
		swcur = atomic_read(&switches);
	
		state &= 0xf;
		read_lock(&ruler_lock);
		for (i = 0; i < MAX_COBT_DRIVES; i++) {
			bit = 1 << i;
			now = state & bit;
			expected = swcur & bit;
			if (now == expected) {
				/* no changes to worry about */
				continue;
			}

			if (now) {
				/* a freshly detached drive */
				atomic_set(&switches, swcur | bit);
		 		if (channels[i]) {
					printk("disabling ide ruler "
						"channel %d\n", i);
					do_busproc(i, channels[i], 
						BUSSTATE_TRISTATE);
		 		} else {
					WPRINTK("drive detach on bad "
						"channel (%d)\n", i);
				}
				set_sled_leds(get_sled_leds() | 
					sled_led_map[i]);
			} else {
				/* 
				 * do we want to do anything when a re-attach 
				 * is detected?
				 */
			}
		}
		read_unlock(&ruler_lock);
	}
}

#ifdef CONFIG_COBALT_ACPI
static int
ruler_interrupt(cobalt_acpi_evt *evt, void * data)
{
	if (cobt_is_monterey() && ruler_detect) {
		u8 state;

		state = read_switches();
		if ((state & 0xf0) != 0xf0) {
			/* this is protected inside mod_timer */
			mod_timer(&cobalt_ruler_timer, jiffies + RULER_TIMEOUT);
		}
		
		evt->ev_data = state;
		/* empirical: delay enough to debounce */
		udelay(10);
	}
	return 0;
}
#endif /* CONFIG_COBALT_ACPI */

#if defined(CONFIG_COBALT_LED)
/* figure which LEDs to blink */
static unsigned int
ide_led_handler(void *data)
{
	unsigned int leds = 0;

	if (cobt_is_monterey()) {
		int i;
		static int ledmap[MAX_COBT_DRIVES] = { 
			LED_DISK0, LED_DISK1, LED_DISK2, LED_DISK3
		};
		static unsigned long old[MAX_COBT_DRIVES];

		read_lock(&ruler_lock);

		for (i = 0; i < MAX_COBT_DRIVES; i++) {
			if (channels[i] && channels[i]->drives[0].present 
			 && channels[i]->drives[0].service_start != old[i]) {
				leds |= ledmap[i];
				old[i] = channels[i]->drives[0].service_start;
			}
		}

		read_unlock(&ruler_lock);
	}

	return leds;
}
#endif

/* this is essentially an exported function - it is in the hwif structs */
static int
ruler_busproc_fn(ide_hwif_t *hwif, int arg)
{
	int r = 0;

	if (cobt_is_monterey()) {
		int idx;

		read_lock(&ruler_lock);

		for (idx = 0; idx < MAX_COBT_DRIVES; idx++) {
			if (channels[idx] == hwif) {
				break;
			}
		}

		if (idx >= MAX_COBT_DRIVES) {
			/* not a hwif we manage? */
			return 0;
		}

		r = do_busproc(idx, hwif, arg);
	
		read_unlock(&ruler_lock);
	
	}

	return r;
}
	
/* 
 * We try to be VERY explicit here.  Fine for now, may eventually break down.
 */
void 
cobalt_ruler_register(ide_hwif_t *hwif)
{
	if (cobt_is_monterey()) {
		struct pci_dev *dev;
		int idx;
		unsigned long flags;

		if (!hwif) {
			return;
		}

		/* Cobalt rulers only have HPT370 controllers on bus 1 */
		dev = hwif->pci_dev;
		if (!dev)
			return;

		if (dev->vendor != PCI_VENDOR_ID_TTI
		 || dev->device != PCI_DEVICE_ID_TTI_HPT366
		 || dev->bus->number != 1) {
			/* ignore it */
			return;
		}

		/* IDE ruler has controllers at dev 3 and 4, ONLY */
		if (dev->devfn == PCI_DEVFN(3,0)) {
			idx = hwif->channel;
		} else if (dev->devfn == PCI_DEVFN(4,0)) {
			idx = 2 + hwif->channel;
		} else {
			return;
		}

		if (idx >= MAX_COBT_DRIVES) {
			return;
		}

		write_lock_irqsave(&ruler_lock, flags);

		/* save a pointer to the hwif, and trap it's busproc() */
		channels[idx] = hwif;
		if (hwif->busproc) {
			busprocs[idx] = hwif->busproc;
			hwif->busproc = ruler_busproc_fn;
		}

		write_unlock_irqrestore(&ruler_lock, flags);

		/* now that we have trapped it, do what we need to initialize 
		 * the drive - if we haven't been initialized, we'll call this
		 * later. 
		 */
		if (initialized) {
			ruler_hwif_added(hwif, idx);
		}
	}
}

static void
ruler_hwif_added(ide_hwif_t *hwif, int idx)
{
	/* the associated switch should be closed */
	if (hwif->drives[0].present) {
		/* set the sled LED off - not safe to remove */
		set_sled_leds(get_sled_leds() & ~sled_led_map[idx]);
	}
}

void 
cobalt_ruler_unregister(ide_hwif_t *hwif)
{
	if (cobt_is_monterey()) {
		int i;
		unsigned long flags;

		write_lock_irqsave(&ruler_lock, flags);

		for (i = 0; i < MAX_COBT_DRIVES; i++) {
			if (channels[i] == hwif) {
				channels[i] = NULL;
				hwif->busproc = busprocs[i];
				busprocs[i] = NULL;
			}
		}

		write_unlock_irqrestore(&ruler_lock, flags);
	}
}

int __init 
cobalt_ruler_init(void)
{
	if (cobt_is_monterey()) {
		int err;
		u8 tmp;
		int i;

		/* initialize switches */
		tmp = read_switches();
		ruler_detect = ((tmp & 0xf0) == 0xf0) ? 0 : 1;
		tmp &= 0xf;
		atomic_set(&switches, tmp);

		/* initialize our timer */
		init_timer(&cobalt_ruler_timer);
		cobalt_ruler_timer.function = ruler_timer_fn;

#ifdef CONFIG_COBALT_ACPI
		err = cobalt_acpi_register_evt_handler(ruler_interrupt, 
			COBALT_ACPI_EVT_SLED, NULL );
		
		if (err) {
			EPRINTK("can't register interrupt handler %p\n", 
				ruler_interrupt);
		}
#endif

		/* set initial sled LED state */
		set_sled_leds(LED_SLED0 | LED_SLED1 | LED_SLED2 | LED_SLED3);

		/* run through any devices that were registered before */
		for (i = 0; i < MAX_COBT_DRIVES; i++) {
			if (channels[i]) {
				ruler_hwif_added(channels[i], i);
			}
		}
		
#if defined(CONFIG_COBALT_LED)
		/* register for a blinky LEDs callback */
		err = cobalt_fpled_register(ide_led_handler, NULL);
		if (err) {
			EPRINTK("can't register LED handler %p\n", 
				ide_led_handler);
		}
#endif
	}

	initialized = 1;

	return 0;
}
