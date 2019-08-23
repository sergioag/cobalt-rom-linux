/*
 * $Id: led.c,v 1.36 2002/05/10 18:44:45 duncan Exp $
 * led.c : driver for Cobalt LEDs
 *
 * Copyright 1996-2000 Cobalt Networks, Inc.
 * Copyright 2001 Sun Microsystems, Inc.
 *
 * By:	Andrew Bose
 *	Timothy Stonis
 *	Tim Hockin
 *	Adrian Sun
 *	Duncan Laurie
 *
 * This should be SMP safe.  There is one definite critical region: the
 * handler list (led_handler_lock).  The led_state is protected by led_lock, 
 * so should be safe against simultaneous writes.  Bit banging of lights is 
 * currently also a protected region (led_lock, rather than add a new lock).
 */

#include <linux/config.h>

#ifdef CONFIG_COBALT_LED

#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/nvram.h>
#include <asm/io.h>
#include <linux/delay.h>

#include <cobalt/cobalt.h>
#include <cobalt/systype.h>
#include <cobalt/led.h>
#include <cobalt/i2c.h>
#include <cobalt/superio.h>

#define LED_DRIVER			"Cobalt Networks LED driver"
#define LED_DRIVER_VMAJ		1
#define LED_DRIVER_VMIN		0

/* the rate at which software controlled frontpanel LEDs blink */
#define FPLED_DEFAULT_HZ	(HZ/20)

/* 
 * This is the abstracted state of active LEDs - see the defines for LED_* 
 * LED masks are always 'unsigned int'.  You must hold led_lock to muck with
 * these.
 */
static unsigned int led_state;
static unsigned int led_blips;

/* leds are PCI on genIII */
static struct pci_dev *led_dev;
/* on XTR the front panel LEDs are software controlled */
struct led_handler {
	unsigned int (*function)(void *);
	void *data;
	struct led_handler *next;
	struct led_handler *prev;
};
struct led_handler *led_handler_list;
static spinlock_t led_handler_lock = SPIN_LOCK_UNLOCKED;
static struct timer_list timer;

static spinlock_t led_lock = SPIN_LOCK_UNLOCKED;

/*
 * RaQ 3
 * RaQ 4
 * Qube 3
 */
#define RAQ3_SHUTLOGO_ADDR	0x7e
#define RAQ3_SHUTDOWN_OFF	0x40 /* reverse polarity */
#define RAQ3_COBALTLOGO_ON	0x80
#define QUBE3_LIGHTBAR_ON	0xc0 /* direct polarity */
#define RAQ3_WEBLIGHT_ADDR	0xb8
#define RAQ3_WEBLIGHT_ON	0x80

/*
 * RaQ XTR
 */
#define MONTEREY_FPLED00		0x8000
#define MONTEREY_FPLED01		0x4000
#define MONTEREY_FPLED02		0x2000
#define MONTEREY_FPLED03		0x0200
#define MONTEREY_FPLED04		0x0080
#define MONTEREY_FPLED05		0x0040
#define MONTEREY_FPLED10		0x1000
#define MONTEREY_FPLED11		0x0800
#define MONTEREY_FPLED12		0x0400
#define MONTEREY_FPLED13		0x0100
#define MONTEREY_FPLED14		0x0020
#define MONTEREY_FPLED15		0x0010
#define MONTEREY_FPLED_ETH0_TXRX	MONTEREY_FPLED00
#define MONTEREY_FPLED_ETH0_LINK	MONTEREY_FPLED10
#define MONTEREY_FPLED_ETH1_TXRX	MONTEREY_FPLED01
#define MONTEREY_FPLED_ETH1_LINK	MONTEREY_FPLED11
#define MONTEREY_FPLED_DISK0		MONTEREY_FPLED02
#define MONTEREY_FPLED_DISK1		MONTEREY_FPLED03
#define MONTEREY_FPLED_DISK2		MONTEREY_FPLED04
#define MONTEREY_FPLED_DISK3		MONTEREY_FPLED05
#define MONTEREY_FPLED_WEB 		MONTEREY_FPLED12
#define MONTEREY_LOGOLED_BIT		0x40
#define MONTEREY_SYSFAULTLED_BIT	0x80
#define MONTEREY_SLED0			(1<<3)
#define MONTEREY_SLED1			(1<<2)
#define MONTEREY_SLED2			(1<<1)
#define MONTEREY_SLED3			(1<<0)

/*
 * Alpine
 */
#define ALPINE_WEBLED_PORT		0x60e
#define ALPINE_WEBLED_BIT		0x20
#define ALPINE_POWERLED_PORT		0x50b
#define ALPINE_POWERLED_CFG             0x23
#define ALPINE_LOGOLED_BIT		0x02
#define ALPINE_SYSFAULTLED_BIT		0x07

/* 
 * actually set the leds (icky details hidden within) 
 * this must be protected against itself with led_lock
 * */
static void 
__set_led_hw(const unsigned int newstate)
{
	if (cobt_is_pacifica() && led_dev) {
		unsigned char tmp;
		/* RaQ 3, RaQ 4
		 * - shutdown light
		 * - logo light
		 * - web light
		 */

		/* read the current state of shutdown/logo lights */
		pci_read_config_byte(led_dev, RAQ3_SHUTLOGO_ADDR, &tmp);

		/* reverse polarity for shutdown light */
		if (newstate & LED_SHUTDOWN)
			tmp &= ~RAQ3_SHUTDOWN_OFF;
		else
			tmp |= RAQ3_SHUTDOWN_OFF;

		/* logo light is straight forward */
		if (newstate & LED_COBALTLOGO)
			tmp |= RAQ3_COBALTLOGO_ON;
		else
			tmp &= ~RAQ3_COBALTLOGO_ON;

		/* write new shutdown/logo light state */
		pci_write_config_byte(led_dev, RAQ3_SHUTLOGO_ADDR, tmp);

		/* read web light state */
		pci_read_config_byte(led_dev, RAQ3_WEBLIGHT_ADDR, &tmp);
		if (newstate & LED_WEBLIGHT) {
			tmp |= RAQ3_WEBLIGHT_ON;
		} else {
			tmp &= ~RAQ3_WEBLIGHT_ON;
		}

		/* write new web light state */
		pci_write_config_byte(led_dev, RAQ3_WEBLIGHT_ADDR, tmp);
	} else if (cobt_is_carmel() && led_dev) {
		unsigned char tmp;
		/* Qube 3
		 * - no shutdown light
		 * - lightbar instead of logo
		 * - no web led (wired to 2nd IDE reset for staggered startup)
		 */

		/* read the current state of lightbar */
		pci_read_config_byte(led_dev, RAQ3_SHUTLOGO_ADDR, &tmp);
		if (newstate & LED_COBALTLOGO) {
			tmp |= QUBE3_LIGHTBAR_ON;
		} else {
			tmp &= ~QUBE3_LIGHTBAR_ON;
		}

		/* write new lightbar state */
		pci_write_config_byte(led_dev, RAQ3_SHUTLOGO_ADDR, tmp);
	} else if (cobt_is_monterey()) {
		unsigned int tmp = 0;
		u8 val; 
		unsigned long flags;

		if (newstate & LED_WEBLIGHT) {
			tmp |= MONTEREY_FPLED_WEB;
		}
		if (newstate & LED_ETH0_TXRX) {
			tmp |= MONTEREY_FPLED_ETH0_TXRX;
		}
		if (newstate & LED_ETH0_LINK) {
			tmp |= MONTEREY_FPLED_ETH0_LINK;
		}
		if (newstate & LED_ETH1_TXRX) {
			tmp |= MONTEREY_FPLED_ETH1_TXRX;
		}
		if (newstate & LED_ETH1_LINK) {
			tmp |= MONTEREY_FPLED_ETH1_LINK;
		}
		if (newstate & LED_DISK0) {
			tmp |= MONTEREY_FPLED_DISK0;
		}
		if (newstate & LED_DISK1) {
			tmp |= MONTEREY_FPLED_DISK1;
		}
		if (newstate & LED_DISK2) {
			tmp |= MONTEREY_FPLED_DISK2;
		}
		if (newstate & LED_DISK3) {
			tmp |= MONTEREY_FPLED_DISK3;
		}
		/* 3 LED's are unused on Monterey, but we support them */
		if (newstate & LED_MONTEREY_UNUSED0) {
			tmp |= MONTEREY_FPLED13;
		}
		if (newstate & LED_MONTEREY_UNUSED1) {
			tmp |= MONTEREY_FPLED14;
		}
		if (newstate & LED_MONTEREY_UNUSED2) {
			tmp |= MONTEREY_FPLED15;
		}
		/* I2C controlled front-panel lights */
		cobalt_i2c_write_byte(COBALT_I2C_DEV_LED_I, 0, tmp & 0xff);
		cobalt_i2c_write_byte(COBALT_I2C_DEV_LED_II, 0, tmp >> 8);
		
		/* drive sled LEDs are on a different i2c device */
		tmp = 0xf0; /* high nibble means something else */
		if (newstate * LED_SLED0)
			tmp |= MONTEREY_SLED0;
		if (newstate * LED_SLED1)
			tmp |= MONTEREY_SLED1;
		if (newstate * LED_SLED2)
			tmp |= MONTEREY_SLED2;
		if (newstate * LED_SLED3)
			tmp |= MONTEREY_SLED3;
		cobalt_i2c_write_byte(COBALT_I2C_DEV_RULER, 0, tmp);

		/* sysfault and logo are in APC page of nvram */
		spin_lock_irqsave(&rtc_lock, flags);
		superio_set_rtc_bank(PC87317_RTC_BANK_APC);
		val = CMOS_READ(PC87317_APCR4);

		/* reverse polarity */
		if (newstate & LED_COBALTLOGO) {
			val &= ~MONTEREY_LOGOLED_BIT; /* logo is on */
		} else {
			val |= MONTEREY_LOGOLED_BIT; /* logo is off */
		}

		if (newstate & LED_SYSFAULT) {
			val |= MONTEREY_SYSFAULTLED_BIT;
		} else {
			val &= ~MONTEREY_SYSFAULTLED_BIT;
		}

		CMOS_WRITE(val, PC87317_APCR4);
		superio_set_rtc_bank(PC87317_RTC_BANK_MAIN);
		spin_unlock_irqrestore(&rtc_lock, flags);
	} else if (cobt_is_alpine()) {
		unsigned char val;
	    
		/* web LED is reverse polarity */
		val = inb(ALPINE_WEBLED_PORT);
		if (newstate & LED_WEBLIGHT) {
			val &= ~ALPINE_WEBLED_BIT;
		} else {
			val |= ALPINE_WEBLED_BIT;
		}
		outb(val, ALPINE_WEBLED_PORT);

                    /* 
                     * the power led is controled by switching the pin between
                     * a GPIO pin (on) and a LED pin (off)
                     */

                outb( ALPINE_POWERLED_CFG, 0x2e );
                val = inb( 0x2f );
		if (newstate & LED_COBALTLOGO) {
			val &= ~ALPINE_LOGOLED_BIT;
		} else {
			val |= ALPINE_LOGOLED_BIT;	
		}
                outb( val, 0x2f );

		if (newstate & LED_SYSFAULT) {
                    val = ALPINE_SYSFAULTLED_BIT;
		} else {
                    val = 0;
		}

		outb(val, ALPINE_POWERLED_PORT);
	}
}

/* blip the front panel leds */
static void 
led_timer_func(unsigned long data)
{
	unsigned int leds = 0;
	struct led_handler *p;
	unsigned long flags;

	/* call all registered callbacks */
	spin_lock_irqsave(&led_handler_lock, flags);
	for (p = led_handler_list; p; p = p->next) {
		leds |= p->function(p->data);
	}
	spin_unlock_irqrestore(&led_handler_lock, flags);
	
	/* set the led hardware */
	spin_lock_irqsave(&led_lock, flags);
	__set_led_hw(led_state | leds | led_blips);
	led_blips = 0;
	spin_unlock_irqrestore(&led_lock, flags);

	/* re-arm ourself */
	mod_timer(&timer, jiffies + FPLED_DEFAULT_HZ);
}

static void
__cobalt_led_set(const unsigned int leds)
{
	led_state = leds;
	__set_led_hw(leds);
}

void 
cobalt_led_set(const unsigned int leds)
{
	unsigned long flags;
	spin_lock_irqsave(&led_lock, flags);
	__cobalt_led_set(leds);
	spin_unlock_irqrestore(&led_lock, flags);
}

void 
cobalt_led_set_bits(const unsigned int leds)
{
	unsigned long flags;
	spin_lock_irqsave(&led_lock, flags);
	__cobalt_led_set(led_state | leds);
	spin_unlock_irqrestore(&led_lock, flags);
}

void 
cobalt_led_clear_bits(const unsigned int leds)
{
	unsigned long flags;
	spin_lock_irqsave(&led_lock, flags);
	__cobalt_led_set(led_state & ~leds);
	spin_unlock_irqrestore(&led_lock, flags);
}

static void
__cobalt_led_set_lazy(const unsigned int leds)
{
	/* the next led timer run will catch these changes */
	led_state = leds;
	/* remember lights that were 'blipped' to force an edge */
	led_blips |= leds;
}

void 
cobalt_led_set_lazy(const unsigned int leds)
{
	unsigned long flags;
	spin_lock_irqsave(&led_lock, flags);
	__cobalt_led_set_lazy(leds);
	spin_unlock_irqrestore(&led_lock, flags);
}

void 
cobalt_led_set_bits_lazy(const unsigned int leds)
{
	unsigned long flags;
	spin_lock_irqsave(&led_lock, flags);
	__cobalt_led_set_lazy(led_state | leds);
	spin_unlock_irqrestore(&led_lock, flags);
}

void 
cobalt_led_clear_bits_lazy(const unsigned int leds)
{
	unsigned long flags;
	spin_lock_irqsave(&led_lock, flags);
	__cobalt_led_set_lazy(led_state & ~leds);
	spin_unlock_irqrestore(&led_lock, flags);
}

unsigned int 
cobalt_led_get(void)
{
	unsigned int r;
	unsigned long flags;

	spin_lock_irqsave(&led_lock, flags);
	r = led_state;
	spin_unlock_irqrestore(&led_lock, flags);

	return r;
}

int 
cobalt_fpled_register(unsigned int (*function)(void *), void *data)
{
	struct led_handler *newh;
	unsigned long flags;

	newh = kmalloc(sizeof(*newh), GFP_ATOMIC);
	if (!newh) {
		EPRINTK("can't allocate memory for handler %p(%p)\n",
			function, data);
		return -1;
	}

	spin_lock_irqsave(&led_handler_lock, flags);

	/* head insert */
	newh->function = function;
	newh->data = data;
	newh->next = led_handler_list;
	newh->prev = NULL;
	if (led_handler_list) {
		led_handler_list->prev = newh;
	}
	led_handler_list = newh;
	
	spin_unlock_irqrestore(&led_handler_lock, flags);

	return 0;
}

int 
cobalt_fpled_unregister(unsigned int (*function)(void *), void *data)
{
	int r = -1;
	struct led_handler *p;
	unsigned long flags;
	
	spin_lock_irqsave(&led_handler_lock, flags);

	for (p = led_handler_list; p; p = p->next) {
		if (p->function == function && p->data == data) {
			if (p->prev) {
				p->prev->next = p->next;
			}
			if (p->next) {
				p->next->prev = p->prev;
			}
			r = 0;
			break;
		}
	}

	spin_unlock_irqrestore(&led_handler_lock, flags);

	return r;
}

int __init 
cobalt_led_init(void)
{	
	unsigned int leds = LED_SHUTDOWN | LED_COBALTLOGO;

	if (cobt_is_3k()) {
		/* LEDs for RaQ3/4 and Qube3 are on the PMU */
		led_dev = pci_find_device(PCI_VENDOR_ID_AL,
			PCI_DEVICE_ID_AL_M7101, NULL);
		if (!led_dev) {
			EPRINTK("can't find PMU for LED control\n");
			return -1;
		}
	} 
	
	/* setup up timer for fp leds */
	init_timer(&timer);
	timer.expires = jiffies + FPLED_DEFAULT_HZ;
	timer.data = 0;
	timer.function = &led_timer_func;
	add_timer(&timer);

	/* set the initial state */
	leds |= cobalt_cmos_read_flag(COBT_CMOS_SYSFAULT_FLAG) ? 
		LED_SYSFAULT : 0;
	led_state = leds;
	__set_led_hw(leds);

	return 0;
}

#endif /* CONFIG_COBALT_LED */
