/*
 * $Id: cobalt-led.h,v 1.7 2001/11/08 01:15:33 thockin Exp $
 * cobalt-led.c
 *
 * Copyright 1996-2000 Cobalt Networks, Inc.
 * Copyright 2001 Sun Microsystems, Inc.
 *
 * By:	Andrew Bose	
 *	Timothy Stonis (x86 version)
 *	Tim Hockin
 *	Adrian Sun
 *	Erik Gilling
 *	Duncan Laurie
 */
#ifndef COBALT_LED_H
#define COBALT_LED_H

/* the set of all leds available on Cobalt systems */
#define LED_SHUTDOWN		(1 << 0)
#define LED_WEBLIGHT		(1 << 1)
#define LED_COBALTLOGO		(1 << 2)
#define LED_ETH0_TXRX		(1 << 3)
#define LED_ETH0_LINK		(1 << 4)
#define LED_ETH1_TXRX		(1 << 5)
#define LED_ETH1_LINK		(1 << 6)
#define LED_DISK0		(1 << 7)
#define LED_DISK1		(1 << 8)
#define LED_DISK2		(1 << 9)
#define LED_DISK3		(1 << 10)
#define LED_SYSFAULT		(1 << 11)
#define LED_MONTEREY_UNUSED0	(1 << 12)
#define LED_MONTEREY_UNUSED1	(1 << 13)
/* LED_MONTEREY_UNUSED2 is below */
#define LED_HEART		LED_MONTEREY_UNUSED0
#define LED_SPARE		LED_MONTEREY_UNUSED1
#define LED_SLED0		(1 << 14)
#define LED_SLED1		(1 << 15)
#define LED_SLED2		(1 << 16)
#define LED_SLED3		(1 << 17)
#define LED_MONTEREY_UNUSED2	(1 << 18)
#define LED_SPARE2		LED_MONTEREY_UNUSED2

#ifdef __KERNEL__

extern void cobalt_led_set(const unsigned int leds);
extern void cobalt_led_set_bits(const unsigned int leds);
extern void cobalt_led_clear_bits(const unsigned int leds);
extern void cobalt_led_set_lazy(const unsigned int leds);
extern void cobalt_led_set_bits_lazy(const unsigned int leds);
extern void cobalt_led_clear_bits_lazy(const unsigned int leds);
extern unsigned int cobalt_led_get(void);

extern int cobalt_fpled_register(unsigned int (*fn)(void *), void *data);
extern int cobalt_fpled_unregister(unsigned int (*fn)(void *), void *data);

#endif /* __KERNEL__ */

#endif /* COBALT_LED_H */

