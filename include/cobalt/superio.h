/*
 * $Id: cobalt-superio.h,v 1.11 2001/11/02 12:00:03 duncan Exp $
 * cobalt-superio.h : SuperIO support for Sun/Cobalt servers
 *
 * Copyright 2000 Cobalt Networks, Inc.
 * Copyright 2001 Sun Microsystems, Inc.
 */
#ifndef COBALT_SUPERIO_H
#define COBALT_SUPERIO_H

#include <cobalt/cobalt.h>
#include <cobalt/systype.h>

/* the lock that protects superio accesses */
extern spinlock_t cobalt_superio_lock;

/* 
 * The main functions of the SuperIO are accessed via index/data registers
 * These are the same for both SuperIO chip families
 */
#define SUPERIO_INDEX_PORT	0x2e
#define SUPERIO_DATA_PORT	0x2f

/* 
 * This index allows you to select a logical device 
 */
#define SUPERIO_LOGICAL_DEV	0x07

/*
 * Type of SUPERIO
 */
#define SUPERIO_SID             0x20

/* 
 * Some indices that are common to all logical devices 
 */
#define SUPERIO_ACTIVATE	0x30
#define SUPERIO_ADDR_HIGH	0x60
#define SUPERIO_ADDR_LOW	0x61
#define SUPERIO_INTR_PIN	0x70
#define SUPERIO_INTR_TYPE	0x71


/* 
 * PC87317 SuperIO is found on XTR 
 */

/*
 * logical devices - write to LOGICAL_DEV index
 */
#define PC87317_DEV_RTC		0x02
#define PC87317_DEV_GPIO	0x07
#define PC87317_DEV_PM		0x08

/* withing the PM dev, are the following indices */
#define PC87317_PMDEV_WDTO	0x05
#define PC87317_PMDEV_GPELO	0x0e
#define PC87317_PMDEV_GPEHI	0x0f

/* within the APC bank of RTC */
#define PC87317_APCR1		0x40
#define PC87317_APCR2		0x41
#define PC87317_APCR3		0x49
#define PC87317_APCR4		0x4a
#define PC87317_APCR5		0x4b
#define PC87317_APCR6		0x4c
#define PC87317_APCR7		0x4d

#define PC87317_RTC_BANK_MAIN	0
#define PC87317_RTC_BANK_RTC	1
#define PC87317_RTC_BANK_APC	2
#define PC87317_RTC_CRA		0x0a
#define PC87317_RTC_BANK_0	0x20
#define PC87317_RTC_BANK_1	0x30
#define PC87317_RTC_BANK_2	0x40

#define PC87317_PWRBUTTON	0x01
#define PC87317_PM1_STATUS	0x01

/* offsets from GPEHI/GPELO */
#define PC87317_GPE_GP1_STS0	0x00
#define PC87317_GPE_GP1_STS1	0x01
#define PC87317_GPE_GP1_STS2	0x02
#define PC87317_GPE_GP1_STS3	0x03
#define PC87317_GPE_GP1_EN0	0x04
#define PC87317_GPE_GP1_EN1	0x05
#define PC87317_GPE_GP1_EN2	0x06
#define PC87317_GPE_GP1_EN3	0x07
#define PC87317_GPE_GP2_EN0	0x08
#define PC87317_GPE_SMI_CMD	0x0c

/*
 * PC87417 family is found on alpine 
 */

#define PC87417_DEV_SWC		0x4
#define PC87417_DEV_GPIO	0x7
#define PC87417_DEV_RTC		0x10

/* registers in the SWC dev */
#define PC87417_SWC_PWONCTL	0x9
/*
 * within the System Wake Control device, there are banks, write the bank you
 * want to SWC_BANK
 */
#define PC87417_SWC_BANK	0xf
#define PC87417_SWCBANK_ACPI	0x2
#define PC87417_SWCBANK_WDT	0x3

/*
 * the SWC WDT bank has 3 main registers 
 */
#define PC87417_WDT_CONTROL	0x10
#define PC87417_WDT_TIMEOUT	0x11
#define PC87417_WDT_CONFIG	0x12

/*
 * within the SWC, two regs serve as index/data for wake-event enabling
 */
#define PC87417_SWC_WKEVENT	0x0
#define PC87417_SWC_WKSTATE	0x1
#define PC87417_SWCWKEVENT_GPIOE42 0xa
#define PC87417_SWCWKEVENT_RTC 	0x18


/*
 * types of superIOs
 */
enum superio_type_t
{
    SIO_TYPE_UNKNOWN,
    SIO_TYPE_PC8731X,
    SIO_TYPE_PC8741X,
};



#define PC8731X_SID 0xd0
#define PC9731X_SID 0xdf
#define PC8741X_SID 0xee

static inline int superio_type( void )
{
    u8 reg;
    unsigned long flags;
    enum superio_type_t type;

    spin_lock_irqsave(&cobalt_superio_lock, flags);

    outb(SUPERIO_SID, SUPERIO_INDEX_PORT);
    reg = inb( SUPERIO_DATA_PORT );
    switch( reg )
    {
	case PC8731X_SID:
	case PC9731X_SID:
	    type = SIO_TYPE_PC8731X;
	    break;

	case PC8741X_SID:
	    type = SIO_TYPE_PC8741X;
	    break;

	default:
	    type = SIO_TYPE_UNKNOWN;
	    break;
    }

    spin_unlock_irqrestore(&cobalt_superio_lock, flags);

    return type;
}

/*
 * stuff to make life easier
 */

/* read the base address of a particular superio logical device */
#define superio_ldev_base(ldev)		superio_ldev_base_n(ldev, 0)
static inline unsigned short
superio_ldev_base_n(int ldev, int n)
{
	unsigned long flags;
	unsigned short addr;

	spin_lock_irqsave(&cobalt_superio_lock, flags);

	/* select the logical device */
	outb(SUPERIO_LOGICAL_DEV, SUPERIO_INDEX_PORT);
	outb(ldev, SUPERIO_DATA_PORT);

	/* read the 2-byte base address */
	outb(SUPERIO_ADDR_HIGH+(n*2), SUPERIO_INDEX_PORT);
	addr = inb(SUPERIO_DATA_PORT) << 8;
	outb(SUPERIO_ADDR_LOW+(n*2), SUPERIO_INDEX_PORT);
	addr |= inb(SUPERIO_DATA_PORT);

	spin_unlock_irqrestore(&cobalt_superio_lock, flags);

	return addr;
}

static inline void
superio_set_rtc_bank(const unsigned int page)
{
	if (cobt_is_monterey()) {
		unsigned char val;

		val = CMOS_READ(0xa);
		val &= ~0x70;
		switch (page) {
		case PC87317_RTC_BANK_MAIN:
			val |= 0x20;
			break;
		case PC87317_RTC_BANK_RTC:
			val |= 0x30;
			break;
		case PC87317_RTC_BANK_APC:
			val |= 0x40;
			break;
		}
		CMOS_WRITE(val, 0xa);
	}
}

#endif /* COBALT_SUPERIO_H */
