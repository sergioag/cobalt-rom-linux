/* $Id: cobalt.c,v 1.34 2002/11/04 17:54:14 thockin Exp $ */
#include <linux/config.h>

#include <linux/types.h>
#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/ptrace.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/timer.h>
#include <linux/init.h>

#include <cobalt/cobalt.h>
#include <cobalt/misc.h>
#include <cobalt/led.h>
#include <cobalt/wdt.h>
#include <cobalt/acpi.h>
#include <cobalt/superio.h>
#include <cobalt/systype.h>

#define MAX_NMI_PS	10

static inline void ledonoff(unsigned long on, unsigned long off);

static u8 last_err;
static u32 last_address;
static unsigned long nmi_repeats;
static struct timer_list nmi_timer;
static int timer_added;
static unsigned long nmi_count;
static spinlock_t nmi_state_lock = SPIN_LOCK_UNLOCKED;

/* clla this holding nmi_state_lock */
static inline void
do_repeats(void)
{
	if (nmi_repeats) {
		printk("NMI: last error repeated %lu times\n", nmi_repeats);
		nmi_repeats = 0;
	}
}

static void 
nmi_throttle_fn(unsigned long data)
{
	unsigned long flags;

	spin_lock_irqsave(&nmi_state_lock, flags);

	/* clear any repeated NMIs */
	do_repeats();

	/* have we had a lot of errors this second */
	if (nmi_count > MAX_NMI_PS) {
		printk("NMI: %lu messages were throttled\n", 
			nmi_count - MAX_NMI_PS);
		nmi_count = 0;
	}

	/* de-activate the timer - will be reactivated by an NMI */
	del_timer(&nmi_timer);
	timer_added = 0;

	spin_unlock_irqrestore(&nmi_state_lock, flags);
}

void 
cobalt_nmi(unsigned char reason, struct pt_regs *regs)
{
	if (cobt_is_5k()) {
		static struct pci_dev *cnb_dev;
		u8 err;
		u32 address = 0;
		unsigned long flags;

		/* find our memory controller */
		if (!cnb_dev) {
			cnb_dev = pci_find_device(PCI_VENDOR_ID_SERVERWORKS,
				  PCI_DEVICE_ID_SERVERWORKS_LE, NULL);
		}
		if (!cnb_dev) {
			EPRINTK("can't find north bridge for NMI status\n");
			return;
		}

		/* read the error number */
		pci_read_config_byte(cnb_dev, 0x47, &err);

		/* if a memory error was detected, where? */
		if (err & 0x06) {
			pci_read_config_dword(cnb_dev, 0x94, &address);
		}

		spin_lock_irqsave(&nmi_state_lock, flags);

		/* set up the timer, if it isn't set to go already */
		if (!timer_added) {
			init_timer(&nmi_timer);
			nmi_timer.expires = jiffies + HZ;
			nmi_timer.function = nmi_throttle_fn;
			add_timer(&nmi_timer);
			timer_added = 1;
		}

		/* if we already printed this error */
		if (last_err && err == last_err && address == last_address) {
			nmi_repeats++;
			spin_unlock_irqrestore(&nmi_state_lock, flags);
		} else {
			unsigned long nmi_now;

			/* different error - show repeats */
			do_repeats();

			/* we only want to do a few messages per second */
			nmi_now = nmi_count++;

			spin_unlock_irqrestore(&nmi_state_lock, flags);

			/* generate a new message */
			if (nmi_now < MAX_NMI_PS) {
				/* only remember NMIs that we can print */
				last_err = err;
				last_address = address;

				printk("NMI:");
				if (err & 0x40)
					printk(" (PCI tx data error)");
				if (err & 0x20)
					printk(" (PCI rx data error)");
				if (err & 0x10)
					printk(" (PCI address error)");
				if (err & 0x04)
					printk(" (DRAM uncorrectable error)");
				if (err & 0x02)
					printk(" (DRAM correctable error)");
				if (err & 0x01)
					printk(" (Shutdown cycle detected)");

				if (err & 0x06) {
					u8 row, dimm, ecc;

					row = (address >> 29) & 0x7;
					pci_read_config_byte(cnb_dev, 
						0x7c + (row >> 1), &dimm);
					dimm = ((row & 1) ? 
						(dimm >> 4) : dimm) & 0xf;
					pci_read_config_byte(cnb_dev, 0xe8, 
						&ecc);

					printk(" [memory row %d, DIMM type %d, "
						"col=0x%x, row=0x%x, ECC=0x%x]",
						row, dimm, 
						(address >> 15) & 0x3fff, 
						address & 0x7fff, ecc);
				} 
				printk("\n");
			}
		}

		/* clear errors */
		pci_write_config_byte(cnb_dev, 0x47, err); 
	} else {
		/* TODO: make throttling generic, handle GP NMIs */
		printk("NMI: unknown error\n");
	}
}

void 
cobalt_restart(void)
{
	if (cobt_is_3k()) {
		/* kick watchdog */
		cobalt_wdt_trigger_reboot();
	} else if (cobt_is_5k()) {
		/* set "Enable Hard Reset" bit to 1 */
		outb(0x02, 0x0cf9);

		/* 0-to-1 transition of bit 2 will cause reset of processor */
		outb(0x06, 0x0cf9);
	}
	mdelay(3000);

	/* we should not get here unless there is a BAD error */
	EPRINTK("can not restart - halting\n");
	machine_halt();
}

void
cobalt_halt(void)
{
	int haltok = current_cpu_data.hlt_works_ok;

	if (cobt_is_5k()) {
		/* we have soft power-off */
		machine_power_off();
	}

	/* 
	 * we want to do cpu_idle, but we don't actually want to 
	 * call cpu_idle. bleah. 
	 */
	while (1) {
		ledonoff(HZ >> 1, HZ >> 1);
		if (haltok) {
			__asm__("hlt");
		}
	}
}

static inline void 
ledonoff(unsigned long on, unsigned long off)
{
#ifdef CONFIG_COBALT_LED
	unsigned long start;
	int haltok = current_cpu_data.hlt_works_ok;

	if (on) {
		start = jiffies;
		cobalt_led_set(cobalt_led_get() | LED_SHUTDOWN);
		while (jiffies < start + on) {
			if (haltok) __asm__("hlt");
		}
	}

	if (off) {
		start = jiffies;
		cobalt_led_set(cobalt_led_get() & ~LED_SHUTDOWN);
		while (jiffies < start + off) {
			if (haltok) __asm__("hlt");
		}
	}
#endif
}

void
cobalt_power_off(void)
{
	u16 addr;

	if (cobt_is_monterey()) {
		u8 val;
		/* use card control reg. 7 to select logical device 2 (APC) */
		addr = superio_ldev_base(PC87317_DEV_RTC);

		/* set up bank 2 */
		outb(PC87317_RTC_CRA, addr);
		val = inb(addr + 1) & 0x8f;
		outb(val | PC87317_RTC_BANK_2, addr + 1);

		/* power off the machine with APCR1 */
		outb(PC87317_APCR1, addr);
		val = inb(addr + 1);
		outb(0x20 | val, addr + 1);
	} else if (cobt_is_alpine()) {
		int i;
		/* clear status bits, base addr 3 */
		addr = superio_ldev_base_n(PC87417_DEV_SWC, 3);
		for (i = 0; i < 4; i++) {
			/* 
			 * if we have an event while running, 
			 * we can't halt unless we clear these
			 * */
			outb(0xff, addr+i);
		}

		/* set sleep state, base addr 2 */
		addr = superio_ldev_base_n(PC87417_DEV_SWC, 2);
		/* PM1b_CNT_HIGH @offset 1 - set state to S5 */
		outb(0x34, addr+1);
	}
	mdelay(3000);
	EPRINTK("can not power off\n");
}

/* put arch specific stuff to run at init time here */
static int __init
cobalt_arch_init(void)
{
	return 0;
}
module_init(cobalt_arch_init);
