#include <linux/config.h>

#include <linux/vmalloc.h>
#include <asm/io.h>
#include <asm/pgalloc.h>

#include <linux/init.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/notifier.h>
#include <linux/ctype.h>

extern asmlinkage ssize_t sys_read(unsigned int fd, char * buf, size_t count);
extern asmlinkage int sys_umount(char * name, int flags);
extern struct notifier_block *reboot_notifier_list;

static int c_remap_area_pages(unsigned long address, unsigned long phys_addr,
				 unsigned long size, unsigned long flags);

/* fallback values for ramcode */
#define RAMCODE_START 0x01000000  /* 16MB */
#define RAMCODE_END   0x02000000  /* 32MB */

#define MAX_KERNEL_SIZE	0x00200000 /* 2MB */

struct cobalt_boot_return_data
{
	int error;
	int flen;
};

static unsigned int cobalt_boot_load;
static unsigned int cobalt_boot_return;
static unsigned int cobalt_boot_data;
static unsigned int cobalt_ramcode_start = 0;
static unsigned int cobalt_ramcode_len = 0;

static char cobalt_boot_image[512] = "/vmlinux.gz";

static int __init 
cobalt_boot_image_setup( char *str )
{
	if (str && *str) {
		strncpy(cobalt_boot_image, str, sizeof(cobalt_boot_image));
	} else {
		printk(KERN_CRIT 
			"BOOTLOADER: cobalt_boot_image not used correctly\n");
	}
	return 1;
}

static unsigned int 
get_int(char *str)
{
	unsigned int val;
	char *endp;

	if (!isdigit(*str)) {
		return 0;
	}

	val = simple_strtoul(str, &endp, 0);

	if (*endp) {
		return 0;
	}
	return val;
}

static int __init 
cobalt_boot_return_setup(char *str)
{
	unsigned int val = get_int(str);

	if (val > 0) {
		cobalt_boot_return = val;
	} else {
		printk(KERN_CRIT
			"BOOTLOADER: cobalt_boot_return not used correctly\n");
	}
	return 1;
}

static int __init 
cobalt_boot_data_setup(char *str)
{
	int val = get_int(str);

	if (val > 0) {
		cobalt_boot_data = val;
	} else {
		printk(KERN_CRIT
			"BOOTLOADER: cobalt_boot_data not used correctly\n");
	}
	return 1;
}

static int __init 
cobalt_boot_load_setup(char *str)
{
	unsigned int val = get_int(str);

	if (val > 0) {
		cobalt_boot_load = val;
	} else {
		printk(KERN_CRIT
			"BOOTLOADER: cobalt_boot_load not used correctly\n");
	}

	return 1;
}

static int __init 
cobalt_ramcode_setup(char *str)
{
	char *k;
	int start;
	int len;
 
	k = strchr(str, ',');
	if (k) {
		*k++ = '\0';
	}
	
	if (!isdigit(*str)) {
		printk( KERN_CRIT
			"BOOTLOADER: cobalt_ramcode_map not used correctly\n");
	}

	start = simple_strtoul(str, NULL, 0);
	len = simple_strtoul(k, NULL, 0);

	cobalt_ramcode_start = start;
	cobalt_ramcode_len = len;

	return 1;
}

__setup("cobalt_boot_load=", cobalt_boot_load_setup);
__setup("cobalt_boot_return=", cobalt_boot_return_setup);
__setup("cobalt_boot_data=", cobalt_boot_data_setup);
__setup("cobalt_boot_image=", cobalt_boot_image_setup);
__setup("cobalt_ramcode_map=", cobalt_ramcode_setup);

int cobalt_boot_open(char *boot_image)
{
	char *c;
	int i = 0;
	int done = 0;
	int fd;

	while (!done) {
		c = boot_image+i;

		while (boot_image[i] && (boot_image[i] != ',')) {
			i++;
		}

		if (!boot_image[i]) {
			done = 1;
		}
	
		boot_image[i++] = '\0';

		printk(KERN_CRIT
			"BOOTLOADER: opening \"%s\"\n", c);

		if ((fd = sys_open(c, O_RDONLY, 0)) >= 0) {
			i = 0;
			while (*c) {
				boot_image[i++] = *c++;
			}
			boot_image[i] = *c;

			return fd;
		}
	}
	return -1;
}

void cobalt_boot_do_it(void)
{
	unsigned char *load_addr;
	struct cobalt_boot_return_data *ret_data;
	int fd, read_len;
 
	if (!(cobalt_boot_load && cobalt_boot_return && cobalt_boot_data)) {
		return;
	}

	if (!(cobalt_ramcode_start || cobalt_ramcode_len)) {
		cobalt_ramcode_start = RAMCODE_START;
		cobalt_ramcode_len = RAMCODE_END - RAMCODE_START;
	}

	printk(KERN_CRIT
		"BOOTLOADER: Mapping in physical locations\n");

	/* map in physical locations of for kernel */
	load_addr = (unsigned char *) 
		ioremap(cobalt_boot_load, MAX_KERNEL_SIZE);
	ret_data = (struct cobalt_boot_return_data *)ioremap(cobalt_boot_data, 
		sizeof(struct cobalt_boot_return_data));

	printk(KERN_CRIT
		"BOOTLOADER: load_addr=0x%08x ret_data=0x%08x\n", 
		(unsigned int)load_addr,
		(unsigned int)ret_data);

	if ((fd = cobalt_boot_open(cobalt_boot_image)) < 0) {
		ret_data->error = -1;
		goto boot_end;
	}

	printk(KERN_CRIT
		"BOOTLOADER: reading \"%s\"\n", cobalt_boot_image );

	ret_data->flen = 0;

	while ((read_len = sys_read(fd, load_addr, 
	 MAX_KERNEL_SIZE - ret_data->flen)) > 0) {
		ret_data->flen += read_len;
		load_addr += read_len;
	}

	if (read_len < 0) {
		ret_data->error = -2;
		goto boot_end;
	}

	printk(KERN_CRIT
		"BOOTLOADER: read %dbytes\n", ret_data->flen);

	printk(KERN_CRIT "BOOTLOADER: unmounting /\n");

	if (sys_umount("/", 0) < 0) {
		ret_data->error = -3;
		goto boot_end;
	}
	ret_data->error = 0;

  boot_end:
	printk(KERN_CRIT
		"BOOTLOADER: calling reboot notifiers\n" );

	notifier_call_chain(&reboot_notifier_list, SYS_RESTART, NULL);

	printk(KERN_CRIT
		"BOOTLOADER: mapping %dM-%dM for ride home\n", 
		cobalt_ramcode_start >> 20, (cobalt_ramcode_len + 
		cobalt_ramcode_start) >> 20);


	/* map in ramcode so we can turn off paging in the rom */
	c_remap_area_pages(cobalt_ramcode_start, cobalt_ramcode_start, 
		cobalt_ramcode_len, 0);

	printk(KERN_CRIT "BOOTLOADER: disabling interrupts\n");

	cli();

	printk(KERN_CRIT "BOOTLOADER: flushing cache\n");

	flush_cache_all();

	printk(KERN_CRIT "BOOTLOADER: Leap of faith!\n");

	/* call back to rom */
	((void(*)(void))(cobalt_boot_return))();
}

/* why cant these functions be public!?!?
 */
static inline void c_remap_area_pte(pte_t * pte, unsigned long address, unsigned long size,
	unsigned long phys_addr, unsigned long flags)
{
        unsigned long end;

        address &= ~PMD_MASK;
        end = address + size;
        if (end > PMD_SIZE)
                end = PMD_SIZE;
        if (address >= end)
                BUG();
        do {
                if (!pte_none(*pte)) {
                        printk("remap_area_pte: page already exists\n");
                        BUG();
                }
                set_pte(pte, mk_pte_phys(phys_addr, __pgprot(_PAGE_PRESENT | _PAGE_RW | 
                                        _PAGE_DIRTY | _PAGE_ACCESSED | flags)));
                address += PAGE_SIZE;
                phys_addr += PAGE_SIZE;
                pte++;
        } while (address && (address < end));
}

static inline int c_remap_area_pmd(pmd_t * pmd, unsigned long address, unsigned long size,
	unsigned long phys_addr, unsigned long flags)
{
        unsigned long end;

        address &= ~PGDIR_MASK;
        end = address + size;
        if (end > PGDIR_SIZE)
                end = PGDIR_SIZE;
        phys_addr -= address;
        if (address >= end)
                BUG();
        do {
                pte_t * pte = pte_alloc(&init_mm, pmd, address);
                if (!pte)
                        return -ENOMEM;
                c_remap_area_pte(pte, address, end - address, address + phys_addr,
 flags);
                address = (address + PMD_SIZE) & PMD_MASK;
                pmd++;
        } while (address && (address < end));
        return 0;
}

static int c_remap_area_pages(unsigned long address, unsigned long phys_addr,
				 unsigned long size, unsigned long flags)
{
        int error;
        pgd_t * dir;
        unsigned long end = address + size;

        phys_addr -= address;
        dir = pgd_offset(&init_mm, address);
        flush_cache_all();
        if (address >= end)
                BUG();
        spin_lock(&init_mm.page_table_lock);
        do {
                pmd_t *pmd;
                pmd = pmd_alloc(&init_mm, dir, address);
                error = -ENOMEM;
                if (!pmd)
                        break;
                if (c_remap_area_pmd(pmd, address, end - address,
                                         phys_addr + address, flags))
                        break;
                error = 0;
                address = (address + PGDIR_SIZE) & PGDIR_MASK;
                dir++;
        } while (address && (address < end));
        spin_unlock(&init_mm.page_table_lock);
        flush_tlb_all();
        return error;
}
