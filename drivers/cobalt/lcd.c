/*
 * $Id: lcd.c,v 1.44 2002/05/10 18:44:45 duncan Exp $
 * lcd.c : driver for Cobalt LCD/Buttons
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
 * This should be SMP safe. We're hardly performance critical,
 * so we lock around lcd_ioctl() and just where needed by other external
 * functions.  There is a static global waiters variable that is atomic_t, and
 * so should be safe. --TPH
 */

#include <linux/config.h>

#ifdef CONFIG_COBALT_LCD

#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/stat.h>
#include <linux/netdevice.h>
#include <linux/proc_fs.h>
#include <linux/in6.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/kernel.h>

#include <asm/io.h>
#include <asm/segment.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/checksum.h>
#include <linux/delay.h>

#include <cobalt/cobalt.h>
#include <cobalt/systype.h>
#include <cobalt/lcd.h>
#include <cobalt/superio.h>
#include <cobalt/i2c.h>

#define TWIDDLE_HZ (HZ/10)

#ifndef min
#define min(a, b)  ((a) < (b) ? (a) : (b))
#endif

#define LCD_DRIVER		"Cobalt Networks LCD driver"
#define LCD_DRIVER_VMAJ		4
#define LCD_DRIVER_VMIN		0

/* this is the generic device minor assigned to /dev/lcd */
#define COBALT_LCD_MINOR	156
#define COBALT_LCD_OLD_MINOR	140

/* io registers */
#define LPT			0x0378
#define LCD_DATA_ADDRESS	LPT+0
#define LCD_CONTROL_ADDRESS	LPT+2

/* LCD device info */
#define LCD_Addr		0x80
#define DD_R00			0x00
#define DD_R01			0x27
#define DD_R10			0x40
#define DD_R11			0x67

/* driver functions */
static int cobalt_lcd_open(struct inode *, struct file *);
static ssize_t cobalt_lcd_read(struct file *, char *, size_t, loff_t *);
static int cobalt_lcd_read_proc(char *, char **, off_t, int, int *, void *);
static char *cobalt_lcddev_read_line(int, char *);
static int cobalt_lcd_ioctl(struct inode *, struct file *,
	unsigned int, unsigned long);
static int cobalt_lcd_panic(struct notifier_block *self, unsigned long, void *);

/* globals used throughout */
#ifdef CONFIG_PROC_FS
#ifdef CONFIG_COBALT_OLDPROC
static struct proc_dir_entry *proc_lcd;
#endif
static struct proc_dir_entry *proc_clcd;
#endif
static int lcd_present;
static int has_i2c_lcd;
static spinlock_t lcd_lock = SPIN_LOCK_UNLOCKED;

/* various file operations we support for this driver */
static struct file_operations lcd_fops = {
	owner:	THIS_MODULE,
	read:	cobalt_lcd_read,
	ioctl:	cobalt_lcd_ioctl,
	open:	cobalt_lcd_open,
};

/* device structure */
static struct miscdevice lcd_dev = {
	COBALT_LCD_MINOR,
	"lcd",
	&lcd_fops
};
#ifdef CONFIG_COBALT_LCD_DEV_COMPAT
/* device structure */
static struct miscdevice lcd_compat_dev = {
	COBALT_LCD_OLD_MINOR,
	"lcd (compatible)",
	&lcd_fops
};
#endif

static int disable_lcd;
static int __init 
lcd_disable_setup(char *str)
{
	disable_lcd = 1;
	return 0;
}
__setup("nolcd", lcd_disable_setup);

/* Read a control instruction from the LCD */
static inline int 
lcddev_read_inst(void)
{
	int a = 0;

	if (cobt_is_5k() && has_i2c_lcd) {
		a = cobalt_i2c_read_byte(
			COBALT_I2C_DEV_LCD_INST | COBALT_I2C_READ, 0);
	} else if (cobt_is_3k() || (cobt_is_5k() && !has_i2c_lcd)) {
		outb(0x21, LCD_CONTROL_ADDRESS); /* RS=0, R/W=1, E=0 */
		outb(0x20, LCD_CONTROL_ADDRESS); /* RS=0, R/W=1, E=1 */
		a = inb(LCD_DATA_ADDRESS);
		outb(0x21, LCD_CONTROL_ADDRESS); /* RS=0, R/W=1, E=0 */
		outb(0x01, LCD_CONTROL_ADDRESS); /* RS=0, R/W=1, E=0 */
	} 

	/* small delay */
	udelay(100);

	return a;
}

#define LCD_MAX_POLL 10000
static inline void 
lcddev_poll_wait(void) 
{
	int i=0;

	while (i++ < LCD_MAX_POLL) {
		int r = lcddev_read_inst();
		if (r < 0 || !(r & 0x80))
			break;
	}
}

/* Write a control instruction to the LCD */
static inline void 
lcddev_write_inst(unsigned char data)
{
	lcddev_poll_wait();

	if (cobt_is_5k() && has_i2c_lcd) {
		cobalt_i2c_write_byte(
			COBALT_I2C_DEV_LCD_INST | COBALT_I2C_WRITE, 0, data);
	} else if (cobt_is_3k() || (cobt_is_5k() && !has_i2c_lcd)) {
		outb(0x03, LCD_CONTROL_ADDRESS); /* RS=0, R/W=0, E=0 */
		outb(data, LCD_DATA_ADDRESS);
		outb(0x02, LCD_CONTROL_ADDRESS); /* RS=0, R/W=0, E=1 */
		outb(0x03, LCD_CONTROL_ADDRESS); /* RS=0, R/W=0, E=0 */
		outb(0x01, LCD_CONTROL_ADDRESS); /* RS=0, R/W=1, E=0 */
	}

	/* small delay */
	udelay(100);
}

/* Write one byte of data to the LCD */
static inline void 
lcddev_write_data(unsigned char data)
{
	lcddev_poll_wait();

	if (cobt_is_5k() && has_i2c_lcd) {
		cobalt_i2c_write_byte(
			COBALT_I2C_DEV_LCD_DATA | COBALT_I2C_WRITE, 0, data);
	} else if (cobt_is_3k() || (cobt_is_5k() && !has_i2c_lcd)) {
		outb(0x07, LCD_CONTROL_ADDRESS); /* RS=1, R/W=0, E=0 */
		outb(data, LCD_DATA_ADDRESS);
		outb(0x06, LCD_CONTROL_ADDRESS); /* RS=1, R/W=0, E=1 */
		outb(0x07, LCD_CONTROL_ADDRESS); /* RS=1, R/W=0, E=0 */
		outb(0x05, LCD_CONTROL_ADDRESS); /* RS=1, R/W=1, E=0 */
	}
	/* small delay */
	udelay(100);
}

/* Read one byte of data from the LCD */
static inline unsigned char 
lcddev_read_data(void)
{
	unsigned char a = 0;

	lcddev_poll_wait();

	if (cobt_is_5k() && has_i2c_lcd) {
		a = cobalt_i2c_read_byte(
			COBALT_I2C_DEV_LCD_DATA | COBALT_I2C_READ, 0);
	} else if (cobt_is_3k() || (cobt_is_5k() && !has_i2c_lcd)) {
		outb(0x25, LCD_CONTROL_ADDRESS); /* RS=1, R/W=1, E=0 */
		outb(0x24, LCD_CONTROL_ADDRESS); /* RS=1, R/W=1, E=1 */
		a = inb(LCD_DATA_ADDRESS);
		outb(0x25, LCD_CONTROL_ADDRESS); /* RS=1, R/W=1, E=0 */
		outb(0x01, LCD_CONTROL_ADDRESS); /* RS=1, R/W=1, E=0 */
	}

	/* small delay */
	udelay(100);

	return a;
}

static inline void
lcddev_init(void)
{
	lcddev_write_inst(0x38);
	lcddev_write_inst(0x38);
	lcddev_write_inst(0x38);
	lcddev_write_inst(0x06);
	lcddev_write_inst(0x0c);
}

static inline char 
read_buttons(void)
{
	char r = 0;

	if (cobt_is_5k() && has_i2c_lcd) {
		unsigned char inst;
		inst = cobalt_i2c_read_byte(COBALT_I2C_DEV_FP_BUTTONS, 0);
		switch (inst) {
			case 0x3e: r = BUTTON_Next_B; break;
			case 0x3d: r = BUTTON_Enter_B; break;
			case 0x1f: r = BUTTON_Left_B; break;
			case 0x3b: r = BUTTON_Right_B; break;
			case 0x2f: r = BUTTON_Up_B; break;
			case 0x37: r = BUTTON_Down_B; break;
			case 0x3f: 
			default: r = BUTTON_NONE_B;
		}
	} else if (cobt_is_3k() || (cobt_is_5k() && !has_i2c_lcd)) {
		outb(0x29, LCD_CONTROL_ADDRESS); /* Sel=0, Bi=1 */
		r = inb(LCD_DATA_ADDRESS) & BUTTON_MASK;
	}
	
	return r;
}

static inline int 
button_pressed(void)
{
	unsigned char b;
	unsigned long flags;

	spin_lock_irqsave(&lcd_lock, flags);
	b = read_buttons();
	spin_unlock_irqrestore(&lcd_lock, flags);

	switch (b) {
	case BUTTON_Next:
	case BUTTON_Next_B:
	case BUTTON_Reset_B:
		return b;
	default:
	}

	return 0;
}

/* this could be protected by CAP_RAW_IO here, or by the FS permissions */
static int 
cobalt_lcd_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
	struct lcd_display button_display, display;
	unsigned long address, a;
	int index;
	int dlen = sizeof(struct lcd_display);
	int r = 0;
	unsigned long flags;

#ifdef CONFIG_COBALT_LCD_TWIDDLE
	cobalt_lcd_stop_twiddle();
#endif	
	switch (cmd) {
	/* Turn the LCD on */
	case LCD_On:
		spin_lock_irqsave(&lcd_lock, flags);
		lcddev_write_inst(0x0F);
		spin_unlock_irqrestore(&lcd_lock, flags);
		break;		

	/* Turn the LCD off */
	case LCD_Off:
		spin_lock_irqsave(&lcd_lock, flags);
		lcddev_write_inst(0x08);
		spin_unlock_irqrestore(&lcd_lock, flags);
		break;

	/* Reset the LCD */
	case LCD_Reset:
		spin_lock_irqsave(&lcd_lock, flags);
		lcddev_write_inst(0x3F);
		lcddev_write_inst(0x3F);
		lcddev_write_inst(0x3F);
		lcddev_write_inst(0x3F);
		lcddev_write_inst(0x01);
		lcddev_write_inst(0x06);
		spin_unlock_irqrestore(&lcd_lock, flags);
		break;

	/* Clear the LCD */
	case LCD_Clear:
		spin_lock_irqsave(&lcd_lock, flags);
		lcddev_write_inst(0x01);
		spin_unlock_irqrestore(&lcd_lock, flags);
		break;

	/* Move the cursor one position to the left */
	case LCD_Cursor_Left:
		spin_lock_irqsave(&lcd_lock, flags);
		lcddev_write_inst(0x10);
		spin_unlock_irqrestore(&lcd_lock, flags);
		break;

	/* Move the cursor one position to the right */
	case LCD_Cursor_Right:
		spin_lock_irqsave(&lcd_lock, flags);
		lcddev_write_inst(0x14);
		spin_unlock_irqrestore(&lcd_lock, flags);
		break;	

	/* Turn the cursor off */
	case LCD_Cursor_Off:
		spin_lock_irqsave(&lcd_lock, flags);
		lcddev_write_inst(0x0C);
		spin_unlock_irqrestore(&lcd_lock, flags);
		break;

	/* Turn the cursor on */
	case LCD_Cursor_On:
		spin_lock_irqsave(&lcd_lock, flags);
		lcddev_write_inst(0x0F);
		spin_unlock_irqrestore(&lcd_lock, flags);
		break;

	/* Turn blinking off? I don't know what this does - TJS */
	case LCD_Blink_Off:
		spin_lock_irqsave(&lcd_lock, flags);
		lcddev_write_inst(0x0E);
		spin_unlock_irqrestore(&lcd_lock, flags);
		break;

	/* Get the current cursor position */
	case LCD_Get_Cursor_Pos:
		spin_lock_irqsave(&lcd_lock, flags);
		display.cursor_address = (unsigned char)lcddev_read_inst();
		display.cursor_address = display.cursor_address & 0x07F;
		spin_unlock_irqrestore(&lcd_lock, flags);
		if (copy_to_user((struct lcd_display *)arg, &display, dlen)) {
			r = -EFAULT;
		}
		break;

	/* Set the cursor position */
	case LCD_Set_Cursor_Pos:
		if (copy_from_user(&display, (struct lcd_display *)arg, dlen)) {
			r = -EFAULT;
			break;
		}
		a = display.cursor_address | LCD_Addr;
		spin_lock_irqsave(&lcd_lock, flags);
		lcddev_write_inst(a);
		spin_unlock_irqrestore(&lcd_lock, flags);
		break;

	/* Get the value at the current cursor position? - TJS */
	case LCD_Get_Cursor:
		spin_lock_irqsave(&lcd_lock, flags);
		display.character = lcddev_read_data();
		lcddev_write_inst(0x10);
		spin_unlock_irqrestore(&lcd_lock, flags);
		if (copy_to_user((struct lcd_display *)arg, &display, dlen)) {
			r = -EFAULT;
		}
		break;

	/* Set the character at the cursor position? - TJS */
	case LCD_Set_Cursor:
		if (copy_from_user(&display, (struct lcd_display *)arg, dlen)) {
			r = -EFAULT;
			break;
		}
		spin_lock_irqsave(&lcd_lock, flags);
		lcddev_write_data(display.character);
		lcddev_write_inst(0x10);
		spin_unlock_irqrestore(&lcd_lock, flags);
		break;

	/* Dunno what this does - TJS */ 
	case LCD_Disp_Left:
		spin_lock_irqsave(&lcd_lock, flags);
		lcddev_write_inst(0x18);
		spin_unlock_irqrestore(&lcd_lock, flags);
		break;

	/* Dunno what this does - TJS */ 
	case LCD_Disp_Right:
		spin_lock_irqsave(&lcd_lock, flags);
		lcddev_write_inst(0x1C);
		spin_unlock_irqrestore(&lcd_lock, flags);
		break;

	/* Dunno what this does - TJS */ 
	case LCD_Home:
		spin_lock_irqsave(&lcd_lock, flags);
		lcddev_write_inst(0x02);
		spin_unlock_irqrestore(&lcd_lock, flags);
		break;

	/* Write a string to the LCD */
	case LCD_Write:
		if (copy_from_user(&display, (struct lcd_display *)arg, dlen)) {
			r = -EFAULT;
			break;
		}

		spin_lock_irqsave(&lcd_lock, flags);

		display.size1 = display.size1 > 0 ? 
		  min(display.size1, (int) sizeof(display.line1)) : 0;
		display.size2 = display.size2 > 0 ? 
		  min(display.size2, (int) sizeof(display.line2)) : 0;

		/* First line */
		lcddev_write_inst(0x80);
		for (index = 0; index < display.size1; index++)
			lcddev_write_data(display.line1[index]);
		for (index = display.size1; index < sizeof(display.line1); index++)
			lcddev_write_data(' ');

		/* Second line */
		lcddev_write_inst(0xC0);	
		for (index = 0; index < display.size2; index++)
			lcddev_write_data(display.line2[index]);
		for (index = display.size2; index < sizeof(display.line2); index++)
			lcddev_write_data(' ');

		spin_unlock_irqrestore(&lcd_lock, flags);
		break;	

	/* Read what's on the LCD */
	case LCD_Read:
		spin_lock_irqsave(&lcd_lock, flags);

		for (address = DD_R00; address <= DD_R01; address++) {
			lcddev_write_inst(address | LCD_Addr);
			display.line1[address] = lcddev_read_data();
		}
		for (address = DD_R10; address <= DD_R11; address++) {
			lcddev_write_inst(address | LCD_Addr);
			display.line2[address - DD_R10] = lcddev_read_data();
		}

		spin_unlock_irqrestore(&lcd_lock, flags);

		display.line1[DD_R01] = '\0';
		display.line2[DD_R01] = '\0';

		if (copy_to_user((struct lcd_display *)arg, &display, dlen)) {
			r = -EFAULT;
		}
		break;

	case LCD_Raw_Inst:
		if (copy_from_user(&display, (struct lcd_display *)arg, dlen)) {
			r = -EFAULT;
			break;
		}
		spin_lock_irqsave(&lcd_lock, flags);
		lcddev_write_inst(display.character);
		spin_unlock_irqrestore(&lcd_lock, flags);
		break;

	case LCD_Raw_Data:
		if (copy_from_user(&display, (struct lcd_display *)arg, dlen)) {
			r = -EFAULT;
			break;
		}
		spin_lock_irqsave(&lcd_lock, flags);
		lcddev_write_data(display.character);
		spin_unlock_irqrestore(&lcd_lock, flags);
		break;

	case LCD_Type:
		if (cobt_is_5k() && has_i2c_lcd) {
			if (put_user(LCD_TYPE_I2C, (int *)arg)) {
				r = -EFAULT;
			}
		} else if (cobt_is_3k() || (cobt_is_5k() && !has_i2c_lcd)) {
			if (put_user(LCD_TYPE_PARALLEL_B, (int *)arg)) {
				r = -EFAULT;
			}
		}
		break;
		
	/* Read the buttons */
	case BUTTON_Read:
		spin_lock_irqsave(&lcd_lock, flags);
		button_display.buttons = read_buttons();
		spin_unlock_irqrestore(&lcd_lock, flags);
		if (copy_to_user((struct lcd_display *)arg, 
				&button_display, dlen)) {
			r = -EFAULT;
		}
		break;
	
#ifdef CONFIG_COBALT_LED
	/* a slightly different api that allows you to set 32 leds */
	case LED32_Set:
		cobalt_led_set_lazy(arg);
		break;

	case LED32_Bit_Set:
		cobalt_led_set_bits_lazy(arg);
		break;

	case LED32_Bit_Clear:
		cobalt_led_clear_bits_lazy(arg);
		break;

	case LED32_Get:
		*(unsigned int *)arg = cobalt_led_get();
		break;

	/* set all the leds */
	case LED_Set:
		if (copy_from_user(&display, (struct lcd_display *)arg, dlen)) {
			r = -EFAULT;
			break;
		}
		cobalt_led_set_lazy(display.leds);
		break;

	/* set a single led */
	case LED_Bit_Set:
		if (copy_from_user(&display, (struct lcd_display *)arg, dlen)) {
			r = -EFAULT;
			break;
		}
		cobalt_led_set_bits_lazy(display.leds);
		break;

	/* clear an led */
	case LED_Bit_Clear:
		if (copy_from_user(&display, (struct lcd_display *)arg, dlen)) {
			r = -EFAULT;
			break;
		}
		cobalt_led_clear_bits_lazy(display.leds);
		break;
#endif

	default:
	}

	return r;
}

static int 
cobalt_lcd_open(struct inode *inode, struct file *file)
{
	if (!lcd_present) {
		return -ENXIO;
	} else {
		return 0;
	}
}

/* LCD daemon sits on this, we wake it up once a key is pressed */
static ssize_t 
cobalt_lcd_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	int bnow;
	static unsigned long lcd_waiters;

	if (test_and_set_bit(0, &lcd_waiters)) {
		return -EINVAL;
	}

	while (((bnow = button_pressed()) == 0) && !(signal_pending(current))) {
		if (file->f_flags & O_NONBLOCK) {
			lcd_waiters = 0;
			return -EAGAIN;
		}
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(2 * HZ);
	}
	lcd_waiters = 0;

	if (signal_pending(current)) {
		return -ERESTARTSYS;
	}

	return bnow;
}

/* read a single line from the LCD into a string */
static char *
cobalt_lcddev_read_line(int lineno, char *line)
{
	unsigned long addr, min, max;
	unsigned long flags;
	
	switch (lineno) {
	case 0:
		min = DD_R00;
		max = DD_R01;
		break;
	case 1:
		min = DD_R10;
		max = DD_R11;
		break;
	default:
		min = 1;
		max = 0;
	}

	spin_lock_irqsave(&lcd_lock, flags);
	for (addr = min; addr <= max; addr++) {
		lcddev_write_inst(addr | LCD_Addr);
		udelay(150);
		line[addr-min] = lcddev_read_data();
		udelay(150);
	}
	spin_unlock_irqrestore(&lcd_lock, flags);
	line[addr-min] = '\0';

	return line;
}

#ifdef CONFIG_PROC_FS
static int 
cobalt_lcd_read_proc(char *buf, char **start, off_t pos,
				int len, int *eof, void *private)
{
	int plen = 0;
	char line[COBALT_LCD_LINELEN+1];

	/* first line */
	cobalt_lcddev_read_line(0, line);
	plen += sprintf(buf+plen, "%s\n", line);

	/* second line */
	cobalt_lcddev_read_line(1, line);
	plen += sprintf(buf+plen, "%s\n", line);
	
	return cobalt_gen_proc_read(buf, plen, start, pos, len, eof);
}
#endif

static char *lcd_panic_str1 = "Kernel";
static char *lcd_panic_str2 = "Panic!";

static int cobalt_lcd_panic(struct notifier_block *self, unsigned long a, void *b)
{
    int i;
    int len;
    
    if( !lcd_present )
        return 0;
    
#ifdef CONFIG_COBALT_LCD_TWIDDLE
    cobalt_lcd_stop_twiddle();
#endif

    lcddev_write_inst( (DD_R00) | LCD_Addr);
    len = strlen( lcd_panic_str1 );
    for( i=0 ; i<16 ; i++ )
	lcddev_write_data( (i<len)?lcd_panic_str1[i]:' ' );

    lcddev_write_inst( (DD_R10) | LCD_Addr);
    len = strlen( lcd_panic_str2 );
    for( i=0 ; i<16 ; i++ )
	lcddev_write_data( (i<len)?lcd_panic_str2[i]:' ' );

    return 0;
}

#ifdef CONFIG_COBALT_LCD_TWIDDLE
static struct timer_list twiddle_timer;
static int twiddling;
static void 
twiddle_timer_func(unsigned long data)
{
	static int state=1;
	static int pos=0;
	unsigned long flags;

	spin_lock_irqsave(&lcd_lock, flags);

	lcddev_write_inst((DD_R10+4+pos) | LCD_Addr);
	lcddev_write_data(' ');

	pos += state;
	if (pos < 0) {
		state = 1; 
		pos = 1;
	}
	if (pos > 11) {
		state = -1; 
		pos = 10;
	}

	lcddev_write_inst((DD_R10+4+pos) | LCD_Addr);
	lcddev_write_data(0xff);

	spin_unlock_irqrestore(&lcd_lock, flags);

	mod_timer(&twiddle_timer, jiffies + TWIDDLE_HZ);
}

void 
cobalt_lcd_start_twiddle(void)
{
	init_timer(&twiddle_timer);
	twiddle_timer.expires = jiffies + TWIDDLE_HZ;
	twiddle_timer.data = 0;
	twiddle_timer.function = &twiddle_timer_func;
	add_timer(&twiddle_timer); 
	twiddling=1; 
}

void 
cobalt_lcd_stop_twiddle(void)
{
	unsigned int flags;

	spin_lock_irqsave(&lcd_lock, flags);
	if (twiddling) {
		del_timer_sync(&twiddle_timer);
		twiddling = 0;
	}
	spin_unlock_irqrestore(&lcd_lock, flags);
}
#endif /* CONFIG_COBALT_LCD_TWIDDLE */

/* stop the lcd */
void cobalt_lcd_off(void)
{
	unsigned int flags;

	spin_lock_irqsave(&lcd_lock, flags);
	lcddev_write_inst(0x01); /* clear */
	lcddev_write_inst(0x08); /* off */
	spin_unlock_irqrestore(&lcd_lock, flags);
}

static int initialized;
static struct notifier_block lcd_nb;

int __init 
cobalt_lcd_init(void)
{	
	if (initialized)
		return 0;

	initialized=1;

	if (disable_lcd) {
		printk(KERN_INFO "%s DISABLED\n", LCD_DRIVER);
		return 0;
	}

	misc_register(&lcd_dev);
#ifdef CONFIG_COBALT_LCD_DEV_COMPAT
	misc_register(&lcd_compat_dev);
#endif

	if (cobt_is_monterey() 
	 && (cobalt_i2c_read_byte(COBALT_I2C_DEV_LCD_INST, 0) != 0xff)) {
		has_i2c_lcd = 1;
	} else {
		has_i2c_lcd = 0;
	}

	/* flag ourselves as present */
	lcd_present = 1;

	/* initialize the device */
	lcddev_init();

#ifdef CONFIG_PROC_FS
#ifdef CONFIG_COBALT_OLDPROC
	/* create /proc/lcd */
	proc_lcd = create_proc_read_entry("lcd", S_IRUSR, NULL, 
		cobalt_lcd_read_proc, NULL);
	if (!proc_lcd) {
		EPRINTK("can't create /proc/lcd\n");
	}
#endif
	proc_clcd = create_proc_read_entry("lcd", S_IRUSR, proc_cobalt, 
		cobalt_lcd_read_proc, NULL);
	if (!proc_clcd) {
		EPRINTK("can't create /proc/cobalt/lcd\n");
	}
#endif

#ifdef CONFIG_COBALT_LCD_TWIDDLE
	cobalt_lcd_start_twiddle();
#endif

            /* register panic notifier */
        lcd_nb.notifier_call = cobalt_lcd_panic;
        lcd_nb.next = NULL;
        lcd_nb.priority = 0;
        
        notifier_chain_register( &panic_notifier_list, &lcd_nb );

	return 0;
}

#endif /* CONFIG_COBALT_LCD */
