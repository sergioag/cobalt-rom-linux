/* $Id: sensors.c,v 1.31 2002/08/29 00:33:01 uzi Exp $
 * Copyright (c) 2000-2001 Sun Microsystems, Inc 
 *
 * This should be SMP safe.  There is just one race - the read in /proc.
 * It now guards against itself with a semaphore.  Note that we don't use a
 * spinlock because any of the methods may (and do!) block.
 */
#include <linux/config.h>
#ifdef CONFIG_COBALT_SENSORS

#include <stdarg.h>
#include <stddef.h>

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/config.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/proc_fs.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include <cobalt/cobalt.h>
#include <cobalt/systype.h>
#include <cobalt/i2c.h>
#include <cobalt/sensors.h>
#include <cobalt/acpi.h>

/* externals */
unsigned int cobalt_nthermals;
unsigned int cobalt_nvoltages;

/* data about a sensor for generic handling */
/* we could add data about a low/high range, if needed */
struct sensor {
	int sensor; /* sensor #, so maps can be logically ordered */
	char *desc;
	int last_val;
	unsigned long cache;
	unsigned long cache_timeout;
	/* pre/post hook - 1 for pre, 0 for post */
	void (*setup)(struct sensor *s, int pre); 
	/* read as an int, to be passed to format() */
	int (*read)(struct sensor *s);
	/* hook for scaling values */
	int (*scale)(struct sensor *s, int val);
	/* format the value as a string */
	char *(*format)(struct sensor *s, int val, char *buf, int len);
};

/* some stuff for generic formatting */
#define DEC_SCALAR		100
static char *decimal_format(struct sensor *s, int val, char *buf, int len);

static DECLARE_MUTEX(sensor_sem);
static struct sensor *therm_map;
static struct sensor *volt_map;

#define CACHE_DEF		30

#ifdef CONFIG_PROC_FS
static struct proc_dir_entry *proc_csensors;
static struct proc_dir_entry *proc_therm;
static struct proc_dir_entry *proc_volt;
static int therm_read_proc(char *buf, char **start, off_t pos, int len,
	int *eof, void *x);
static int therm_write_proc(struct file *file, const char *buf,
	unsigned long len, void *x);
static int volt_read_proc(char *buf, char **start, off_t pos, int len,
	int *eof, void *x);
static int volt_write_proc(struct file *file, const char *buf,
	unsigned long len, void *x);
#endif

static int lm77_therm_read(struct sensor *s);
static int adm1029_init(void);
static int adm1029_therm_read(struct sensor *s);
static int adm1029_volt_read(struct sensor *s);
static int alpine_vcore_scale(struct sensor *s, int val);
static void alpine_vbat_switch(struct sensor *s, int pre);
static int alpine_vbat_scale(struct sensor *s, int val);

/* sensor name mappings */
static struct sensor gen3_therm_map[] = {
	{0, "CPU", 0, 0, CACHE_DEF, NULL, lm77_therm_read, NULL, decimal_format},
};
static struct sensor monterey_therm_map[] = {
	{0, "CPU0", 0, 0, CACHE_DEF, NULL, lm77_therm_read, NULL, decimal_format},
	{1, "CPU1", 0, 0, CACHE_DEF, NULL, lm77_therm_read, NULL, decimal_format},
	{2, "Case0", 0, 0, CACHE_DEF, NULL, lm77_therm_read, NULL, decimal_format},
	{3, "Case1", 0, 0, CACHE_DEF, NULL, lm77_therm_read, NULL, decimal_format},
};
static struct sensor alpine_therm_map[] = {
	{1, "CPU", 0, 0, CACHE_DEF, NULL, adm1029_therm_read, NULL, decimal_format},
	{0, "Case", 0, 0, CACHE_DEF, NULL, adm1029_therm_read, NULL, decimal_format},
};
static struct sensor alpine_volt_map[] = {
	{0, "Vcore", 0, 0, CACHE_DEF, NULL, adm1029_volt_read, 
		alpine_vcore_scale, decimal_format},
	{1, "Vtt", 0, 0, CACHE_DEF, NULL, adm1029_volt_read, NULL, decimal_format},
	{0, "Vbat", 0, 0, CACHE_DEF<<10, alpine_vbat_switch, adm1029_volt_read, 
		alpine_vbat_scale, decimal_format},
};

int __init
cobalt_sensors_init(void)
{
	if (cobt_is_3k()) {
		cobalt_nthermals = 1;
		cobalt_nvoltages = 0;
		therm_map = gen3_therm_map;
	} else if (cobt_is_monterey()) {
		cobalt_nthermals = 4;
		cobalt_nvoltages = 0;
		therm_map = monterey_therm_map;
	} else if (cobt_is_alpine()) {
		cobalt_nthermals = 2;
		cobalt_nvoltages = 3;
		therm_map = alpine_therm_map;
		volt_map = alpine_volt_map;
		adm1029_init();
	} else  {
		return -1;
	}

#ifdef CONFIG_PROC_FS
	/* make files in /proc */
	proc_csensors = proc_mkdir("sensors", proc_cobalt);
	if (!proc_csensors) {
		EPRINTK("can't create /proc/cobalt/sensors\n");
		return -1;
	}
	if (cobalt_nthermals) {
		proc_therm = create_proc_entry("thermal",
					       S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH,
					       proc_csensors);
		if (!proc_therm) {
			EPRINTK("can't create /proc/cobalt/sensors/thermal\n");
		}
		proc_therm->read_proc = therm_read_proc;
		proc_therm->write_proc = therm_write_proc;
	}
	if (cobalt_nvoltages) {
		proc_volt = create_proc_entry("voltage",
					      S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH,
					      proc_csensors);
		if (!proc_volt) {
			EPRINTK("can't create /proc/cobalt/sensors/voltage\n");
		}
		proc_volt->read_proc = volt_read_proc;
		proc_volt->write_proc = volt_write_proc;

	}
#endif

	return 0;
}

static char *
sensor_read(struct sensor *s, char *buf, int len)
{
	int val;

	if (s->cache && time_after(s->cache_timeout*HZ + s->cache, jiffies))
		val = s->last_val;
	else {
		if (s->setup) s->setup(s, 1);
		val = s->read(s);
		s->last_val = val;
		s->cache = jiffies;
		if (s->setup) s->setup(s, 0);
	}
		
	if (s->scale) val = s->scale(s, val);
	return s->format(s, val, buf, len);
}

/* exported - nicer inline functions in header */
char *
__cobalt_thermal_read(unsigned int idx, char *buf, int len)
{
	if (idx >= cobalt_nthermals || !buf) {
		return NULL;
	}

	return sensor_read(&therm_map[idx], buf, len);
}

/* exported - nicer inline functions in header */
char *
__cobalt_voltage_read(unsigned int idx, char *buf, int len)
{
	if (idx >= cobalt_nvoltages || !buf) {
		return NULL;
	}

	return sensor_read(&volt_map[idx], buf, len);
}

/* generic function for formatting decimal scaled data */
static char *
decimal_format(struct sensor *s, int val, char *buf, int len)
{
	int plen;
	
	if (!buf || len <= 0) {
		return NULL;
	}

	plen = snprintf(buf, len, "%d", val/DEC_SCALAR);
	len -= plen;

	if (val % DEC_SCALAR && len > 0) {
		snprintf(buf+plen, len, ".%02d", val%DEC_SCALAR);
	}

	return buf;
}

#define LM77_TEMP		0x0
static int 
lm77_therm_read(struct sensor *s)
{
	int sensor = s->sensor;
	int tmp;
	int val = 0;
	int tries = 2;

	/* sometimes it reads as zero... try again */
	while (tries--) {
		/* LM77 returns the bytes backwards - <shrug> */
		/* address = base + deviceid + 1 for read */
		val = cobalt_i2c_read_word(COBALT_I2C_DEV_LM77 +
			(sensor<<1) + 1, LM77_TEMP);
		if (val < 0) {
			/* read failed, return the last known value */
			return s->last_val;
		}

		tmp = (val<<8 & 0xff00) + (val>>8 & 0x00ff);
		if (tmp) {
			val = tmp >> 4;
			val *= DEC_SCALAR;
			if (tmp & 0x8) {
				val += DEC_SCALAR/2;
			}
			break;
		}
	}
	return val;
}

#define ADM1029_CTL_CFAULT_OVER		0x01
#define ADM1029_CTL_ALARM_OVER		0x02
#define ADM1029_CTL_INT_OVER		0x04
#define ADM1029_CTL_ALARM_LOW		0x08
#define ADM1029_CTL_CFAULT_UNDER	0x10
#define ADM1029_CTL_ALARM_UNDER		0x20
#define ADM1029_CTL_INT_UNDER		0x40
#define ADM1029_CTL_LATCH		0x80

#define ADM1029_FAN_CTL(i)		(0x18 + i)
#define ADM1029_TEMP_CTL(i)		(0x40 + i)
#define ADM1029_AIN_CTL(i)		(0x50 + i)

#define ADM1029_TEMP_HIGH(i)		(0x90 + i)
#define ADM1029_TEMP_LOW(i)		(0x98 + i)
#define ADM1029_AIN_HIGH(i)		(0xa8 + i)
#define ADM1029_AIN_LOW(i)		(0xb0 + i)

#define ADM1029_TEMP_VALUE(i)		(0xa0 + i)
#define ADM1029_AIN_VALUE(i)		(0xb8 + i)

#ifdef CONFIG_COBALT_ACPI
static int
adm1029_handler(cobalt_acpi_evt *evt, void * data)
{
	int j, k;

	switch (evt->ev_type) {
	case COBALT_ACPI_EVT_SM_INT:
		evt->ev_data = 0;
		evt->ev_type = COBALT_ACPI_EVT_VOLT;
		for (j=0; j<cobalt_nvoltages; j++) {
			k = cobalt_i2c_read_byte(COBALT_I2C_DEV_ADM1029,
				 ADM1029_AIN_CTL(volt_map[j].sensor));
			if (k & ADM1029_CTL_LATCH) {
				evt->ev_data |= (1 << j);
				volt_map[j].cache = 0;
			}
		}
		break;

	case COBALT_ACPI_EVT_THERM:
		evt->ev_data = 0;
		for (j=0; j<cobalt_nthermals; j++) {
			k = cobalt_i2c_read_byte(COBALT_I2C_DEV_ADM1029,
				 ADM1029_TEMP_CTL(therm_map[j].sensor));
			if (k & ADM1029_CTL_LATCH) {
				evt->ev_data |= (1 << j);
				therm_map[j].cache = 0;
			}
		}
		break;

	default:
		return -1;
	}
	return 0;
}
#endif /* CONFIG_COBALT_ACPI */

static int 
adm1029_init(void)
{

#ifdef CONFIG_COBALT_ACPI
	cobalt_acpi_register_evt_handler(adm1029_handler, 
		COBALT_ACPI_EVT_THERM, NULL);
	cobalt_acpi_register_evt_handler(adm1029_handler, 
		COBALT_ACPI_EVT_SM_INT, NULL);
#endif

	return 0;
}

static int 
adm1029_therm_read(struct sensor *s)
{
	int sensor = s->sensor;
	int val;

	val = cobalt_i2c_read_byte(COBALT_I2C_DEV_ADM1029, 
		ADM1029_TEMP_VALUE(sensor));
	if (val < 0) {
		/* read failed, return the last known value */
		return s->last_val;
	}
	if (val & 0x80) {
		val -= 256;
	}
	val *= DEC_SCALAR;

	return val;
}

static int
adm1029_volt_read(struct sensor *s)
{
	int sensor = s->sensor;
	int val;

	val = cobalt_i2c_read_byte(COBALT_I2C_DEV_ADM1029, 
		ADM1029_AIN_VALUE(sensor));
	if (val < 0) {
		/* read failed, return the last known value */
		return s->last_val;
	}
	
	/* already scaled by 100 */
	val *= DEC_SCALAR/100;

	return val;
}

static int
alpine_vcore_scale(struct sensor *s, int val)
{
	/* the measured Vbat switch cost is negligable
	 * due to very low current through the diode */
	return val;
}

#define VBAT_REG	0x608
#define VBAT_BIT	0x1
static void
alpine_vbat_switch(struct sensor *s, int pre)
{
	unsigned char v = inb(VBAT_REG);
	unsigned long j = jiffies;

	if (pre) {
		v |= VBAT_BIT;
		/*
		 * disable AIN0 INT# assertion before switching to
		 * Vbat because the input is shared with Vcore and
		 * their acceptable ranges are very different.
		 */
		cobalt_i2c_write_byte(COBALT_I2C_DEV_ADM1029,
			ADM1029_AIN_CTL(s->sensor), 0x0);
	} else {
		v &= ~VBAT_BIT;
	}

	outb(v, VBAT_REG);

	/*
	 * wait for the round-robin monitor to complete a cycle
	 * before _and_ after toggling Vbat switch, otherwise
	 * stale data in AIN0 will trigger INT# assertion.
	 */
	while ((jiffies - j) < HZ) {
		/* block for ~ 1sec */
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ);
	}

	if (!pre) {
		/*
		 * now re-enable INT# assertion capability for AIN0
		 * (this also clears the AIN0 fault latch at bit 7)
		 */
		cobalt_i2c_write_byte(COBALT_I2C_DEV_ADM1029,
			ADM1029_AIN_CTL(s->sensor),
			ADM1029_CTL_INT_OVER | ADM1029_CTL_INT_UNDER);
	}
}

static int
alpine_vbat_scale(struct sensor *s, int val)
{
	/* 
	 * The spec says 2.5V max - but empirically, 3.3V works :)
	 * The Vbat switch costs 0.3 volts
	 */
	if (val) val += (3 * DEC_SCALAR)/10;

	return val;
}

#ifdef CONFIG_PROC_FS
static int
sensor_write_proc(int nsensors, struct sensor *map,
	struct file *file, const char *buf, unsigned long len, void *x)
{
	char *pg;

	if (len > PAGE_SIZE) {
		return -EOVERFLOW;
	}

	pg = (char *)__get_free_page(GFP_KERNEL);
	if (!pg) {
		return -ENOMEM;
	}

	if (copy_from_user(pg, buf, len)) {
		free_page((unsigned long)pg);
		return -EFAULT;
	}
	pg[len] = '\0';

	/* format: `cache_timeout #' in seconds */
	if (len>15 && !strncmp("cache_timeout ", pg, 14) && isdigit(*(pg+14))) {
		unsigned long i, sec = simple_strtoul(pg+14, NULL, 0);
		for (i=0; i<nsensors; i++)
			map[i].cache_timeout = sec;
	}

	free_page((unsigned long)pg);
	return len;
}

static int
sensor_read_proc(int nsensors, struct sensor *map,
	char *buf, char **start, off_t pos, int len, int *eof, void *x)
{
	int i;
	static int plen = 0;

	down(&sensor_sem);

	/* remember how big our last read was to avoid read() calling twice */
	if (pos && pos >= plen) {
		*eof = 1;
		up(&sensor_sem);
		return 0;
	}

	plen = 0;
	for (i = 0; i < nsensors; i++) {
		char sbuf[32];
		if (sensor_read(&map[i], sbuf, sizeof(sbuf)))
			plen += sprintf(buf+plen, "%d [%s]: %s\n", i, map[i].desc, sbuf);
	}

	up(&sensor_sem);

	return cobalt_gen_proc_read(buf, plen, start, pos, len, eof);
}

static int
therm_read_proc(char *buf, char **start, off_t pos, int len, int *eof, void *x)
{
	return sensor_read_proc(cobalt_nthermals, therm_map,
		buf, start, pos, len, eof, x);
}
static int
therm_write_proc(struct file *file, const char *buf, unsigned long len, void *x)
{
	return sensor_write_proc(cobalt_nthermals, therm_map, file, buf, len, x);
}

static int
volt_read_proc(char *buf, char **start, off_t pos, int len, int *eof, void *x)
{
	return sensor_read_proc(cobalt_nvoltages, volt_map,
		buf, start, pos, len, eof, x);
}
static int
volt_write_proc(struct file *file, const char *buf, unsigned long len, void *x)
{
	return sensor_write_proc(cobalt_nvoltages, volt_map, file, buf, len, x);
}
#endif /* CONFIG_PROC_FS */

#endif /* CONFIG_COBALT_SENSORS */
