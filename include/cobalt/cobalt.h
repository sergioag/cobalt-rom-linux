/* $Id: cobalt.h,v 1.16 2002/11/04 17:54:15 thockin Exp $ */
/* Copyright 2001 Sun Microsystems, Inc. */
#include <linux/config.h>
#if !defined(COBALT_H) && defined(CONFIG_COBALT)
#define COBALT_H

/* generational support - for easy checking */
#ifdef CONFIG_COBALT_GEN_III
# define COBT_SUPPORT_GEN_III 1
#else
# define COBT_SUPPORT_GEN_III 0
#endif

#ifdef CONFIG_COBALT_GEN_V
# define COBT_SUPPORT_GEN_V 1
#else
# define COBT_SUPPORT_GEN_V 0
#endif

/* macros for consistent errors/warnings */
#define EPRINTK(fmt, args...) \
	printk(KERN_ERR "%s:%s: " fmt , __FILE__ , __FUNCTION__ , ##args)

#define WPRINTK(fmt, args...) \
	printk(KERN_WARNING "%s:%s: " fmt , __FILE__ , __FUNCTION__ , ##args)

/* the root of /proc/cobalt */
extern struct proc_dir_entry *proc_cobalt;
int cobalt_gen_proc_read(char *buf, int plen, char **start, off_t pos, 
	int len, int *eof);

#ifdef CONFIG_COBALT_RAQ
/* keep this test up to date with new generation defines */
#if !defined(CONFIG_COBALT_GEN_III) && !defined(CONFIG_COBALT_GEN_V)
/* barf if no generation has been selected */
#error You asked for CONFIG_COBALT_RAQ, but no CONFIG_COBALT_GEN_* !
#endif

/* accesses for CMOS */
#include <linux/mc146818rtc.h>
#include <cobalt/nvram.h>

static inline int
cobalt_cmos_read_flag(const unsigned int flag)
{
	unsigned long flags;
	u16 cmosfl;

	spin_lock_irqsave(&rtc_lock, flags);
	cmosfl = CMOS_READ(COBT_CMOS_FLAG_BYTE_0) << 8;
	cmosfl |= CMOS_READ(COBT_CMOS_FLAG_BYTE_1);
	spin_unlock_irqrestore(&rtc_lock, flags);

	return (cmosfl & flag) ? 1 : 0;
}
#endif /* CONFIG_COBALT_RAQ */

#endif /* !defined(COBALT_H) && defined(CONFIG_COBALT) */
