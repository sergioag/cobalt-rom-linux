 /* 
 * cobalt acpi driver 
 * Copyright (c) 2000, Cobalt Networks, Inc.
 * Copyright (c) 2001, Sun Microsystems, Inc.
 * $Id: acpi.c,v 1.32 2002/06/26 19:08:54 duncan Exp $
 *
 * author: asun@cobalt.com, thockin@sun.com
 *
 * this driver just sets stuff up for ACPI interrupts
 *
 * if acpi support really existed in the kernel, we would read
 * data from the ACPI tables. however, it doesn't. as a result,
 * we use some hardcoded values. 
 *
 * This should be SMP safe.  The only data that needs protection is the acpi
 * handler list.  It gets scanned at timer-interrupts, must use
 * irqsave/restore locks. Read/write locks would be useful if there were any
 * other times that the list was read but never written. --TPH
 *
 * /proc/acpi emulation emulates the /proc/acpi/events interface supplied by 
 * the INTEL acpi drivers.  A lot of the code to handle it has been adapted
 * from there.
 */

#include <stdarg.h>
#include <stddef.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/config.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <cobalt/cobalt.h>
#include <cobalt/systype.h>
#include <cobalt/acpi.h>
#include <cobalt/superio.h>

#define POWER_BUTTON_SHUTDOWN	0

#define ACPI_IRQ	10	/* XXX: hardcoded interrupt */
#define ACPI_NAME	"sci"
#define ACPI_MAGIC	0xc0b7ac21

#define SUPERIO_EVENT	0xff
#define OSB4_EVENT	0x40
#define OSB4_INDEX_PORT	SERVERWORKS_ACPI_INDEX_PORT
#define OSB4_DATA_PORT	SERVERWORKS_ACPI_DATA_PORT

#define GEN_ACPI_TMR_STS      (0x1 <<  0)
#define GEN_ACPI_BM_STS       (0x1 <<  4)
#define GEN_ACPI_GBL_STS      (0x1 <<  5)
#define GEN_ACPI_PWRBTN_STS   (0x1 <<  8)
#define GEN_ACPI_SLPBTN_STS   (0x1 <<  9)
#define GEN_ACPI_RTC_STS      (0x1 << 10)
#define GEN_ACPI_WAK_STS      (0x1 << 15)

#ifdef CONFIG_COBALT_EMU_ACPI
static int cobalt_acpi_setup_proc(void);
static int cobalt_acpi_open_event(struct inode *inode, struct file *file);
static int cobalt_acpi_close_event(struct inode *inode, struct file *file);
static ssize_t cobalt_acpi_read_event(struct file *file, char *buf, 
	size_t count, loff_t *ppos);
static unsigned int cobalt_acpi_poll_event(struct file *file, poll_table *wait);
#endif



typedef struct 
{
    u16 hw_type;
    cobalt_acpi_hw_handler hw_handler;
    cobalt_acpi_enable_handler en_handler;
    void *data;
    struct list_head link;
} hw_handler_datum;

typedef struct 
{
    u16 hw_type;
    u16 table_len;
    u16 *table;
    struct list_head link;
} trans_table_datum;

typedef struct 
{
    cobalt_acpi_evt_handler handler;
    u16 ev_type;
    void *data;
    struct list_head link;
} evt_handler_datum;

typedef struct 
{
    cobalt_acpi_evt evt;
    struct list_head link;
} evt_list_datum;

static LIST_HEAD( hw_handler_list );
static spinlock_t hw_handler_list_lock = SPIN_LOCK_UNLOCKED;
static LIST_HEAD( trans_table_list );
static spinlock_t trans_table_list_lock = SPIN_LOCK_UNLOCKED;
static LIST_HEAD( evt_handler_list );
static spinlock_t evt_handler_list_lock = SPIN_LOCK_UNLOCKED;
static LIST_HEAD( dispatch_queue );
static spinlock_t dispatch_queue_lock = SPIN_LOCK_UNLOCKED;
 
typedef struct
{
    u16 hw_type;

	/* block lengths */
    u16 pm1_evt_len;
    u16 pm1_cnt_len;
    u16 pm2_cnt_len;
    u16 pm_tmr_len;
    u16 gpe0_len;
    u16 gpe1_len;
    
	/* block I/O locations */
    u16 pm1a_evt_blk;
    u16 pm1b_evt_blk;
    u16 pm1a_cnt_blk;
    u16 pm1b_cnt_blk;
    u16 pm2_cnt_blk;
    u16 pm_tmr_blk;
    u16 p_blk;
    u16 gpe0_blk;
    u16 gpe1_blk;

	/* ponters to strings for the io names */
    char *pm1a_evt_nam;
    char *pm1b_evt_nam;
    char *pm1a_cnt_nam;
    char *pm1b_cnt_nam;
    char *pm2_cnt_nam;
    char *pm_tmr_nam;
    char *p_nam;
    char *gpe0_nam;
    char *gpe1_nam;

	/* reference counts for events */
    atomic_t tmr_ref_cnt;
    atomic_t bm_ref_cnt;
    atomic_t gbl_ref_cnt;
    atomic_t pwrbtn_ref_cnt;
    atomic_t slpbtn_ref_cnt;
    atomic_t rtc_ref_cnt;
    atomic_t wak_ref_cnt;
    atomic_t *gpe_ref_cnt;


} generic_acpi_regions;


static void cobalt_acpi_enable_event( u16 ev_type, int en );
static void cobalt_acpi_run_enable_handler( u16 hw_type, u16 ev_type, 
					    u16 ev_data, int en);
static int cobalt_acpi_apply_evt_handlers( evt_list_datum *d );
static int cobalt_acpi_run_dispatch_queue( void );
static void acpi_interrupt(int irq, void *dev_id, struct pt_regs *regs);
static void cobalt_acpi_cleanup( void );

static int register_acpi_regions( generic_acpi_regions *regions, char * subsys_name );
static int unregister_acpi_regions( generic_acpi_regions *regions );
static void cobalt_acpi_handle_pm1_blk( u16 io_addr, u16 len, 
					generic_acpi_regions * regions );
static void cobalt_acpi_handle_gpe_blk( u16 io_addr, u16 len, 
					generic_acpi_regions * regions );
static int cobalt_acpi_generic_hw_handler( int irq, void *dev_id, 
					   struct pt_regs *regs, void * data );

static int cobalt_acpi_osb4_init( void );
static int cobalt_acpi_osb4_cleanup( void );
static int get_osb4_regions( generic_acpi_regions *regions);

static int cobalt_acpi_csb5_init( void );
static int cobalt_acpi_csb5_cleanup( void );
static int get_csb5_regions( generic_acpi_regions *regions);

static int cobalt_acpi_pc8731x_init( void );
static int cobalt_acpi_pc8731x_cleanup( void );
static int get_pc8731x_regions( generic_acpi_regions *regions );

static int cobalt_acpi_pc8741x_init( void );
static int cobalt_acpi_pc8741x_cleanup( void );
static int get_pc8741x_regions( generic_acpi_regions *regions );

static int cobalt_acpi_monterey_init( void );
static int cobalt_acpi_monterey_cleanup( void );

static int cobalt_acpi_alpine_init( void );
static int cobalt_acpi_alpine_cleanup( void );

static __inline__ struct list_head *list_pop( struct list_head *head )
{
    struct list_head *e;
    
    if( list_empty( head ) )
	return NULL;
    
    e = head->next;
    list_del( e );
    return e;
}

static __inline__ u16 get_reg(u16 index, u16 data, u8 port)
{
    u16 reg;

    outb(port, index);
    reg = inb(data);
    outb(port + 1, index);
    reg |= inb(data) << 8;
    return reg;
}

/*
 *
 * Main ACPI Subsystem Code
 *
 */

extern int cobalt_acpi_register_hw_handler( u16 hw_type,
					    cobalt_acpi_hw_handler hw_handler, 
					    cobalt_acpi_enable_handler en_handler,
					    void *data )
{
    hw_handler_datum *d;
    unsigned long flags;

    if( ! (d = (hw_handler_datum *) kmalloc( sizeof( hw_handler_datum ), GFP_ATOMIC )) )
	return -ENOMEM;

    d->hw_type = hw_type;
    d->hw_handler = hw_handler;
    d->en_handler = en_handler;
    d->data = data;

    spin_lock_irqsave( &hw_handler_list_lock, flags );
    list_add( &(d->link), &hw_handler_list );
    spin_unlock_irqrestore( &hw_handler_list_lock, flags );

    return 0;
}

extern int cobalt_acpi_unregister_hw_handler( cobalt_acpi_hw_handler handler )
{
    struct list_head *pos;
    unsigned long flags;
    
    spin_lock_irqsave( &hw_handler_list_lock, flags );
    list_for_each( pos, &hw_handler_list )
	{
	    if( list_entry( pos, hw_handler_datum, link )->hw_handler == handler )
	    {
		list_del( pos );
		spin_unlock_irqrestore( &hw_handler_list_lock, flags );

		kfree( list_entry( pos, hw_handler_datum, link ) );
		return 0;
	    }
		
	};
    
    spin_unlock_irqrestore( &hw_handler_list_lock, flags );
    return -1;
}

extern int cobalt_acpi_register_trans_table( u16 hw_type, u16 table_len, u16 *table )
{
    trans_table_datum *d;
    unsigned long flags;
    
    if( ! (d = (trans_table_datum *) kmalloc( sizeof( trans_table_datum ), GFP_ATOMIC )) )
	return -ENOMEM;

    d->hw_type = hw_type;
    d->table_len = table_len;
    d->table = table;

    spin_lock_irqsave( &trans_table_list_lock, flags );
    list_add( &(d->link), &trans_table_list );
    spin_unlock_irqrestore( &trans_table_list_lock, flags );

    return 0;
}

extern int cobalt_acpi_unregister_trans_table( u16 hw_type )
{
    struct list_head *pos;
    unsigned long flags;
    
    spin_lock_irqsave( &trans_table_list_lock, flags );
    list_for_each( pos, &trans_table_list )
	{
	    if( list_entry( pos, trans_table_datum, link )->hw_type == hw_type )
	    {
		list_del( pos );
		spin_unlock_irqrestore( &trans_table_list_lock, flags );
		
		kfree( list_entry( pos, trans_table_datum, link ) );
		return 0;
	    }
		
	};
    
    spin_unlock_irqrestore( &trans_table_list_lock, flags );
    return -1;
}

extern int cobalt_acpi_register_evt_handler( cobalt_acpi_evt_handler handler, 
					     u16 ev_type,
					     void *data )
{
    evt_handler_datum *d;
    unsigned long flags;
    
    if( ! (d = (evt_handler_datum *) kmalloc( sizeof( evt_handler_datum ), GFP_ATOMIC )) )
	return -ENOMEM;

    d->handler = handler;
    d->data = data;
    d->ev_type = ev_type;

    spin_lock_irqsave( &evt_handler_list_lock, flags );
    list_add( &(d->link), &evt_handler_list );
    spin_unlock_irqrestore( &evt_handler_list_lock, flags );

    cobalt_acpi_enable_event( ev_type, 1 );

    return 0;
}

extern int cobalt_acpi_unregister_evt_handler( cobalt_acpi_evt_handler handler )
{
    struct list_head *pos;
    unsigned long flags;
    

    spin_lock_irqsave( &evt_handler_list_lock, flags );
    list_for_each( pos, &evt_handler_list )
	{
	    if( list_entry( pos, evt_handler_datum, link )->handler == handler )
	    {
		list_del( pos );
		spin_unlock_irqrestore( &evt_handler_list_lock, flags );
		
		cobalt_acpi_enable_event( list_entry( pos, 
						      evt_handler_datum, 
						      link )->ev_type, 0 );

		kfree( list_entry( pos, evt_handler_datum, link ) );
		return 0;
	    }
		
	};
    
    spin_unlock_irqrestore( &evt_handler_list_lock, flags );
    return -EINVAL;
}

static void cobalt_acpi_enable_event( u16 ev_type, int en )
{
    if( ev_type >= 0x8000 )
    {
	struct list_head *pos;
	trans_table_datum *d;
	int i;
	unsigned long flags;

	spin_lock_irqsave( &trans_table_list_lock, flags );
	list_for_each( pos, &trans_table_list )
	    {
		d = list_entry( pos, trans_table_datum, link );
		for( i=0 ; i<d->table_len ; i++ )
		{
		    if( d->table[i] == ev_type )
		    {
			cobalt_acpi_run_enable_handler( d->hw_type, 
							COBALT_ACPI_EVT_GPE, 
							i, en );
		    }
		}
	    }
	spin_unlock_irqrestore( &trans_table_list_lock, flags );
    }
    else
	cobalt_acpi_run_enable_handler( COBALT_ACPI_HW_ANY, ev_type, 0, en);
}

static void cobalt_acpi_run_enable_handler( u16 hw_type, u16 ev_type, 
					    u16 ev_data, int en)
{   
    struct list_head *pos;
    unsigned long flags;
    hw_handler_datum *d;

    spin_lock_irqsave(&hw_handler_list_lock, flags);
    list_for_each( pos, &hw_handler_list )
	{
	    d = list_entry( pos, hw_handler_datum, link );
	    if( (!hw_type) || (d->hw_type == hw_type) )
		d->en_handler( ev_type, ev_data, en, d->data );
	}
    spin_unlock_irqrestore(&hw_handler_list_lock, flags);
    
}

static int cobalt_acpi_translate_event( cobalt_acpi_evt *evt )
{
    struct list_head *pos;
    unsigned long flags;
    trans_table_datum *d;

    if( evt->ev_type != COBALT_ACPI_EVT_GPE )
	return 0;

    spin_lock_irqsave( &trans_table_list_lock, flags );
    list_for_each( pos, &trans_table_list )
	{
	    d = list_entry( pos, trans_table_datum, link );
	    if( d->hw_type == evt->hw_type )
	    {
		if( evt->ev_data >= d->table_len )
		    goto err_out;

		if( d->table[ evt->ev_data ] != COBALT_ACPI_EVT_NONE )
		{
		    evt->ev_type = d->table[ evt->ev_data ];
		    evt->ev_data = 0;
		}
		
		spin_unlock_irqrestore( &trans_table_list_lock, flags );
		return 0;
	    }
	}

  err_out:
    spin_unlock_irqrestore( &trans_table_list_lock, flags );
    return -1;
}

extern int cobalt_acpi_post_event( cobalt_acpi_evt evt )
{
    evt_list_datum *d;
    unsigned long flags;
    
    if( ! (d = (evt_list_datum *) kmalloc( sizeof( evt_handler_datum ), GFP_ATOMIC )) )
	return -ENOMEM;
    

    cobalt_acpi_translate_event( &evt );

    memcpy( &(d->evt), &evt, sizeof(evt) );

    spin_lock_irqsave( &dispatch_queue_lock, flags );
    list_add_tail( &(d->link), &dispatch_queue );
    spin_unlock_irqrestore( &dispatch_queue_lock, flags );

    return 0;
}

static int cobalt_acpi_apply_evt_handlers( evt_list_datum *d )
{
    struct list_head *pos;
    evt_handler_datum *evt_h;
    int ret,err = 0;
    unsigned long flags;
    
    spin_lock_irqsave( &evt_handler_list_lock, flags );
    list_for_each( pos, &evt_handler_list )
	{
	    evt_h = list_entry( pos, evt_handler_datum, link );
	    if( (! evt_h->ev_type) || (evt_h->ev_type == d->evt.ev_type) )
	    {
		if( (ret = evt_h->handler( &d->evt, evt_h->data )) < 0 )
		    err = ret;
	    }
	}
    spin_unlock_irqrestore( &evt_handler_list_lock, flags );

    return err;
}

static int cobalt_acpi_run_dispatch_queue( void )
{
    struct list_head *pos;
    int ret;
    int err=0;
    evt_list_datum *d;
    unsigned long flags;

    spin_lock_irqsave( &dispatch_queue_lock, flags );
    while( (pos = list_pop( &dispatch_queue )) )
    {
	d = list_entry( pos, evt_list_datum, link );
	if( (ret = cobalt_acpi_apply_evt_handlers( d )) < 0 )
	    err = ret;
#ifdef CONFIG_COBALT_EMU_ACPI
	cobalt_acpi_generate_proc_evt( &d->evt );
#endif
	kfree( d );
    }
    spin_unlock_irqrestore( &dispatch_queue_lock, flags );

    return err;
}

static void acpi_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
    struct list_head *pos;
    hw_handler_datum *d;
    int ret=0, err=0;
    unsigned long flags;

    spin_lock_irqsave(&hw_handler_list_lock, flags);
    list_for_each( pos, &hw_handler_list )
	{
	    d = list_entry( pos, hw_handler_datum, link );
	    if( (ret = d->hw_handler( irq, dev_id, regs, d->data )) < 0 )
		err = ret;
	    
	}
    spin_unlock_irqrestore(&hw_handler_list_lock, flags);

    if( (err = cobalt_acpi_run_dispatch_queue()) < 0 )
	err = ret;

    if( err )
	EPRINTK( "error at interrupt time of type %d.\n", err );

    return;
}




int __init cobalt_acpi_init(void)
{
    int err;

    printk(KERN_INFO "Initializing Cobalt ACPI driver.\n");
    
    if( cobt_is_monterey() )
	cobalt_acpi_monterey_init();
    else if( cobt_is_alpine() )
	cobalt_acpi_alpine_init();

    if( cobt_is_5k() )
    {
	if( pci_find_device(PCI_VENDOR_ID_SERVERWORKS, 
			    PCI_DEVICE_ID_SERVERWORKS_OSB4, NULL ) )
	{
	    if( (err = cobalt_acpi_osb4_init()) < 0 )
	    {
		goto cleanup;
	    }
	}
	
	if( pci_find_device(PCI_VENDOR_ID_SERVERWORKS, 
			    PCI_DEVICE_ID_SERVERWORKS_CSB5, NULL ) )
	{
	    if( (err = cobalt_acpi_csb5_init()) < 0 )
	    {
		goto cleanup;
	    }
	}

	switch( superio_type() )
	{
	    case SIO_TYPE_PC8731X:
		if( (err = cobalt_acpi_pc8731x_init()) )
		{
		    goto cleanup;
		}
		break;

	    case SIO_TYPE_PC8741X:
		if( (err = cobalt_acpi_pc8741x_init()) )
		{
		    goto cleanup;
		}
		break;

	    case SIO_TYPE_UNKNOWN:
		EPRINTK("unknown superio type\n");
		break;
	}
	
	    /* setup an interrupt handler for an ACPI SCI */
	err = request_irq(ACPI_IRQ, acpi_interrupt, 
			  SA_SHIRQ, ACPI_NAME, (void *)ACPI_MAGIC);
	if (err) {
	    EPRINTK("can't assign ACPI IRQ (%d)\n", ACPI_IRQ);
	    return err;
	}

#ifdef CONFIG_COBALT_EMU_ACPI
	cobalt_acpi_setup_proc();
#endif
    }

	/* enable some events we may want */
    cobalt_acpi_enable_event( COBALT_ACPI_EVT_PWRBTN, 1 );

    return 0;
    
  cleanup:
    cobalt_acpi_cleanup();
    return err;
}

static void cobalt_acpi_cleanup( void )
{
    cobalt_acpi_osb4_cleanup();
    cobalt_acpi_csb5_cleanup();
    cobalt_acpi_pc8731x_cleanup();
    cobalt_acpi_pc8741x_cleanup();

    if( cobt_is_monterey() )
	cobalt_acpi_monterey_cleanup();
    if( cobt_is_alpine() )
	cobalt_acpi_alpine_cleanup();
}

/*
 *
 * Generic ACPI HW Support
 *
 */

static __inline__ char *region_name( char * subsys_name, char * blk_name )
{
    char * new_name;
    
    if( !( new_name = (char *) kmalloc( strlen(subsys_name) + strlen(blk_name) + 14,
					GFP_ATOMIC)) )
	return NULL;

    sprintf( new_name, "%s (%s)", subsys_name, blk_name );
    return new_name;
}

static void free_region_names( generic_acpi_regions *regions )
{
    if( regions->pm1a_evt_nam )
	kfree( regions->pm1a_evt_nam );

    if( regions->pm1b_evt_nam )
	kfree( regions->pm1b_evt_nam );

    if( regions->pm1a_cnt_nam )
	kfree( regions->pm1a_cnt_nam );

    if( regions->pm1b_cnt_nam )
	kfree( regions->pm1b_cnt_nam );

    if( regions->pm2_cnt_nam )
	kfree( regions->pm2_cnt_nam );

    if( regions->pm_tmr_nam )
	kfree( regions->pm_tmr_nam );

    if( regions->p_nam )
	kfree( regions->p_nam );

    if( regions->gpe0_nam )
	kfree( regions->gpe0_nam );

    if( regions->gpe1_nam )
	kfree( regions->gpe1_nam );
}

static int register_acpi_regions( generic_acpi_regions *regions,  char * subsys_name )
{
    int i;
    
    if( regions->pm1a_evt_blk && regions->pm1_evt_len )
    {
	if( !(regions->pm1a_evt_nam = region_name( subsys_name, "pm1a_evt_blk" )) )
	    goto cleanup0;

	if( !request_region( regions->pm1a_evt_blk, regions->pm1_evt_len, 
			     regions->pm1a_evt_nam ) )
	    goto cleanup0;
    }

    if( regions->pm1b_evt_blk && regions->pm1_evt_len )
    {
	if( !(regions->pm1b_evt_nam = region_name( subsys_name, "pm1b_evt_blk" )) )
	    goto cleanup0;

	if( !request_region( regions->pm1b_evt_blk, regions->pm1_evt_len,
			     regions->pm1b_evt_nam) )
	    goto cleanup1;
    }

    if( regions->pm1a_cnt_blk && regions->pm1_cnt_len )
    {
	if( !(regions->pm1a_cnt_nam = region_name( subsys_name, "pm1a_cnt_blk" )) )
	    goto cleanup1;

	if( !request_region( regions->pm1a_cnt_blk, regions->pm1_cnt_len,
			     regions->pm1a_cnt_nam ) )
	    goto cleanup2;
    }

    if( regions->pm1b_cnt_blk && regions->pm1_cnt_len )
    {
	if( !(regions->pm1b_cnt_nam = region_name( subsys_name, "pm1b_cnt_blk" )) )
	    goto cleanup2;

	if( !request_region( regions->pm1b_cnt_blk, regions->pm1_cnt_len,
			     regions->pm1b_cnt_nam ) )
	    goto cleanup3;
    }

    if( regions->pm2_cnt_blk && regions->pm2_cnt_len )
    {
	if( !(regions->pm2_cnt_nam = region_name( subsys_name, "pm2_cnt_blk" )) )
	    goto cleanup3;

	if( !request_region( regions->pm2_cnt_blk, regions->pm2_cnt_len,
			     regions->pm2_cnt_nam ) )
	    goto cleanup4;
    }

    if( regions->pm_tmr_blk && regions->pm_tmr_len )
    {
	if( !(regions->pm_tmr_nam = region_name( subsys_name, "pm_tmp_blk" )) )
	    goto cleanup4;

	if( !request_region( regions->pm_tmr_blk, regions->pm_tmr_len,
			     regions->pm_tmr_nam ) )
	    goto cleanup5;
    }

    if( regions->p_blk )
    {
	if( !(regions->p_nam = region_name( subsys_name, "p_blk" )) )
	    goto cleanup5;

	if( !request_region( regions->p_blk, 6, regions->p_nam ) )
	    goto cleanup6;
    }

    if( regions->gpe0_blk && regions->gpe0_len )
    {
	if( !(regions->gpe0_nam = region_name( subsys_name, "gpe0_blk" )) )
	    goto cleanup6;

	if( !request_region( regions->gpe0_blk, regions->gpe0_len, 
			     regions->gpe0_nam ) )
	    goto cleanup7;
    }

    if( regions->gpe1_blk && regions->gpe1_len )
    {
	if( !(regions->gpe1_nam = region_name( subsys_name, "gpe1_blk" )) )
	    goto cleanup7;

	if( !request_region( regions->gpe1_blk, regions->gpe1_len,
			     regions->gpe1_nam ) )
	    goto cleanup8;
    }

    if( (regions->gpe_ref_cnt = (atomic_t *) kmalloc( sizeof( atomic_t ) * 
						      regions->gpe0_len * 8,
						      GFP_ATOMIC)) == NULL )
	goto cleanup9;

    memset( regions->gpe_ref_cnt, 0x0, sizeof( atomic_t ) * regions->gpe0_len * 8 );

	/* disable all events and ack them */
    if( regions->pm1a_evt_blk )
    {
	outw( 0x0000, regions->pm1a_evt_blk + regions->pm1_evt_len/2 );
	outw( 0xffff, regions->pm1a_evt_blk );
    }

    if( regions->pm1b_evt_blk )
    {
	outw( 0x0000, regions->pm1b_evt_blk + regions->pm1_evt_len/2 );
	outw( 0xffff, regions->pm1b_evt_blk );
    }

    if( regions->gpe0_blk )
    {
	for( i=0 ; i<(regions->gpe0_len/2) ; i++ )
	{
	    outb( 0x00, regions->gpe0_blk + regions->gpe0_len/2 + i );
	    outb( 0xff, regions->gpe0_blk + i );
	}
    }

    if( regions->gpe1_blk )
    {
	for( i=0 ; i<(regions->gpe1_len/2) ; i++ )
	{
	    outb( 0x00, regions->gpe1_blk + regions->gpe1_len/2 + i );
	    outb( 0xff, regions->gpe1_blk + i );
	}
    }

    return 0;
	
  cleanup9:
    if( regions->gpe1_blk )
    {
	release_region( regions->gpe1_blk, regions->gpe1_len );
	regions->gpe1_blk = 0;
    }

  cleanup8:
    if( regions->gpe0_blk )
    {
	release_region( regions->gpe0_blk, regions->gpe0_len );
	regions->gpe0_blk = 0;
    }

  cleanup7:
    if( regions->p_blk )
    {
	release_region( regions->p_blk, 6 );
	regions->p_blk = 0;
    }

  cleanup6:
    if( regions->pm_tmr_blk )
    {
	release_region( regions->pm_tmr_blk, regions->pm_tmr_len );
	regions->pm_tmr_blk = 0;
    }

  cleanup5:
    if( regions->pm2_cnt_blk )
    {
	release_region( regions->pm2_cnt_blk, regions->pm2_cnt_len );
	regions->pm2_cnt_blk = 0;
    }

  cleanup4:
    if( regions->pm1b_cnt_blk )
    {
	release_region( regions->pm1b_cnt_blk, regions->pm1_cnt_len );
	regions->pm1b_cnt_blk = 0;
    }

  cleanup3:
    if( regions->pm1a_cnt_blk )
    {
	release_region( regions->pm1a_cnt_blk, regions->pm1_cnt_len );
	regions->pm1a_cnt_blk = 0;
    }

  cleanup2:
    if( regions->pm1b_evt_blk )
    {
	release_region( regions->pm1b_evt_blk, regions->pm1_evt_len );
	regions->pm1b_evt_blk = 0;
    }

  cleanup1:
    if( regions->pm1a_evt_blk )
    {
	release_region( regions->pm1a_evt_blk, regions->pm1_evt_len );
	regions->pm1a_evt_blk = 0;
    }

  cleanup0:
    free_region_names( regions );
    
    return -EBUSY;
}

static int unregister_acpi_regions( generic_acpi_regions *regions )
{
    if( regions->pm1a_evt_blk && regions->pm1_evt_len )
    {
	release_region( regions->pm1a_evt_blk, regions->pm1_evt_len );
	regions->pm1a_evt_blk = 0;
    }
    
    if( regions->pm1b_evt_blk && regions->pm1_evt_len )
    {
	release_region( regions->pm1b_evt_blk, regions->pm1_evt_len );
	regions->pm1b_evt_blk = 0;
    }

    if( regions->pm1a_cnt_blk && regions->pm1_cnt_len )
    {
	release_region( regions->pm1a_cnt_blk, regions->pm1_cnt_len );
	regions->pm1a_cnt_blk = 0;
    }

    if( regions->pm1b_cnt_blk && regions->pm1_cnt_len )
    {
	release_region( regions->pm1b_cnt_blk, regions->pm1_cnt_len );
	regions->pm1b_cnt_blk = 0;
    }

    if( regions->pm2_cnt_blk && regions->pm2_cnt_len )
    {
	release_region( regions->pm2_cnt_blk, regions->pm2_cnt_len );
	regions->pm2_cnt_blk = 0;
    }

    if( regions->pm_tmr_blk && regions->pm_tmr_len )
    {
	release_region( regions->pm_tmr_blk, regions->pm_tmr_len );
	regions->pm_tmr_blk = 0;
    }

    if( regions->p_blk )
    {
	release_region( regions->p_blk, 6 );
	regions->p_blk = 0;
    }

    if( regions->gpe0_blk && regions->gpe0_len )
    {
	release_region( regions->gpe0_blk, regions->gpe0_len );
	regions->gpe0_blk = 0;
    }

    if( regions->gpe1_blk && regions->gpe1_len )
    {
	release_region( regions->gpe1_blk, regions->gpe1_len );
	regions->gpe1_blk = 0;
    }

    if( regions->gpe_ref_cnt )
	kfree( regions->gpe_ref_cnt );
    
    free_region_names( regions );

    return 0;
}

static void cobalt_acpi_handle_pm1_blk( u16 io_addr, u16 len, 
				       generic_acpi_regions * regions )
{
    cobalt_acpi_evt evt;
    u16 sts, en;
    
    evt.hw_type = regions->hw_type;
    
    if( (sts = inw( io_addr )) &&
	(en  = inw( io_addr + len/2 )) )
    {


	    /* clear status bits */
	outw( sts, io_addr);

	if( (en  & GEN_ACPI_TMR_STS) &&
	    (sts & GEN_ACPI_TMR_STS) )
	{
	    evt.ev_type = COBALT_ACPI_EVT_TMR;
	    evt.ev_data = 0x0;
	    cobalt_acpi_post_event( evt );
	}
	if( (en & GEN_ACPI_BM_STS) &&
	    (sts & GEN_ACPI_BM_STS) )
	{
	    evt.ev_type = COBALT_ACPI_EVT_BM;
	    evt.ev_data = 0x0;
	    cobalt_acpi_post_event( evt );
	}
	if( (en  & GEN_ACPI_GBL_STS) &&
	    (sts & GEN_ACPI_GBL_STS) )
	{
	    evt.ev_type = COBALT_ACPI_EVT_GBL;
	    evt.ev_data = 0x0;
	    cobalt_acpi_post_event( evt );
	}
	if( (en  & GEN_ACPI_PWRBTN_STS) &&
	    (sts & GEN_ACPI_PWRBTN_STS) )
	{
	    evt.ev_type = COBALT_ACPI_EVT_PWRBTN;
	    evt.ev_data = 0x0;
	    cobalt_acpi_post_event( evt );
	}
	if( (en  & GEN_ACPI_SLPBTN_STS) &&
	    (sts & GEN_ACPI_SLPBTN_STS) )
	{
	    evt.ev_type = COBALT_ACPI_EVT_SLPBTN;
	    evt.ev_data = 0x0;
	    cobalt_acpi_post_event( evt );
	}
	if( (en & GEN_ACPI_RTC_STS) &&
	    (sts & GEN_ACPI_RTC_STS) )
	{
	    evt.ev_type = COBALT_ACPI_EVT_RTC;
	    evt.ev_data = 0x0;
	    cobalt_acpi_post_event( evt );
	}
	if( (sts & GEN_ACPI_WAK_STS) )
	{
	    evt.ev_type = COBALT_ACPI_EVT_WAK;
	    evt.ev_data = 0x0;
	    cobalt_acpi_post_event( evt );
	}
    }
}

static void cobalt_acpi_handle_gpe_blk( u16 io_addr, u16 len, 
					generic_acpi_regions * regions )
{
    cobalt_acpi_evt evt;
    int i,j;
    u8 sts, en;
    
    evt.hw_type = regions->hw_type;
    evt.ev_type = COBALT_ACPI_EVT_GPE;

    for( i=0 ; i<(len/2) ; i++ )
    {
	sts = inb( io_addr + i );
	en =  inb( io_addr + len/2 + i );

	    /* clear status bits */
	outb( sts, io_addr);

	for( j=0 ; j<8 ; j++ )
	{
	    if( (en  & 0x1) &&
		(sts & 0x1) )
	    {
		evt.ev_data = i*8 + j;
		cobalt_acpi_post_event( evt );
	    }
	    en >>= 1;
	    sts >>= 1;
	}
    }
}

static int cobalt_acpi_generic_hw_handler( int irq, void *dev_id, 
					   struct pt_regs *regs, void * data )
{
    generic_acpi_regions * regions = (generic_acpi_regions *) data;
    cobalt_acpi_evt evt;

    evt.hw_type = regions->hw_type;

	/* various PM events */
    if( regions->pm1a_evt_blk )
	cobalt_acpi_handle_pm1_blk( regions->pm1a_evt_blk, regions->pm1_evt_len, regions );

    if( regions->pm1b_evt_blk )
	cobalt_acpi_handle_pm1_blk( regions->pm1b_evt_blk, regions->pm1_evt_len, regions );

    if( regions->gpe0_blk )
	cobalt_acpi_handle_gpe_blk( regions->gpe0_blk, regions->gpe0_len, regions );

    if( regions->gpe1_blk )
	cobalt_acpi_handle_gpe_blk( regions->gpe1_blk, regions->gpe1_len, regions );
	

    return 0;
}

static int cobalt_acpi_generic_en_handler( u16 ev_type, u16 ev_data, int en, void *data )
{
    generic_acpi_regions * regions = (generic_acpi_regions *) data;
    int block, offset;
    u8 data8;
    u16 data16;
    
    switch( ev_type )
    {
	case COBALT_ACPI_EVT_TMR:
	    data16 = inw( regions->pm1a_evt_blk + (regions->pm1_evt_len / 2) );
	    
	    if( en )
	    {
		data16 |= GEN_ACPI_TMR_STS;
		atomic_inc( &regions->tmr_ref_cnt );
	    }
	    else
	    {
		if( atomic_dec_and_test( &regions->tmr_ref_cnt ) )
		    data16 &= ~GEN_ACPI_TMR_STS;
	    }
	    outw( data16, regions->pm1a_evt_blk + (regions->pm1_evt_len / 2) );
	    break;
	    
	case COBALT_ACPI_EVT_BM:
	    data16 = inw( regions->pm1a_evt_blk + (regions->pm1_evt_len / 2) );
	    
	    if( en )
	    {
		data16 |= GEN_ACPI_BM_STS;
		atomic_inc( &regions->bm_ref_cnt );
	    }
	    else
	    {
		if( atomic_dec_and_test( &regions->bm_ref_cnt ) )
		    data16 &= ~GEN_ACPI_BM_STS;
	    }
	    outw( data16, regions->pm1a_evt_blk + (regions->pm1_evt_len / 2) );
	    break;
	    
	case COBALT_ACPI_EVT_GBL:
	    data16 = inw( regions->pm1a_evt_blk + (regions->pm1_evt_len / 2) );
	    
	    if( en )
	    {
		data16 |= GEN_ACPI_GBL_STS;
		atomic_inc( &regions->gbl_ref_cnt );
	    }
	    else
	    {
		if( atomic_dec_and_test( &regions->gbl_ref_cnt ) )
		    data16 &= ~GEN_ACPI_GBL_STS;
	    }
	    outw( data16, regions->pm1a_evt_blk + (regions->pm1_evt_len / 2) );
	    break;
	    
	case COBALT_ACPI_EVT_PWRBTN:
	    data16 = inw( regions->pm1a_evt_blk + (regions->pm1_evt_len / 2) );
	    
	    if( en )
	    {
		data16 |= GEN_ACPI_PWRBTN_STS;
		atomic_inc( &regions->pwrbtn_ref_cnt );
	    }
	    else
	    {
		if( atomic_dec_and_test( &regions->pwrbtn_ref_cnt ) )
		    data16 &= ~GEN_ACPI_PWRBTN_STS;
	    }
	    outw( data16, regions->pm1a_evt_blk + (regions->pm1_evt_len / 2) );
	    break;
	    
	case COBALT_ACPI_EVT_SLPBTN:
	    data16 = inw( regions->pm1a_evt_blk + (regions->pm1_evt_len / 2) );
	    
	    if( en )
	    {
		data16 |= GEN_ACPI_SLPBTN_STS;
		atomic_inc( &regions->slpbtn_ref_cnt );
	    }
	    else
	    {
		if( atomic_dec_and_test( &regions->slpbtn_ref_cnt ) )
		    data16 &= ~GEN_ACPI_SLPBTN_STS;
	    }
	    outw( data16, regions->pm1a_evt_blk + (regions->pm1_evt_len / 2) );
	    break;
	    
	case COBALT_ACPI_EVT_RTC:
	    data16 = inw( regions->pm1a_evt_blk + (regions->pm1_evt_len / 2) );
	    
	    if( en )
	    {
		data16 |= GEN_ACPI_RTC_STS;
		atomic_inc( &regions->rtc_ref_cnt );
	    }
	    else
	    {
		if( atomic_dec_and_test( &regions->rtc_ref_cnt ) )
		    data16 &= ~GEN_ACPI_RTC_STS;
	    }
	    outw( data16, regions->pm1a_evt_blk + (regions->pm1_evt_len / 2) );
	    break;
	    
	case COBALT_ACPI_EVT_WAK:
	    data16 = inw( regions->pm1a_evt_blk + (regions->pm1_evt_len / 2) );
	    
	    if( en )
	    {
		data16 |= GEN_ACPI_WAK_STS;
		atomic_inc( &regions->wak_ref_cnt );
	    }
	    else
	    {
		if( atomic_dec_and_test( &regions->wak_ref_cnt ) )
		    data16 &= ~GEN_ACPI_WAK_STS;
	    }
	    outw( data16, regions->pm1a_evt_blk + (regions->pm1_evt_len / 2) );
	    break;
	    
	case COBALT_ACPI_EVT_GPE:
	    if( (ev_data/8) >= (regions->gpe0_len / 2) )
		return -EINVAL;
	    
	    block = ev_data / 8;
	    offset = ev_data % 8;
	    
	    data8 = inb( regions->gpe0_blk + (regions->gpe0_len / 2) + block );
	    
	    if( en )
	    {
		data8 |= 0x1 << offset;
		atomic_inc( &regions->gpe_ref_cnt[ev_data] );
	    }
	    else
	    {
		if( atomic_dec_and_test( &regions->gpe_ref_cnt[ev_data] ) )
		    data8 &= ~( 0x1 << offset );
	    }
	    
	    outb( data8, regions->gpe0_blk + (regions->gpe0_len / 2) + block );
	    
	    break;
	    
	default:
	    return -EINVAL;
		    
    }

    return 0;
}

/*
 *
 * Generic ServerWorks region code
 *
 */

static int get_serverworks_regions( generic_acpi_regions *regions, u16 type )
{
   int reg;
    
    memset( regions, 0x0, sizeof( *regions ) );
    
    regions->hw_type = type;

    regions->pm1_evt_len = 4;
    regions->pm1_cnt_len = 2;
    regions->pm_tmr_len = 4;
    regions->gpe0_len = 8;

    if( (reg = get_reg(OSB4_INDEX_PORT, OSB4_DATA_PORT, 0x20)) )
    	regions->pm1a_evt_blk = (u16) reg;
    
    if( (reg = get_reg(OSB4_INDEX_PORT, OSB4_DATA_PORT, 0x22)) )
	regions->pm1a_cnt_blk = (u16) reg;
    
    if( (reg = get_reg(OSB4_INDEX_PORT, OSB4_DATA_PORT, 0x24)) )
	regions->pm_tmr_blk = (u16) reg;
    
    if( (reg = get_reg(OSB4_INDEX_PORT, OSB4_DATA_PORT, 0x26)) )
	regions->p_blk = (u16) reg;
    
    if( (reg = get_reg(OSB4_INDEX_PORT, OSB4_DATA_PORT, 0x28)) )
	regions->gpe0_blk = (u16) reg;

    if( type == COBALT_ACPI_HW_OSB4 )
    {
	regions->pm2_cnt_len = 1;
	if( (reg = get_reg(OSB4_INDEX_PORT, OSB4_DATA_PORT, 0x2E)) )
	    regions->pm2_cnt_blk = (u16) reg;
    }

    switch( type )
    {
	case COBALT_ACPI_HW_OSB4:
	    return register_acpi_regions( regions, "OSB4" );

	case COBALT_ACPI_HW_CSB5:
	    return register_acpi_regions( regions, "CSB5" );
    }
    
    return -EINVAL;

}

/*
 *
 * ServerWorks OSB4
 *
 */

static generic_acpi_regions osb4_regions;

static int cobalt_acpi_osb4_init( void )
{
    int err;
    
    if( (err = get_osb4_regions( &osb4_regions )) < 0 )
	return err;

    if( (err = cobalt_acpi_register_hw_handler( COBALT_ACPI_HW_OSB4,
						cobalt_acpi_generic_hw_handler, 
						cobalt_acpi_generic_en_handler, 
						&osb4_regions )) < 0 )
	return err;

    return 0;
}

static int cobalt_acpi_osb4_cleanup( void )
{
    unregister_acpi_regions( &osb4_regions );
    return 0;
}

static int get_osb4_regions( generic_acpi_regions *regions)
{
    return get_serverworks_regions( regions, COBALT_ACPI_HW_OSB4 );
}

/*
 *
 * ServerWorks CSB5
 *
 */

/* static generic_acpi_regions csb5_regions; */

static generic_acpi_regions csb5_regions;

static int cobalt_acpi_csb5_init( void )
{
    int err;
    
    if( (err = get_csb5_regions( &csb5_regions )) < 0 )
	return err;

    if( (err = cobalt_acpi_register_hw_handler( COBALT_ACPI_HW_CSB5,
						cobalt_acpi_generic_hw_handler, 
						cobalt_acpi_generic_en_handler, 
						&csb5_regions )) < 0 )
	return err;

    return 0;
}

static int cobalt_acpi_csb5_cleanup( void )
{
    unregister_acpi_regions( &csb5_regions );
    return 0;
}

static int get_csb5_regions( generic_acpi_regions *regions)
{
    return get_serverworks_regions( regions, COBALT_ACPI_HW_CSB5 );
}

/*
 *
 * NatSemi PC8731x
 *
 */
static generic_acpi_regions pc8731x_regions;

static int cobalt_acpi_pc8731x_init( void )
{
    int err;
    
    if( (err = get_pc8731x_regions( &pc8731x_regions )) < 0 )
	return err;

    if( (err = cobalt_acpi_register_hw_handler( COBALT_ACPI_HW_PC8731X,
						cobalt_acpi_generic_hw_handler, 
						cobalt_acpi_generic_en_handler, 
						&pc8731x_regions )) < 0 )
	return err;

    return 0;
}

static int cobalt_acpi_pc8731x_cleanup( void )
{
    unregister_acpi_regions( &pc8731x_regions );
    return 0;
}

static int get_pc8731x_regions( generic_acpi_regions *regions )
{
    int reg;
    u16 addr;
    
    memset( regions, 0x0, sizeof( *regions ) );

    regions->hw_type = COBALT_ACPI_HW_PC8731X;

    regions->pm1_evt_len = 4;
    regions->pm1_cnt_len = 2;
    regions->pm_tmr_len = 4;
    regions->gpe0_len = 4;
    
	/* superi/o -- select pm logical device and get base address */
    addr = superio_ldev_base(PC87317_DEV_PM);
    if( addr )
    {
	    /* get registers */
	if( (reg = get_reg(addr, addr + 1, 0x08)) )
	    regions->pm1a_evt_blk = reg;

	if( (reg = get_reg(addr, addr + 1, 0x0a)) )
	    regions->pm_tmr_blk = reg;

	if( (reg = get_reg(addr, addr + 1, 0x0c)) )
	    regions->pm1a_cnt_blk = reg;

	if( (reg = get_reg(addr, addr + 1, 0x0e)) )
	    regions->gpe0_blk = reg;
    }
    
    return register_acpi_regions( regions, "pc8731x" );
}

/*
 *
 * NatSemi PC8741x
 *
 */

static generic_acpi_regions pc8741x_regions;

static int cobalt_acpi_pc8741x_init( void )
{
    int err;
    
    if( (err = get_pc8741x_regions( &pc8741x_regions )) < 0 )
	return err;

    if( (err = cobalt_acpi_register_hw_handler( COBALT_ACPI_HW_PC8741X,
						cobalt_acpi_generic_hw_handler, 
						cobalt_acpi_generic_en_handler, 
						&pc8741x_regions )) < 0 )
	return err;

    return 0;
}

static int cobalt_acpi_pc8741x_cleanup( void )
{
    unregister_acpi_regions( &pc8741x_regions );
    return 0;
}

static int get_pc8741x_regions( generic_acpi_regions *regions )
{
    int reg;
    
    memset( regions, 0x0, sizeof( *regions ) );

    regions->hw_type = COBALT_ACPI_HW_PC8741X;

    regions->pm1_evt_len = 4;
    regions->pm1_cnt_len = 2;
    regions->pm_tmr_len = 4;
    regions->gpe0_len = 8;
    
	/* get registers */
    if( (reg = superio_ldev_base_n(PC87417_DEV_SWC, 1)) )
	regions->pm1a_evt_blk = reg;

    if( (reg = superio_ldev_base_n(PC87417_DEV_SWC, 2)) )
	regions->pm1a_cnt_blk = reg;

    if( (reg = superio_ldev_base_n(PC87417_DEV_SWC, 3)) )
	regions->gpe0_blk = reg;

    return register_acpi_regions( regions, "pc8741x" );
}

/*
 *
 * Platform support
 * 
 */

/*
 *
 * Monterey
 *
 */

static u16 cobalt_acpi_monterey_osb4_table[] = {
/* GPE  0 */ COBALT_ACPI_EVT_NONE,
/* GPE  1 */ COBALT_ACPI_EVT_NONE,
/* GPE  2 */ COBALT_ACPI_EVT_NONE,
/* GPE  3 */ COBALT_ACPI_EVT_NONE,
/* GPE  4 */ COBALT_ACPI_EVT_NONE,
/* GPE  5 */ COBALT_ACPI_EVT_NONE,
/* GPE  6 */ COBALT_ACPI_EVT_SLED,
/* GPE  7 */ COBALT_ACPI_EVT_NONE,
/* GPE  8 */ COBALT_ACPI_EVT_NONE,
/* GPE  9 */ COBALT_ACPI_EVT_NONE,
/* GPE 10 */ COBALT_ACPI_EVT_NONE,
/* GPE 11 */ COBALT_ACPI_EVT_NONE,
/* GPE 12 */ COBALT_ACPI_EVT_NONE,
/* GPE 13 */ COBALT_ACPI_EVT_NONE,
/* GPE 14 */ COBALT_ACPI_EVT_NONE,
/* GPE 15 */ COBALT_ACPI_EVT_NONE };

static u16 cobalt_acpi_monterey_superio_table[] = {
/* GPE  0 */ COBALT_ACPI_EVT_NONE,
/* GPE  1 */ COBALT_ACPI_EVT_NONE,
/* GPE  2 */ COBALT_ACPI_EVT_NONE,
/* GPE  3 */ COBALT_ACPI_EVT_NONE,
/* GPE  4 */ COBALT_ACPI_EVT_NONE,
/* GPE  5 */ COBALT_ACPI_EVT_NONE,
/* GPE  6 */ COBALT_ACPI_EVT_NONE,
/* GPE  7 */ COBALT_ACPI_EVT_NONE,
/* GPE  8 */ COBALT_ACPI_EVT_NONE,
/* GPE  9 */ COBALT_ACPI_EVT_NONE,
/* GPE 10 */ COBALT_ACPI_EVT_NONE,
/* GPE 11 */ COBALT_ACPI_EVT_NONE,
/* GPE 12 */ COBALT_ACPI_EVT_NONE,
/* GPE 13 */ COBALT_ACPI_EVT_NONE,
/* GPE 14 */ COBALT_ACPI_EVT_NONE,
/* GPE 15 */ COBALT_ACPI_EVT_NONE,
/* GPE 16 */ COBALT_ACPI_EVT_NONE,
/* GPE 17 */ COBALT_ACPI_EVT_NONE,
/* GPE 18 */ COBALT_ACPI_EVT_NONE,
/* GPE 19 */ COBALT_ACPI_EVT_NONE,
/* GPE 20 */ COBALT_ACPI_EVT_NONE,
/* GPE 21 */ COBALT_ACPI_EVT_NONE,
/* GPE 22 */ COBALT_ACPI_EVT_NONE,
/* GPE 23 */ COBALT_ACPI_EVT_NONE,
/* GPE 24 */ COBALT_ACPI_EVT_NONE,
/* GPE 25 */ COBALT_ACPI_EVT_NONE,
/* GPE 26 */ COBALT_ACPI_EVT_NONE,
/* GPE 27 */ COBALT_ACPI_EVT_NONE,
/* GPE 28 */ COBALT_ACPI_EVT_NONE,
/* GPE 29 */ COBALT_ACPI_EVT_NONE,
/* GPE 30 */ COBALT_ACPI_EVT_NONE,
/* GPE 31 */ COBALT_ACPI_EVT_NONE };

static int cobalt_acpi_monterey_init( void )
{
    int err;
    
    err = cobalt_acpi_register_trans_table( COBALT_ACPI_HW_OSB4, 
					    sizeof( cobalt_acpi_monterey_osb4_table )/sizeof( u16 ),
					    cobalt_acpi_monterey_osb4_table );			      
    if( err < 0 )
	return err;

    err = cobalt_acpi_register_trans_table( COBALT_ACPI_HW_PC8731X, 
					    sizeof( cobalt_acpi_monterey_superio_table )/sizeof( u16 ),
					    cobalt_acpi_monterey_superio_table );
    if( err < 0 )
	return err;

    return 0;
}

static int cobalt_acpi_monterey_cleanup( void )
{
    cobalt_acpi_unregister_trans_table( COBALT_ACPI_HW_OSB4 );
    cobalt_acpi_unregister_trans_table( COBALT_ACPI_HW_PC8731X );

    return 0;
}

/*
 *
 * Alpine
 *
 */

static u16 cobalt_acpi_alpine_csb5_table[] = {
/* GPE  0 */ COBALT_ACPI_EVT_NONE,
/* GPE  1 */ COBALT_ACPI_EVT_NONE,
/* GPE  2 */ COBALT_ACPI_EVT_NONE,
/* GPE  3 */ COBALT_ACPI_EVT_FAN,
/* GPE  4 */ COBALT_ACPI_EVT_NONE,
/* GPE  5 */ COBALT_ACPI_EVT_SM_INT,
/* GPE  6 */ COBALT_ACPI_EVT_THERM,
/* GPE  7 */ COBALT_ACPI_EVT_NONE,
/* GPE  8 */ COBALT_ACPI_EVT_NONE,
/* GPE  9 */ COBALT_ACPI_EVT_NONE,
/* GPE 10 */ COBALT_ACPI_EVT_NONE,
/* GPE 11 */ COBALT_ACPI_EVT_NONE,
/* GPE 12 */ COBALT_ACPI_EVT_NONE,
/* GPE 13 */ COBALT_ACPI_EVT_NONE,
/* GPE 14 */ COBALT_ACPI_EVT_NONE,
/* GPE 15 */ COBALT_ACPI_EVT_NONE };

static u16 cobalt_acpi_alpine_superio_table[] = {
/* GPE  0 */ COBALT_ACPI_EVT_NONE,
/* GPE  1 */ COBALT_ACPI_EVT_NONE,
/* GPE  2 */ COBALT_ACPI_EVT_NONE,
/* GPE  3 */ COBALT_ACPI_EVT_NONE,
/* GPE  4 */ COBALT_ACPI_EVT_NONE,
/* GPE  5 */ COBALT_ACPI_EVT_NONE,
/* GPE  6 */ COBALT_ACPI_EVT_NONE,
/* GPE  7 */ COBALT_ACPI_EVT_NONE,
/* GPE  8 */ COBALT_ACPI_EVT_NONE,
/* GPE  9 */ COBALT_ACPI_EVT_NONE,
/* GPE 10 */ COBALT_ACPI_EVT_NONE,
/* GPE 11 */ COBALT_ACPI_EVT_NONE,
/* GPE 12 */ COBALT_ACPI_EVT_NONE,
/* GPE 13 */ COBALT_ACPI_EVT_NONE,
/* GPE 14 */ COBALT_ACPI_EVT_NONE,
/* GPE 15 */ COBALT_ACPI_EVT_NONE,
/* GPE 16 */ COBALT_ACPI_EVT_NONE,
/* GPE 17 */ COBALT_ACPI_EVT_NONE,
/* GPE 18 */ COBALT_ACPI_EVT_NONE,
/* GPE 19 */ COBALT_ACPI_EVT_NONE,
/* GPE 20 */ COBALT_ACPI_EVT_NONE,
/* GPE 21 */ COBALT_ACPI_EVT_NONE,
/* GPE 22 */ COBALT_ACPI_EVT_NONE,
/* GPE 23 */ COBALT_ACPI_EVT_NONE,
/* GPE 24 */ COBALT_ACPI_EVT_NONE,
/* GPE 25 */ COBALT_ACPI_EVT_NONE,
/* GPE 26 */ COBALT_ACPI_EVT_NONE,
/* GPE 27 */ COBALT_ACPI_EVT_NONE,
/* GPE 28 */ COBALT_ACPI_EVT_NONE,
/* GPE 29 */ COBALT_ACPI_EVT_NONE,
/* GPE 30 */ COBALT_ACPI_EVT_NONE,
/* GPE 31 */ COBALT_ACPI_EVT_NONE };

static int cobalt_acpi_alpine_init( void )
{
    int err;
    
    err = cobalt_acpi_register_trans_table( COBALT_ACPI_HW_CSB5, 
					    sizeof( cobalt_acpi_alpine_csb5_table )/sizeof( u16 ),
					    cobalt_acpi_alpine_csb5_table );			      
    if( err < 0 )
	return err;

    err = cobalt_acpi_register_trans_table( COBALT_ACPI_HW_PC8741X, 
					    sizeof( cobalt_acpi_alpine_superio_table )/sizeof( u16 ),
					    cobalt_acpi_alpine_superio_table );
    if( err < 0 )
	return err;

    return 0;
}

static int cobalt_acpi_alpine_cleanup( void )
{
    cobalt_acpi_unregister_trans_table( COBALT_ACPI_HW_CSB5 );
    cobalt_acpi_unregister_trans_table( COBALT_ACPI_HW_PC8741X );

    return 0;
}

/*
 * end platform support
 */
#ifdef CONFIG_COBALT_EMU_ACPI
/*
 * This is all necessary because we don't have BIOS support for ACPI yet.
 * We can fake it here, and when full support is ready just yank this.
 */
typedef struct {
	char *device_type;
	char *device_instance;
	u32 event_type;
	u32 event_data;
	struct list_head list;
} cobalt_acpi_event_t;

#define COBALT_ACPI_MAX_STRING_LENGTH	80

static LIST_HEAD(cobalt_acpi_event_list);
static DECLARE_WAIT_QUEUE_HEAD(cobalt_acpi_event_wait_queue);
static int event_is_open = 0;
static spinlock_t cobalt_acpi_event_lock = SPIN_LOCK_UNLOCKED;

static struct proc_dir_entry *cobalt_acpi_proc_root;
static struct proc_dir_entry *cobalt_acpi_proc_event;

static struct file_operations proc_event_ops = {
	open: cobalt_acpi_open_event,
	read: cobalt_acpi_read_event,
	release: cobalt_acpi_close_event,
	poll: cobalt_acpi_poll_event,
};

static int
cobalt_acpi_setup_proc(void)
{
	cobalt_acpi_proc_root = proc_mkdir("acpi", NULL);
	if (!cobalt_acpi_proc_root) {
		return -ENOMEM;
	}

	cobalt_acpi_proc_event = create_proc_entry("event", S_IRUSR, 
						cobalt_acpi_proc_root);
	if (!cobalt_acpi_proc_event) {
		return -ENOMEM;
	}

	cobalt_acpi_proc_event->proc_fops = &proc_event_ops;
 
	return 0;
}


int
cobalt_acpi_generate_proc_evt( cobalt_acpi_evt * evt )
{
	cobalt_acpi_event_t *event = NULL, *tmp;
	u32 flags = 0;
	char *dev_type;
	char *dev_instance;
	u32 event_type;
	u32 event_data;
	struct list_head *pos;

	/* drop event on the floor if no one's listening */
	if (!event_is_open)
		return 0;
	
	event_type = (evt->ev_type << 0x10) |
	    evt->hw_type;
	event_data = evt->ev_data;


	    /*
	     * Check to see if an event of this type is already
	     * pending
	     */
	spin_lock_irqsave(&cobalt_acpi_event_lock, flags);
	list_for_each( pos, &cobalt_acpi_event_list )
	    {
		tmp = list_entry(pos, cobalt_acpi_event_t, list);
		if( (tmp->event_type == event_type) &&
		    (tmp->event_data == event_data) )
		{
		    spin_unlock_irqrestore(&cobalt_acpi_event_lock, flags);
		    return 0;
		}


	    }
	spin_unlock_irqrestore(&cobalt_acpi_event_lock, flags);


	    /* parse the event struct */
	switch( evt->ev_type )
	{
	    case COBALT_ACPI_EVT_TMR:
		dev_type = "generic";
		dev_instance = "timer";
		break;

	    case COBALT_ACPI_EVT_BM:
		dev_type = "generic";
		dev_instance = "bus-master";
		break;
		
	    case COBALT_ACPI_EVT_GBL:
		dev_type = "generic";
		dev_instance = "global";
		break;	    
		
	    case COBALT_ACPI_EVT_PWRBTN:
		dev_type = "button";
		dev_instance = "power";
		break;	    

	    case COBALT_ACPI_EVT_SLPBTN:
		dev_type = "button";
		dev_instance = "sleep";
		break;	    
		
	    case COBALT_ACPI_EVT_RTC:
		dev_type = "generic";
		dev_instance = "rtc";
		break;	    

	    case COBALT_ACPI_EVT_WAK:
		dev_type = "generic";
		dev_instance = "wake";
		break;	    

	    case COBALT_ACPI_EVT_GPE:
		dev_type = "generic";
		dev_instance = "gpe";
		break;	    

	    case COBALT_ACPI_EVT_SLED:
		dev_type = "cobalt";
		dev_instance = "sled";
		break;	    
		
	    case COBALT_ACPI_EVT_THERM:
		dev_type = "cobalt";
		dev_instance = "therm_trip";
		break;

	    case COBALT_ACPI_EVT_FAN:
		dev_type = "cobalt";
		dev_instance = "fan";
		break;

	    case COBALT_ACPI_EVT_SM_INT:
		dev_type = "cobalt";
		dev_instance = "sm_int";
		break;

	    case COBALT_ACPI_EVT_VOLT:
		dev_type = "cobalt";
		dev_instance = "volt_trip";
		break;
		
	    default:
		dev_type = "unknown";
		dev_instance = "unknown";
		break;
	}
	      
		
	/*
	 * Allocate a new event structure.
	 */
	event = kmalloc(sizeof(*event), GFP_ATOMIC);
	if (!event)
		goto alloc_error;

	event->device_type=NULL;
	event->device_instance=NULL;

	event->device_type = kmalloc(strlen(dev_type) + sizeof(char), 
		GFP_ATOMIC);
	if (!event->device_type)
		goto alloc_error;

	event->device_instance = kmalloc(strlen(dev_instance) + sizeof(char), 
		GFP_ATOMIC );
	if (!event->device_instance)
		goto alloc_error;

	/*
	 * Set event data.
	 */
	strcpy(event->device_type, dev_type);
	strcpy(event->device_instance, dev_instance);
	event->event_type = event_type;
	event->event_data = event_data;

	/*
	 * Add to the end of our event list.
	 */
	spin_lock_irqsave(&cobalt_acpi_event_lock, flags);
	list_add_tail(&event->list, &cobalt_acpi_event_list);
	spin_unlock_irqrestore(&cobalt_acpi_event_lock, flags);

	/*
	 * Signal waiting threads (if any).
	 */
	wake_up_interruptible(&cobalt_acpi_event_wait_queue);

	return 0;

alloc_error:
	if(event)
	{
	    if (event->device_instance)
		kfree(event->device_instance);
	    
	    if (event->device_type)
		kfree(event->device_type);
	    
	    kfree(event);		
	}

	return -ENOMEM;
}


static int 
cobalt_acpi_open_event(struct inode *inode, struct file *file)
{
    int flags;
    spin_lock_irqsave(&cobalt_acpi_event_lock, flags);

    if (event_is_open)
	goto out_busy;

    event_is_open = 1;
    
    spin_unlock_irqrestore(&cobalt_acpi_event_lock, flags);
    return 0;

out_busy:
    spin_unlock_irqrestore(&cobalt_acpi_event_lock, flags);
    return -EBUSY;
}


static int 
cobalt_acpi_close_event(struct inode *inode, struct file *file)
{
    unsigned long flags;
    struct list_head *pos;
    cobalt_acpi_event_t *tmp;
    
    spin_lock_irqsave(&cobalt_acpi_event_lock, flags);

    while( (pos = list_pop( &cobalt_acpi_event_list )) )
    {
	tmp = list_entry(pos, cobalt_acpi_event_t, list);
	if (tmp->device_instance)
		kfree(tmp->device_instance);
	    
	if (tmp->device_type)
		kfree(tmp->device_type);
	
	kfree( tmp );
    }
    event_is_open = 0;
    spin_unlock_irqrestore(&cobalt_acpi_event_lock, flags);
    return 0;
}

#define ACPI_MAX_STRING_LENGTH		80
static ssize_t
cobalt_acpi_read_event(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	cobalt_acpi_event_t *event = NULL;
	unsigned long flags = 0;
	static char str[ACPI_MAX_STRING_LENGTH];
	static int strsize;
	static char *ptr;

	if (!strsize) {
		DECLARE_WAITQUEUE(wait, current);

		if (list_empty(&cobalt_acpi_event_list)) {
			if (file->f_flags & O_NONBLOCK) {
				return -EAGAIN;
			}
			set_current_state(TASK_INTERRUPTIBLE);
			add_wait_queue(&cobalt_acpi_event_wait_queue, &wait);
	
			if (list_empty(&cobalt_acpi_event_list)) {
				schedule();
			}
	
			remove_wait_queue(&cobalt_acpi_event_wait_queue, &wait);
			set_current_state(TASK_RUNNING);
	 
			if (signal_pending(current)) {
				return -ERESTARTSYS;
			}
		}

		spin_lock_irqsave(&cobalt_acpi_event_lock, flags);
		event = list_entry(cobalt_acpi_event_list.next, 
			cobalt_acpi_event_t, list);
		list_del(&event->list);
		spin_unlock_irqrestore(&cobalt_acpi_event_lock, flags);

		strsize = sprintf(str, "%s %s %08x %08x\n",
			event->device_type, event->device_instance,
			event->event_type, event->event_data);
		ptr = str;

		kfree(event->device_type);
		kfree(event->device_instance);
		kfree(event);
	}
	if (strsize < count)
		count = strsize;

	if (copy_to_user(buf, ptr, count))
		return -EFAULT;

	*ppos += count;
	strsize -= count;
	ptr += count;

	return count;
}

static unsigned int 
cobalt_acpi_poll_event(struct file *file, poll_table *wait)
{
	poll_wait(file, &cobalt_acpi_event_wait_queue, wait);
	if (!list_empty(&cobalt_acpi_event_list))
		return POLLIN | POLLRDNORM;
	return 0;
}

#endif /* CONFIG_COBALT_EMU_ACPI */
