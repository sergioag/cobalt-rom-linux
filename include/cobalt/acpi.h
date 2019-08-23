/*
 * $Id: cobalt-acpi.h,v 1.7 2001/10/27 07:03:31 erik Exp $
 * cobalt-acpi.h : support for ACPI interrupts
 *
 * Copyright 2000 Cobalt Networks, Inc.
 * Copyright 2001 Sun Microsystems, Inc.
 */
#ifndef COBALT_ACPI_H
#define COBALT_ACPI_H

#define SERVERWORKS_ACPI_INDEX_PORT	0x0cd6
#define SERVERWORKS_ACPI_DATA_PORT	0x0cd7


typedef struct
{
    u16 hw_type;
    u16 ev_type;
    u32 ev_data;
} cobalt_acpi_evt;

enum __cobalt_acpi_hw_types
{
    COBALT_ACPI_HW_ANY = 0x0000,
    COBALT_ACPI_HW_OSB4 = 0x0001,
    COBALT_ACPI_HW_CSB5 = 0x0002,
    COBALT_ACPI_HW_PC8731X = 0x0003,
    COBALT_ACPI_HW_PC8741X = 0x0004,
};

enum __cobalt_acpi_event_types
{
    COBALT_ACPI_EVT_NONE = 0x0000,
    COBALT_ACPI_EVT_TMR = 0x0001, /* Timer Event */
    COBALT_ACPI_EVT_BM = 0x00002, /* Bus Master Event */
    COBALT_ACPI_EVT_GBL = 0x0003, /* BIOS Global Lock release */
    COBALT_ACPI_EVT_PWRBTN = 0x0004, /* Power Button press */
    COBALT_ACPI_EVT_SLPBTN = 0x0005, /* Sleep Button press */
    COBALT_ACPI_EVT_RTC = 0x0006, /* RTC Alarm */
    COBALT_ACPI_EVT_WAK = 0x0007, /* Wake event */
    COBALT_ACPI_EVT_GPE = 0x0008, /* General Purpose Event (ev_data = gpe number) */

	/* events greater than 0x7fff are symbolic events */
    COBALT_ACPI_EVT_SLED = 0x8000, /* Sled removal */
    COBALT_ACPI_EVT_THERM = 0x8001, /* Thermal trip */
    COBALT_ACPI_EVT_FAN = 0x8002, /* Fan Down */
    COBALT_ACPI_EVT_SM_INT = 0x8003, /* System Monitor Interrupt */
    COBALT_ACPI_EVT_VOLT = 0x8004, /* System Monitor Interrupt */
    
};

typedef int (* cobalt_acpi_hw_handler)( int irq, void *dev_id, struct pt_regs *regs, void * data );
typedef int (* cobalt_acpi_enable_handler)( u16 ev_type, u16 ev_data, int en, void *data );
typedef int (* cobalt_acpi_evt_handler)( cobalt_acpi_evt *evt, void * data );


extern int cobalt_acpi_register_hw_handler( u16 hw_type,
					    cobalt_acpi_hw_handler hw_handler, 
					    cobalt_acpi_enable_handler en_handler,
					    void *data );
extern int cobalt_acpi_unregister_hw_handler( cobalt_acpi_hw_handler handler );

extern int cobalt_acpi_register_trans_table( u16 hw_type, u16 table_len, u16 *table ); 
extern int cobalt_acpi_unregister_trans_table( u16 hw_type );

extern int cobalt_acpi_register_evt_handler( cobalt_acpi_evt_handler handler, 
					     u16 evt_type,
					     void *data );
extern int cobalt_acpi_unregister_evt_handler( cobalt_acpi_evt_handler handler );

extern int cobalt_acpi_post_event( cobalt_acpi_evt evt );

#ifdef CONFIG_COBALT_EMU_ACPI
int cobalt_acpi_generate_proc_evt( cobalt_acpi_evt * evt );
#else
#define cobalt_acpi_generate_proc_evt( a )
#endif

    


#endif /* COBALT_ACPI_H */
