/* $Id: cobalt-wdt.h,v 1.1 2001/03/07 01:58:24 thockin Exp $ */
/* Copyright 2001 Sun Microsystems, Inc. */
#ifndef COBALT_WDT_H
#define COBALT_WDT_H

#include <cobalt/cobalt.h>

void cobalt_wdt_refresh(unsigned long refresh_timer);
void cobalt_wdt_trigger_reboot(void);

void cobalt_wdt_disable(void);
void cobalt_wdt_reenable(void);

void cobalt_wdt_cleardog(void);

#endif
