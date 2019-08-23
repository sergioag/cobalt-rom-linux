/* $Id: cobalt-misc.h,v 1.1 2001/04/04 03:36:43 thockin Exp $ */
/* Copyright 2001 Sun Microsystems, Inc. */
#ifndef COBALT_MISC_H
#define COBALT_MISC_H

void cobalt_nmi(unsigned char reason, struct pt_regs *regs);
void cobalt_restart(void);
void cobalt_halt(void);
void cobalt_power_off(void);

#endif
