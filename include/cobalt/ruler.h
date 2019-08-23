/* $Id: cobalt-ruler.h,v 1.4 2001/06/08 20:46:44 thockin Exp $ */
/* Copyright 2001 Sun Microsystems, Inc. */
#ifndef COBALT_RULER_H
#define COBALT_RULER_H

#include <linux/ide.h>

void cobalt_ruler_register(ide_hwif_t *hwif);
void cobalt_ruler_unregister(ide_hwif_t *hwif);

#endif
