/*
 * $Id: cobalt-serialnum.h,v 1.1 2001/03/07 01:58:24 thockin Exp $
 * cobalt-serialnum.h : access to the DS2401 serial number
 *
 * Copyright 2000 Cobalt Networks, Inc.
 * Copyright 2001 Sun Microsystems, Inc.
 */
#ifndef COBALT_SERIALNUM_H
#define COBALT_SERIALNUM_H

#include <cobalt/cobalt.h>

char *cobalt_serialnum_get(void);
unsigned long cobalt_hostid_get(void);

#endif /* COBALT_SERIALNUM_H */
