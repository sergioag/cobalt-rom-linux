/* $Id: cobalt-net.h,v 1.3 2002/10/25 01:02:42 thockin Exp $ */
/* Copyright 2001 Sun Microsystems, Inc. */
#ifndef COBALT_NET_H
#define COBALT_NET_H

#include <linux/netdevice.h>
#include <linux/config.h>

void cobalt_net_register(struct net_device *ndev);
void cobalt_net_unregister(struct net_device *ndev);

#endif
