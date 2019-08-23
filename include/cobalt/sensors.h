/* $Id: cobalt-sensors.h,v 1.2 2001/09/25 18:10:29 thockin Exp $ */
/* Copyright 2001 Sun Microsystems, Inc. */
#ifndef COBALT_SENSORS_H
#define COBALT_SENSORS_H

#include <cobalt/cobalt.h>

extern unsigned int cobalt_nthermals;
extern unsigned int cobalt_nvoltages;

/* return NULL if the sensor doesn't exist, fill buf if it does */
char *__cobalt_thermal_read(unsigned int sensor, char *buf, int len);
char *__cobalt_voltage_read(unsigned int sensor, char *buf, int len);

static inline char *
cobalt_thermal_read(unsigned int sensor)
{
	char buf[32];
	return __cobalt_thermal_read(sensor, buf, sizeof(buf)-1);
}

static inline char *
cobalt_voltage_read(unsigned int sensor)
{
	char buf[32];
	return __cobalt_voltage_read(sensor, buf, sizeof(buf)-1);
}

#endif
