/*
 * $Id: cobalt-systype.h,v 1.8 2002/08/08 22:46:50 carls Exp $
 * cobalt-systype.h : figure out what Cobalt system we are on
 *
 * Copyright 2000 Cobalt Networks, Inc.
 * Copyright 2001-2002 Sun Microsystems, Inc.
 */
#ifndef COBALT_SYSTYPE_H
#define COBALT_SYSTYPE_H

#include <cobalt/cobalt.h>

typedef enum {
	COBT_UNINITIALIZED,
	COBT_UNKNOWN,
	COBT_PACIFICA,
	COBT_CARMEL,
	COBT_MONTEREY,
	COBT_ALPINE,
	COBT_BIGBEAR,
} cobt_sys_t;

extern cobt_sys_t cobt_type;
extern cobt_sys_t cobalt_systype_probe(void);
extern unsigned long cobt_rev;

/* 
 * one test for each major board-type 
 * COBT_SUPPORT_* from cobalt.h
 */

/* pacifica is the RaQ 3 and RaQ 4 platform */
static inline int
cobt_is_pacifica(void)
{
	if (!COBT_SUPPORT_GEN_III) {
		return 0;
	}
	if (cobt_type == COBT_UNINITIALIZED) {
		cobalt_systype_probe();
	}
	return (cobt_type == COBT_PACIFICA);
}

/* carmel is the Qube 3 platform */
static inline int
cobt_is_carmel(void)
{
	if (!COBT_SUPPORT_GEN_III) {
		return 0;
	}
	if (cobt_type == COBT_UNINITIALIZED) {
		cobalt_systype_probe();
	}
	return (cobt_type == COBT_CARMEL);
}

/* monterey is the RaQ XTR platform */
static inline int
cobt_is_monterey(void)
{
	if (!COBT_SUPPORT_GEN_V) {
		return 0;
	}
	if (cobt_type == COBT_UNINITIALIZED) {
		cobalt_systype_probe();
	}
	return (cobt_type == COBT_MONTEREY);
}

static inline int
cobt_is_alpine(void)
{
	if (!COBT_SUPPORT_GEN_V) {
		return 0;
	}
	if (cobt_type == COBT_UNINITIALIZED) {
		cobalt_systype_probe();
	}
	return (cobt_type == COBT_ALPINE);
}

static inline int
cobt_is_bigbear(void)
{
	if (!COBT_SUPPORT_GEN_V) {
		return 0;
	}
	if (cobt_type == COBT_UNINITIALIZED) {
		cobalt_systype_probe();
	}
	return (cobt_type == COBT_BIGBEAR);
}

/* one for each major generation */
#define cobt_is_3k()	 (cobt_is_pacifica() || cobt_is_carmel())
#define cobt_is_5k()	 (cobt_is_monterey() || cobt_is_alpine() || cobt_is_bigbear())

#endif
