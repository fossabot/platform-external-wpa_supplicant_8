/*
 * ACS - Automatic Channel Selection module
 * Copyright (c) 2011, Atheros Communications
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Alternatively, this software may be distributed under the terms of BSD
 * license.
 *
 * See README and COPYING for more details.
 */

#ifndef ACS_H
#define ACS_H

#include "utils/common.h"
#include "ap/hostapd.h"
#include "list.h"

#ifdef CONFIG_ACS

enum hostapd_chan_status acs_init(struct hostapd_iface *iface);
int hostapd_acs_completed(struct hostapd_iface *iface);

#else /* CONFIG_ACS */

static inline enum hostapd_chan_status acs_init(struct hostapd_iface *iface)
{
	wpa_printf(MSG_ERROR, "ACS was disabled on your build, "
		   "rebuild hostapd with CONFIG_ACS=y or set channel");
	return HOSTAPD_CHAN_INVALID;
}

#endif /* CONFIG_ACS */

#endif /* ACS_H */
