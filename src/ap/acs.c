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

#include "includes.h"
#include <math.h>

#include "acs.h"
#include "drivers/driver.h"
#include "ap/ap_drv_ops.h"
#include "ap/ap_config.h"
#include "ap/hw_features.h"
#include "common/ieee802_11_defs.h"

/*
 * Automatic Channel Selection
 *
 * http://wireless.kernel.org/en/users/Documentation/acs
 *
 * This is the hostapd AP ACS implementation
 *
 * You get automatic channel selection when you configure hostapd
 * with a channel=acs_survey or channel=0 on the hostapd.conf file.
 *
 * TODO:
 *
 * - The current algorithm is heavily based on the amount of time
 *   we are willing to spend offchannel configurable via acs_roc_duration_ms,
 *   and acs_num_req_surveys, this will work for the period of time we do
 *   the analysis, so if these values are too low you'd use an ideal channel
 *   only based on the short bursts of traffic on the channel. We can also take
 *   into consideration other data to help us further make a better analysis
 *   and speed out our decision:
 *
 *	* Use a frequency broker to collect other PHY RF interference:
 *	  * BT devices, etc, assign interference value aggregates to these
 *
 * - An ideal result would continue surveying the channels and collect a
 *   histogram, the ideal channel then will remain ideal for most of the
 *   collected history.
 *
 * - Add wpa_supplicant support for ACS, ideal for P2P
 *
 * - Randomize channel study
 *
 *
 * survey_interference_factor in struct hostapd_channel_data:
 *
 * Computed interference factor on this channel
 *
 * The survey interference factor is defined as the ratio of the
 * observed busy time over the time we spent on the channel,
 * this value is then amplified by the observed noise floor on
 * the channel in comparison to the lowest noise floor observed
 * on the entire band.
 *
 * This corresponds to:
 * ---
 * (busy time - tx time) / (active time - tx time) * 2^(chan_nf + band_min_nf)
 * ---
 *
 * The coefficient of 2 reflects the way power in "far-field"
 * radiation decreases as the square of distance from the antenna [1].
 * What this does is it decreases the observed busy time ratio if the
 * noise observed was low but increases it if the noise was high,
 * proportionally to the way "far field" radiation changes over
 * distance.
 *
 * If channel busy time is not available the fallback is to use channel rx time.
 *
 * Since noise floor is in dBm it is necessary to convert it into Watts so that
 * combined channel intereference (e.g. HT40, which uses two channels) can be
 * calculated easily.
 * ---
 * (busy time - tx time) / (active time - tx time) *
 *    2^(10^(chan_nf/10) + 10^(band_min_nf/10))
 * ---
 *
 * However to account for cases where busy/rx time is 0 (channel load is then
 * 0%) channel noise floor signal power is combined into the equation which
 * then becomes:
 * ---
 * 10^(chan_nf/5) + (busy time - tx time) / (active time - tx time) *
 *    2^(10^(chan_nf/10) + 10^(band_min_nf/10))
 * ---
 *
 * All this "interference factor" is purely subjective and only time
 * will tell how usable this is. By using the minimum noise floor we
 * remove any possible issues due to card calibration. The computation
 * of the interference factor then is dependent on what the card itself
 * picks up as the minimum noise, not an actual real possible card
 * noise value.
 *
 * Example ACS analysis:
 *
 * ACS: Survey analysis for channel 1 (2412 MHz)
 * ACS:  1: min_nf=-111 interference_factor=0.0338551 nf=-111 time=5878 busy=0 rx=199
 * ACS:  2: min_nf=-111 interference_factor=0.0347777 nf=-111 time=7016 busy=0 rx=244
 * ACS:  * interference factor average: 0.0343164
 * ACS: Survey analysis for channel 2 (2417 MHz)
 * ACS:  1: min_nf=-111 interference_factor=0.0580031 nf=-111 time=5879 busy=0 rx=341
 * ACS:  2: min_nf=-111 interference_factor=0.0569963 nf=-111 time=7018 busy=0 rx=400
 * ACS:  * interference factor average: 0.0574997
 * ACS: Survey analysis for channel 3 (2422 MHz)
 * ACS:  1: min_nf=-111 interference_factor=0.0195545 nf=-111 time=5881 busy=0 rx=115
 * ACS:  2: min_nf=-111 interference_factor=0.0188034 nf=-111 time=7020 busy=0 rx=132
 * ACS:  * interference factor average: 0.019179
 * ACS: Survey analysis for channel 4 (2427 MHz)
 * ACS:  1: min_nf=-112 interference_factor=0.0161565 nf=-112 time=5880 busy=0 rx=95
 * ACS:  2: min_nf=-112 interference_factor=0.0161015 nf=-112 time=7018 busy=0 rx=113
 * ACS:  * interference factor average: 0.016129
 * ACS: Survey analysis for channel 5 (2432 MHz)
 * ACS:  1: min_nf=-111 interference_factor=0.0357143 nf=-111 time=5880 busy=0 rx=210
 * ACS:  2: min_nf=-111 interference_factor=0.0347628 nf=-111 time=7019 busy=0 rx=244
 * ACS:  * interference factor average: 0.0352385
 * ACS: Survey analysis for channel 6 (2437 MHz)
 * ACS:  1: min_nf=-109 interference_factor=0.0486395 nf=-109 time=5880 busy=0 rx=286
 * ACS:  2: min_nf=-109 interference_factor=0.046737 nf=-109 time=7018 busy=0 rx=328
 * ACS:  * interference factor average: 0.0476882
 * ACS: Survey analysis for channel 7 (2442 MHz)
 * ACS:  1: min_nf=-109 interference_factor=0.0518884 nf=-109 time=5878 busy=0 rx=305
 * ACS:  2: min_nf=-109 interference_factor=0.0500285 nf=-108 time=7016 busy=0 rx=351
 * ACS:  * interference factor average: 0.0509585
 * ACS: Survey analysis for channel 8 (2447 MHz)
 * ACS:  1: min_nf=-110 interference_factor=0.0836877 nf=-110 time=5879 busy=0 rx=492
 * ACS:  2: min_nf=-110 interference_factor=0.0799487 nf=-110 time=7017 busy=0 rx=561
 * ACS:  * interference factor average: 0.0818182
 * ACS: Survey analysis for channel 9 (2452 MHz)
 * ACS:  1: min_nf=-112 interference_factor=0.0741623 nf=-112 time=5879 busy=0 rx=436
 * ACS:  2: min_nf=-112 interference_factor=0.0706855 nf=-112 time=7017 busy=0 rx=496
 * ACS:  * interference factor average: 0.0724239
 * ACS: Survey analysis for channel 10 (2457 MHz)
 * ACS:  1: min_nf=-112 interference_factor=0.0443953 nf=-112 time=5879 busy=0 rx=261
 * ACS:  2: min_nf=-112 interference_factor=0.0433172 nf=-111 time=7018 busy=0 rx=304
 * ACS:  * interference factor average: 0.0438562
 * ACS: Survey analysis for channel 11 (2462 MHz)
 * ACS:  1: min_nf=-111 interference_factor=0.0654985 nf=-111 time=5878 busy=0 rx=385
 * ACS:  2: min_nf=-111 interference_factor=0.0648425 nf=-110 time=7017 busy=0 rx=455
 * ACS:  * interference factor average: 0.0651705
 * ACS: Survey analysis for selected bandwidth 20MHz
 * ACS:  * channel 1: total interference = 0.110995
 * ACS:  * channel 2: total interference = 0.127124
 * ACS:  * channel 3: total interference = 0.162362
 * ACS:  * channel 4: total interference = 0.175734
 * ACS:  * channel 5: total interference = 0.169193
 * ACS:  * channel 6: total interference = 0.231832
 * ACS:  * channel 7: total interference = 0.288127
 * ACS:  * channel 8: total interference = 0.296745
 * ACS:  * channel 9: total interference = 0.314227
 * ACS:  * channel 10: total interference = 0.263269
 * ACS:  * channel 11: total interference = 0.181451
 * ACS: ideal channel is 1 (2412 MHz) with total interference factor of 0.110995
 *
 * [1] http://en.wikipedia.org/wiki/Near_and_far_field
 */

static void acs_clean_chan_surveys(struct hostapd_channel_data *chan)
{
	struct freq_survey *survey, *tmp;

	if (dl_list_empty(&chan->survey_list))
		return;

	dl_list_for_each_safe(survey, tmp, &chan->survey_list,
			      struct freq_survey, list) {
		dl_list_del(&survey->list);
		os_free(survey);
	}
}


static void acs_cleanup(struct hostapd_iface *iface)
{
	int i;
	struct hostapd_channel_data *chan;

	for (i = 0; i < iface->current_mode->num_channels; i++) {
		chan = &iface->current_mode->channels[i];

		if (chan->flag & HOSTAPD_CHAN_SURVEY_LIST_INITIALIZED)
			acs_clean_chan_surveys(chan);

		dl_list_init(&chan->survey_list);
		chan->flag |= HOSTAPD_CHAN_SURVEY_LIST_INITIALIZED;
		chan->min_nf = 0;
	}

	iface->chans_surveyed = 0;
}


void acs_fail(struct hostapd_iface *iface)
{
	wpa_printf(MSG_ERROR, "ACS: Failed to start");
	acs_cleanup(iface);
}


static long double
acs_survey_interference_factor(struct freq_survey *survey, s8 min_nf)
{
	long double factor, busy, total;

	if (survey->filled & SURVEY_HAS_CHAN_TIME_BUSY)
		busy = survey->channel_time_busy;
	else if (survey->filled & SURVEY_HAS_CHAN_TIME_RX)
		busy = survey->channel_time_rx;
	else {
		/* This shouldn't really happen as survey data is checked in
		 * acs_sanity_check() */
		wpa_printf(MSG_ERROR, "ACS: Survey data missing!");
		return 0;
	}

	total = survey->channel_time;

	if (survey->filled & SURVEY_HAS_CHAN_TIME_TX) {
		busy -= survey->channel_time_tx;
		total -= survey->channel_time_tx;
	}

	factor = pow(10, survey->nf / 5.0L) +
		(busy / total) * pow(2, pow(10, (long double)survey->nf / 10.0L) -
					pow(10, (long double)min_nf / 10.0L));

	return factor;
}


static void
acs_survey_chan_interference_factor(struct hostapd_iface *iface,
				    struct hostapd_channel_data *chan)
{
	struct freq_survey *survey;
	unsigned int i = 0;
	long double int_factor = 0;

	if (dl_list_empty(&chan->survey_list))
		return;

	if (chan->flag & HOSTAPD_CHAN_DISABLED)
		return;

	chan->survey_interference_factor = 0;

	dl_list_for_each(survey, &chan->survey_list, struct freq_survey, list)
	{
		int_factor = acs_survey_interference_factor(survey,
							    iface->lowest_nf);
		chan->survey_interference_factor += int_factor;
		wpa_printf(MSG_DEBUG, "ACS:  %d: min_nf=%d interference_"
			   "factor=%Lg nf=%d time=%lu busy=%lu rx=%lu",
			   ++i, chan->min_nf, int_factor,
			   survey->nf, (unsigned long) survey->channel_time,
			   (unsigned long) survey->channel_time_busy,
			   (unsigned long) survey->channel_time_rx);
	}

	chan->survey_interference_factor = chan->survey_interference_factor /
		dl_list_len(&chan->survey_list);
}


static int acs_usable_chan(struct hostapd_channel_data *chan)
{
	if (dl_list_empty(&chan->survey_list))
		return 0;
	if (chan->flag & HOSTAPD_CHAN_DISABLED)
		return 0;
	return 1;
}


static int acs_usable_ht40_chan(struct hostapd_channel_data *chan)
{
	const int allowed[] = { 36, 44, 52, 60, 100, 108, 116, 124, 132, 149,
			        157, 184, 192 };
	unsigned int i;

	for (i = 0; i < sizeof(allowed) / sizeof(allowed[0]); i++)
		if (chan->chan == allowed[i])
			return 1;

	return 0;
}


static int acs_survey_is_sufficient(struct freq_survey *survey)
{
	if (!(survey->filled & SURVEY_HAS_NF)) {
		wpa_printf(MSG_ERROR, "ACS: Survey is missing noise floor");
		return 0;
	}

	if (!(survey->filled & SURVEY_HAS_CHAN_TIME)) {
		wpa_printf(MSG_ERROR, "ACS: Survey is missing channel time");
		return 0;
	}

	if (!(survey->filled & SURVEY_HAS_CHAN_TIME_BUSY) &&
	    !(survey->filled & SURVEY_HAS_CHAN_TIME_RX)) {
		wpa_printf(MSG_ERROR, "ACS: Survey is missing rx and busy "
			  "time (at least one is required)");
		return 0;
	}

	return 1;
}


static int acs_surveys_are_sufficient(struct hostapd_iface *iface)
{
	int i;
	struct hostapd_channel_data *chan;
	struct freq_survey *survey;

	for (i = 0; i < iface->current_mode->num_channels; i++) {
		chan = &iface->current_mode->channels[i];
		if (chan->flag & HOSTAPD_CHAN_DISABLED)
			continue;

		dl_list_for_each(survey, &chan->survey_list,
				 struct freq_survey, list)
		{
			if (!acs_survey_is_sufficient(survey)) {
				wpa_printf(MSG_ERROR, "ACS: Channel %d has "
					   "insufficient survey data",
					   chan->chan);
				return 0;
			}
		}
	}

	return 1;
}


static void acs_survey_all_chans_intereference_factor(struct hostapd_iface *iface)
{
	int i;
	struct hostapd_channel_data *chan;

	for (i = 0; i < iface->current_mode->num_channels; i++) {
		chan = &iface->current_mode->channels[i];

		if (!acs_usable_chan(chan))
			continue;

		wpa_printf(MSG_DEBUG, "ACS: Survey analysis for channel "
			   "%d (%d MHz)", chan->chan, chan->freq);

		acs_survey_chan_interference_factor(iface, chan);

		wpa_printf(MSG_DEBUG, "ACS:  * interference factor "
			   "average: %Lg", chan->survey_interference_factor);
	}
}


static struct hostapd_channel_data *acs_find_chan(struct hostapd_iface *iface,
						  int freq)
{
	struct hostapd_channel_data *chan;
	int i;

	for (i = 0; i < iface->current_mode->num_channels; i++) {
		chan = &iface->current_mode->channels[i];

		if (!acs_usable_chan(chan))
			continue;

		if (chan->freq == freq)
			return chan;
	}

	return NULL;
}


/*
 * At this point its assumed we have the iface->lowest_nf
 * and all chan->min_nf values
 */
static struct hostapd_channel_data *
acs_find_ideal_chan(struct hostapd_iface *iface)
{
	struct hostapd_channel_data *chan, *adj_chan, *ideal_chan = NULL;
	long double factor, ideal_factor = 0;
	int i, j;
	int n_chans = 1;

	/* TODO: HT40- support */

	if (iface->conf->ieee80211n &&
	    iface->conf->secondary_channel == -1) {
		wpa_printf(MSG_ERROR, "ACS: HT40- is not supported yet. Please "
			   "try HT40+");
		return NULL;
	}

	if (iface->conf->ieee80211n &&
	    iface->conf->secondary_channel)
		n_chans = 2;

	if (iface->conf->ieee80211ac &&
	    iface->conf->vht_oper_chwidth == 1)
		n_chans = 4;

	/* TODO: VHT80+80, VHT160. Update acs_adjust_vht_sec_chan() too. */

	wpa_printf(MSG_DEBUG, "ACS: Survey analysis for selected "
		   "bandwidth %dMHz",
		   n_chans == 1 ? 20 :
		   n_chans == 2 ? 40 :
		   n_chans == 4 ? 80 :
		   -1);

	for (i = 0; i < iface->current_mode->num_channels; i++) {
		chan = &iface->current_mode->channels[i];

		if (!acs_usable_chan(chan))
			continue;

		/* HT40 on 5GHz has a limited set of primary channels as per
		 * 11n Annex J */
		if (iface->current_mode->mode == HOSTAPD_MODE_IEEE80211A &&
		    iface->conf->ieee80211n &&
		    iface->conf->secondary_channel &&
		    !acs_usable_ht40_chan(chan)) {
			wpa_printf(MSG_DEBUG, "ACS: Channel %d: not allowed as "
				   "primary channel for HT40", chan->chan);
			continue;
		}

		factor = chan->survey_interference_factor;

		for (j = 1; j < n_chans; j++) {
			adj_chan = acs_find_chan(iface, chan->freq + (j * 20));
			if (!adj_chan)
				break;

			factor += adj_chan->survey_interference_factor;
		}

		if (j != n_chans) {
			wpa_printf(MSG_DEBUG, "ACS: Channel %d: not enough "
				  "bandwidth", chan->chan);
			continue;
		}

		/* 2.4GHz has overlapping 20MHz channels. Include adjacent
		 * channel interference factor. */
		if (iface->current_mode->mode == HOSTAPD_MODE_IEEE80211B ||
		    iface->current_mode->mode == HOSTAPD_MODE_IEEE80211G) {
			for (j = 0; j < n_chans; j++) {
				/* TODO: perhaps a multiplier should be used here? */

				adj_chan = acs_find_chan(iface, chan->freq + (j * 20) - 5);
				if (adj_chan)
					factor += adj_chan->survey_interference_factor;

				adj_chan = acs_find_chan(iface, chan->freq + (j * 20) - 10);
				if (adj_chan)
					factor += adj_chan->survey_interference_factor;

				adj_chan = acs_find_chan(iface, chan->freq + (j * 20) + 5);
				if (adj_chan)
					factor += adj_chan->survey_interference_factor;

				adj_chan = acs_find_chan(iface, chan->freq + (j * 20) + 10);
				if (adj_chan)
					factor += adj_chan->survey_interference_factor;
			}
		}

		wpa_printf(MSG_DEBUG, "ACS:  * channel %d: total interference "
			   "= %Lg", chan->chan, factor);

		if (!ideal_chan || factor < ideal_factor) {
			ideal_factor = factor;
			ideal_chan = chan;
		}
	}

	if (ideal_chan)
		wpa_printf(MSG_DEBUG, "ACS: Ideal channel is %d (%d MHz) "
			   "with total interference factor of %Lg",
			   ideal_chan->chan, ideal_chan->freq, ideal_factor);

	return ideal_chan;
}


static void acs_adjust_vht_sec_chan(struct hostapd_iface *iface)
{
	if (!iface->conf->ieee80211ac)
		return;

	wpa_printf(MSG_INFO, "ACS: Adjusting VHT second oper channel");

	switch (iface->conf->vht_oper_chwidth) {
	case VHT_CHANWIDTH_USE_HT:
		iface->conf->vht_oper_centr_freq_seg0_idx =
				iface->conf->channel + 2;
		break;
	case VHT_CHANWIDTH_80MHZ:
		iface->conf->vht_oper_centr_freq_seg0_idx =
				iface->conf->channel + 6;
		break;
	default:
		/* TODO: How can this be calculated? Adjust
		 * acs_find_ideal_chan() */
		wpa_printf(MSG_INFO, "ACS: Only VHT20/40/80 is "
			   "supported now");
		break;
	}
}


static int acs_study_survey_based(struct hostapd_iface *iface)
{
	wpa_printf(MSG_DEBUG, "ACS: Trying survey-based ACS");

	if (!iface->chans_surveyed) {
		wpa_printf(MSG_ERROR, "ACS: Unable to collect survey data");
		return -1;
	}

	if (!acs_surveys_are_sufficient(iface)) {
		wpa_printf(MSG_ERROR, "ACS: Surveys have insufficient data");
		return -1;
	}

	acs_survey_all_chans_intereference_factor(iface);
	return 0;
}


static int acs_study_options(struct hostapd_iface *iface)
{
	int err;

	err = acs_study_survey_based(iface);
	if (err == 0)
		return 0;

	/* TODO: If no surveys are available/sufficient this is a good
	 * place to fallback to BSS-based ACS */

	return -1;
}


static void acs_study(struct hostapd_iface *iface)
{
	struct hostapd_channel_data *ideal_chan;
	int err;

	err = acs_study_options(iface);
	if (err < 0) {
		wpa_printf(MSG_ERROR, "ACS: All study options have failed");
		goto fail;
	}

	ideal_chan = acs_find_ideal_chan(iface);
	if (!ideal_chan) {
		wpa_printf(MSG_ERROR, "ACS: Failed to compute ideal channel");
		goto fail;
	}

	iface->conf->channel = ideal_chan->chan;

	acs_adjust_vht_sec_chan(iface);

	/*
	 * hostapd_setup_interface_complete() will return -1 on failure,
	 * 0 on success and 0 is HOSTAPD_CHAN_VALID :)
	 */
	switch (hostapd_acs_completed(iface)) {
	case HOSTAPD_CHAN_VALID:
		acs_cleanup(iface);
		return;
	case HOSTAPD_CHAN_INVALID:
	case HOSTAPD_CHAN_ACS:
	default:
		/* This can possibly happen if channel parameters (secondary
		 * channel, center frequencies) are misconfigured */
		wpa_printf(MSG_ERROR, "ACS: Possibly channel configuration is "
			   "invalid, please report this along with your "
			   "config file.");
		goto fail;
	}

fail:
	acs_fail(iface);
}


static void acs_scan_complete(struct hostapd_iface *iface)
{
	int err;

	wpa_printf(MSG_DEBUG, "ACS: using survey based algorithm "
		   "(acs_chan_time_ms=%d)",
		   iface->conf->acs_chan_time_ms);

	err = hostapd_drv_get_survey(iface->bss[0], 0);
	if (err) {
		wpa_printf(MSG_ERROR, "ACS: Failed to get survey data");
		acs_fail(iface);
	}

	acs_study(iface);
}


static int acs_request_scan(struct hostapd_iface *iface)
{
	struct wpa_driver_scan_params params;
	struct hostapd_channel_data *chan;
	int i, *freq;

	os_memset(&params, 0, sizeof(params));
	params.chan_time = iface->conf->acs_chan_time_ms;
	params.freqs = os_malloc(sizeof(params.freqs[0]) *
			(iface->current_mode->num_channels + 1));

	freq = params.freqs;
	for (i = 0; i < iface->current_mode->num_channels; i++) {
		chan = &iface->current_mode->channels[i];
		if (chan->flag & HOSTAPD_CHAN_DISABLED)
			continue;

		*freq++ = chan->freq;
	}
	*freq = 0;

	iface->scan_cb = acs_scan_complete;

	if (hostapd_driver_scan(iface->bss[0], &params) < 0) {
		wpa_printf(MSG_ERROR, "ACS: Failed to request initial scan");
		acs_cleanup(iface);
		return -1;
	}

	os_free(params.freqs);
	return 0;
}


enum hostapd_chan_status acs_init(struct hostapd_iface *iface)
{
	int err;

	wpa_printf(MSG_INFO, "ACS: Automatic channel selection started, this "
		   "may take a bit");

	acs_cleanup(iface);

	err = acs_request_scan(iface);
	if (err < 0)
		return HOSTAPD_CHAN_INVALID;

	return HOSTAPD_CHAN_ACS;
}
