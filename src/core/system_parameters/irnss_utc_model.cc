/*
 * \file irnss_utc_model.h
 * \brief  Interface of a IRNSS UTC MODEL storage
 * 
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#include "irnss_utc_model.h"
#include <cmath>


Irnss_Utc_Model::Irnss_Utc_Model()
{
    valid = false;
    d_A0 = 0.0;
    d_A1 = 0.0;
    d_A2 = 0.0;
    d_t_OT = 0;
    i_WN_T = 0;
    d_DeltaT_LS = 0;
    i_WN_LSF = 0;
    i_DN = 0;
    d_DeltaT_LSF = 0;
}


double Irnss_Utc_Model::utc_time(double irnsstime_corrected, int32_t i_IRNSS_week)
{
    double t_utc;
    double t_utc_daytime;
    double Delta_t_UTC = d_DeltaT_LS + d_A0 + d_A1 * (irnsstime_corrected - d_t_OT + 604800 * static_cast<double>(i_IRNSS_week - i_WN_T));

    // Determine if the effectivity time of the leap second event is in the past
    int32_t weeksToLeapSecondEvent = i_WN_LSF - i_IRNSS_week;

    if (weeksToLeapSecondEvent >= 0)  // is not in the past
        {
            // Detect if the effectivity time and user's time is within six hours  = 6 * 60 *60 = 21600 s
            int32_t secondOfLeapSecondEvent = i_DN * 24 * 60 * 60;
            if (weeksToLeapSecondEvent > 0)
                {
                    t_utc_daytime = fmod(irnsstime_corrected - Delta_t_UTC, 86400);
                }
            else  // we are in the same week than the leap second event
                {
                    if (std::abs(irnsstime_corrected - secondOfLeapSecondEvent) > 21600)
                        {
                            /* 20.3.3.5.2.4a
                             * Whenever the effectivity time indicated by the IRNSS_WN_LSF and the IRNSS_DN values
                             * is not in the past (relative to the user's present time), and the user's
                             * present time does not fall in the time span which starts at six hours prior
                             * to the effectivity time and ends at six hours after the effectivity time,
                             * the UTC/IRNSS-time relationship is given by
                             */
                            t_utc_daytime = fmod(irnsstime_corrected - Delta_t_UTC, 86400);
                        }
                    else
                        {
                            /* 20.3.3.5.2.4b
                             * Whenever the user's current time falls within the time span of six hours
                             * prior to the effectivity time to six hours after the effectivity time,
                             * proper accommodation of the leap second event with a possible week number
                             * transition is provided by the following expression for UTC:
                             */
                            int32_t W = static_cast<int32_t>(fmod(irnsstime_corrected - Delta_t_UTC - 43200, 86400)) + 43200;
                            t_utc_daytime = fmod(W, 86400 + d_DeltaT_LSF - d_DeltaT_LS);
                            // implement something to handle a leap second event!
                        }
                    if ((irnsstime_corrected - secondOfLeapSecondEvent) > 21600)
                        {
                            Delta_t_UTC = d_DeltaT_LSF + d_A0 + d_A1 * (irnsstime_corrected - d_t_OT + 604800 * static_cast<double>(i_IRNSS_week - i_WN_T));
                            t_utc_daytime = fmod(irnsstime_corrected - Delta_t_UTC, 86400);
                        }
                }
        }
    else  // the effectivity time is in the past
        {
            /* 20.3.3.5.2.4c
             * Whenever the effectivity time of the leap second event, as indicated by the
             * WNLSF and IRNSS_DN values, is in the "past" (relative to the user's current time),
             * and the user's current time does not fall in the time span as given above
             * in 20.3.3.5.2.4b,*/
            Delta_t_UTC = d_DeltaT_LSF + d_A0 + d_A1 * (irnsstime_corrected - d_t_OT + 604800 * static_cast<double>(i_IRNSS_week - i_WN_T));
            t_utc_daytime = fmod(irnsstime_corrected - Delta_t_UTC, 86400);
        }

    double secondsOfWeekBeforeToday = 86400 * floor(irnsstime_corrected / 86400);
    t_utc = secondsOfWeekBeforeToday + t_utc_daytime;
    return t_utc;
}
