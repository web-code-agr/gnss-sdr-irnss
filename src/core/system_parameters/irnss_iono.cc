/*!
 * \file irnss_iono.cc
 * \brief  Interface of an IRNSS IONOSPHERIC MODEL storage
 *
 * See https://www.isro.gov.in/sites/default/files/irnss_sps_icd_version1.1-2017.pdf Appendix D
 * 
 *
 * -------------------------------------------------------------------------
 *
 * 
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

#include "irnss_iono.h"

Irnss_Iono::Irnss_Iono()
{
    valid = false;
    d_alpha0 = 0.0;
    d_alpha1 = 0.0;
    d_alpha2 = 0.0;
    d_alpha3 = 0.0;
    d_beta0 = 0.0;
    d_beta1 = 0.0;
    d_beta2 = 0.0;
    d_beta3 = 0.0;
}
