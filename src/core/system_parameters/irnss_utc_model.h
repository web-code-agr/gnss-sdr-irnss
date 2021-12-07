/*!
 * \file irnss_utc_model.h
 * \brief  Interface of a IRNSS UTC MODEL storage
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


#ifndef GNSS_SDR_IRNSS_UTC_MODEL_H
#define GNSS_SDR_IRNSS_UTC_MODEL_H

#include <boost/serialization/nvp.hpp>
#include <cstdint>

/*!
 * \brief This class is a storage for the IRNSS UTC MODEL data as described in Page 24 of IRNSS document
 *
 * See https://www.isro.gov.in/sites/default/files/irnss_sps_icd_version1.1-2017.pdf
 */
class Irnss_Utc_Model
{
public:
    bool valid;
    // UTC parameters
    double d_A0;           //!< Constant of a model that relates IRNSS and UTC time (ref. 6.1.3.3 IRNSS) [s]
    double d_A1;           //!< 1st order term of a model that relates IRNSS and UTC time (ref. 6.1.3.3 IRNSS) [s/s]
    double d_A2;           //!< 2nd order term of a model that relates IRNSS and UTC time (ref. 6.1.3.3 IRNSS) [s/s^2]
    int32_t d_t_OT;        //!< Reference time for UTC data (ref. 6.1.3.3 IRNSS) [s]
    int32_t i_WN_T;        //!< UTC reference week number [weeks]
    int32_t d_DeltaT_LS;   //!< delta time due to leap seconds [s]
    int32_t i_WN_LSF;      //!< Week number at the end of which the leap second becomes effective [weeks]
    int32_t i_DN;          //!< Day number (IRNSS_DN) at the end of which the leap second becomes effective [days]
    int32_t d_DeltaT_LSF;  //!< Scheduled future or recent past (relative to NAV message upload) value of the delta time due to leap seconds [s]

    /*!
     * Default constructor
     */
    Irnss_Utc_Model();

    template <class Archive>
    /*
     * \brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the ephemeris data on disk file.
     */
    inline void serialize(Archive& archive, const uint32_t version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };
        archive& make_nvp("valid", valid);
        archive& make_nvp("d_A1", d_A1);
        archive& make_nvp("d_A0", d_A0);
        archive& make_nvp("d_t_OT", d_t_OT);
        archive& make_nvp("i_WN_T", i_WN_T);
        archive& make_nvp("d_DeltaT_LS", d_DeltaT_LS);
        archive& make_nvp("i_WN_LSF", i_WN_LSF);
        archive& make_nvp("i_DN", i_DN);
        archive& make_nvp("d_DeltaT_LSF", d_DeltaT_LSF);
    }

    /*!
     * \brief Computes the Coordinated Universal Time (UTC) and
     * returns it in [s] 
     */
    double utc_time(double irnsstime_corrected, int32_t i_IRNSS_week);
};

#endif
