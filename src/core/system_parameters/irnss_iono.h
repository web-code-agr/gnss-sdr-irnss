/*!
 * \file irnsss_iono.h
 * \brief  Interface of a IRNSS IONOSPHERIC MODEL storage
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
 *
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_IRNSS_IONO_H
#define GNSS_SDR_IRNSS_IONO_H


#include <boost/serialization/nvp.hpp>


/*!
 * \brief This class is a storage for the IRNSS IONOSPHERIC data as described in Page 25 Table 17 of the IRNSS document
 *
 * See https://www.isro.gov.in/sites/default/files/irnss_sps_icd_version1.1-2017.pdf Appendix D
 */
class Irnss_Iono
{
public:
    bool valid;  //!< Valid flag
    // Ionospheric parameters
    double d_alpha0;  //!< Coefficient 0 of a cubic equation representing the amplitude of the vertical delay [s]
    double d_alpha1;  //!< Coefficient 1 of a cubic equation representing the amplitude of the vertical delay [s/semi-circle]
    double d_alpha2;  //!< Coefficient 2 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^2]
    double d_alpha3;  //!< Coefficient 3 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^3]
    double d_beta0;   //!< Coefficient 0 of a cubic equation representing the period of the model [s]
    double d_beta1;   //!< Coefficient 1 of a cubic equation representing the period of the model [s/semi-circle]
    double d_beta2;   //!< Coefficient 2 of a cubic equation representing the period of the model [s(semi-circle)^2]
    double d_beta3;   //!< Coefficient 3 of a cubic equation representing the period of the model [s(semi-circle)^3]

    Irnss_Iono();  //!< Default constructor

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the ephemeris data on disk file.
     */
    inline void serialize(Archive& archive, const unsigned int version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };
        archive& make_nvp("d_alpha0", d_alpha0);
        archive& make_nvp("d_alpha1", d_alpha1);
        archive& make_nvp("d_alpha2", d_alpha2);
        archive& make_nvp("d_alpha3", d_alpha3);
        archive& make_nvp("d_beta0", d_beta0);
        archive& make_nvp("d_beta1", d_beta1);
        archive& make_nvp("d_beta2", d_beta2);
        archive& make_nvp("d_beta3", d_beta3);
    }
};

#endif
