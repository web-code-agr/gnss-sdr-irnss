/*!
 * \file irnss_navigation_message.h
 * \brief  Interface of a IRNSS NAV Data message decoder
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


#ifndef GNSS_SDR_IRNSS_NAVIGATION_MESSAGE_H
#define GNSS_SDR_IRNSS_NAVIGATION_MESSAGE_H


#include "IRNSS_at_1.h"
#include "irnss_ephemeris.h"
#include "irnss_iono.h"
#include "irnss_utc_model.h"
#include <bitset>
#include <cstdint>
#include <map>
#include <string>
#include <utility>  // for pair
#include <vector>


/*!
 * \brief This class decodes a IRNSS NAV Data message
 *
 * See https://www.isro.gov.in/sites/default/files/irnss_sps_icd_version1.1-2017.pdf
 */
class Irnss_Navigation_Message
{
public:
    /*!
     * Default constructor
     */
    Irnss_Navigation_Message();

    /*!
     * \brief Obtain a IRNSS SV Ephemeris class filled with current SV data
     */
    Irnss_Ephemeris get_ephemeris();

    /*!
     * \brief Obtain a IRNSS ionospheric correction parameters class filled with current SV data
     */
    Irnss_Iono get_iono();

    /*!
     * \brief Obtain a IRNSS UTC model parameters class filled with current SV data
     */
    Irnss_Utc_Model get_utc_model();

    /*!
     * \brief Decodes the IRNSS NAV message
     */
    int32_t subframe_decoder(std::string& subframe);

    /*!
     * \brief Computes the Coordinated Universal Time (UTC) and
     * returns it in [s] 
     */
    double utc_time(const double irnsstime_corrected);

    inline int32_t get_TOW() const
    {
        return d_TOW;
    }

    inline int32_t get_GPS_week() const
    {
        return i_IRNSS_week;
    }

    inline void set_satellite_PRN(uint32_t prn)
    {
        i_satellite_PRN = prn;
    }

    inline uint32_t get_satellite_PRN() const
    {
        return i_satellite_PRN;
    }

    inline void set_channel(int32_t channel_id)
    {
        i_channel_ID = channel_id;
    }

    inline bool get_flag_iono_valid() const
    {
        return flag_iono_valid;
    }

    inline bool get_flag_utc_model_valid() const
    {
        return flag_utc_model_valid;
    }

    bool satellite_validation();

    bool b_valid_ephemeris_set_flag;  // flag indicating that this ephemeris set have passed the validation check
    // broadcast orbit 1
    int32_t d_TOW;      //!< Time of IRNSS Week of the ephemeris set (taken from subframes IRNSS_TOW) [s]
    int32_t d_TOW_SF1;  //!< Time of IRNSS Week from HOW word of Subframe 1 [s]
    int32_t d_TOW_SF2;  //!< Time of IRNSS Week from HOW word of Subframe 2 [s]
    int32_t d_TOW_SF3;  //!< Time of IRNSS Week from HOW word of Subframe 3 [s]
    int32_t d_TOW_SF4;  //!< Time of IRNSS Week from HOW word of Subframe 4 [s]
 
    int32_t d_IODEC_SF1;
    int32_t d_IODEC_SF3;
    int32_t d_IODEC_SF4;
    double d_Crs;      //!< Amplitude of the Sine Harmonic Correction Term to the Orbit Radius [m]
    double d_Delta_n;  //!< Mean Motion Difference From Computed Value [semi-circles/s]
    double d_M_0;      //!< Mean Anomaly at Reference Time [semi-circles]
    // broadcast orbit 2
    double d_Cuc;             //!< Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude [rad]
    double d_e_eccentricity;  //!< Eccentricity [dimensionless]
    double d_Cus;             //!< Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude [rad]
    double d_sqrt_A;          //!< Square Root of the Semi-Major Axis [sqrt(m)]
    // broadcast orbit 3
    int32_t d_Toe;    //!< Ephemeris data reference time of week (Ref. 6.2.1.8 IRNSS Doc) [s]
    int32_t d_Toc;    //!< clock data reference time (Ref. Appendix A IRNSS Doc) [s]
    double d_Cic;     //!< Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination [rad]
    double d_OMEGA0;  //!< Longitude of Ascending Node of Orbit Plane at Weekly Epoch [semi-circles]
    double d_Cis;     //!< Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination [rad]
    // broadcast orbit 4
    double d_i_0;        //!< Inclination Angle at Reference Time [semi-circles]
    double d_Crc;        //!< Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius [m]
    double d_OMEGA;      //!< Argument of Perigee [semi-cicles]
    double d_OMEGA_DOT;  //!< Rate of Right Ascension [semi-circles/s]
    // broadcast orbit 5
    double d_IDOT;          //!< Rate of Inclination Angle [semi-circles/s]
    int32_t d_IODC;
    int32_t i_code_on_S;
    int32_t i_code_on_L2;   //!< If 1, P code ON in L2;  if 2, C/A code ON in L2;
    int32_t i_IRNSS_week;     //!< IRNSS week number, aka WN [week]
    bool b_S_P_data_flag;  //!< When true, indicates that the NAV data stream was commanded OFF on the P-code of the S channel (Ref 6.2.1.6)
    // broadcast orbit 6
    bool b_L5_P_data_flag;
    int32_t i_SV_accuracy;  //!< User Range Accuracy (URA) index of the SV (reference paragraph 6.2.1.4) for the standard positioning service user
    int32_t i_SV_health;
    double d_TGD;    //!< Estimated Group Delay Differential: S-L5 correction term only for the benefit of "S" or "L5" s users [s]
    int32_t d_IODEC;  //!< Issue of Data, Clock
    // broadcast orbit 7
    //int32_t i_AODO;            //!< Age of Data Offset (AODO) term for the navigation message correction table (NMCT) contained in subframe 4 (reference paragraph 20.3.3.5.1.9) [s]
 //   bool b_fit_interval_flag;  //!< indicates the curve-fit interval used by the CS (Block II/IIA/IIR/IIR-M/IIF) and SS (Block IIIA) in determining the ephemeris parameters, as follows: 0 = 4 hours, 1 = greater than 4 hours.
    double d_spare1;
    double d_spare2;
    double d_A_f0;  //!< Coefficient 0 of code phase offset model [s]  (Ref. Appendix A)
    double d_A_f1;  //!< Coefficient 1 of code phase offset model [s/s]
    double d_A_f2;  //!< Coefficient 2 of code phase offset model [s/s^2]

    // Almanac
    int32_t i_Toa;                             //!< Almanac reference time [s]
    int32_t i_WN_A;                            //!< To which the almanac reference time (i_Toa) is referenced
    std::map<int32_t, int32_t> almanacHealth;  //!< Map that stores the health information stored in the almanac

    std::map<int32_t, std::string> satelliteBlock;  //!< Map that stores to which block the PRN belongs

    // Flags

    /*! \brief If true, enhanced level of integrity assurance.
     *
     *  If false, indicates that the conveying signal is provided with the legacy level of integrity assurance.
     *  That is, the probability that the instantaneous URE of the conveying signal exceeds 4.42 times the upper bound
     *  value of the current broadcast URA index, for more than 5.2 seconds, without an accompanying alert, is less
     *  than 1E-5 per hour. If true, indicates that the conveying signal is provided with an enhanced level of
     *  integrity assurance. That is, the probability that the instantaneous URE of the conveying signal exceeds 5.73
     *  times the upper bound value of the current broadcast URA index, for more than 5.2 seconds, without an
     *  accompanying alert, is less than 1E-8 per hour.
     */
    bool b_integrity_status_flag;
    bool b_alert_flag;         //!< If true, indicates  that the SV URA may be worse than indicated in d_SV_accuracy, use that SV at our own risk.
    //bool b_antispoofing_flag;  //!<  If true, the AntiSpoofing mode is ON in that SV

    // clock terms
    // double d_master_clock;  // IRNSS transmission time
    double d_satClkCorr;  // IRNSS clock error
    double d_dtr;         // relativistic clock correction term
    double d_satClkDrift;

    // satellite positions
    double d_satpos_X;  //!< Earth-fixed coordinate x of the satellite [m]. Intersection of the IERS Reference Meridian (IRM) and the plane passing through the origin and normal to the Z-axis.
    double d_satpos_Y;  //!< Earth-fixed coordinate y of the satellite [m]. Completes a right-handed, Earth-Centered, Earth-Fixed orthogonal coordinate system.
    double d_satpos_Z;  //!< Earth-fixed coordinate z of the satellite [m]. The direction of the IERS (International Earth Rotation and Reference Systems Service) Reference Pole (IRP).

    // satellite identification info
    int32_t i_channel_ID;
    uint32_t i_satellite_PRN;

    // time synchro
    double d_subframe_timestamp_ms;  // [ms]

    // Ionospheric parameters
    bool flag_iono_valid;  //!< If set, it indicates that the ionospheric parameters are filled
    double d_alpha0;       //!< Coefficient 0 of a cubic equation representing the amplitude of the vertical delay [s]
    double d_alpha1;       //!< Coefficient 1 of a cubic equation representing the amplitude of the vertical delay [s/semi-circle]
    double d_alpha2;       //!< Coefficient 2 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^2]
    double d_alpha3;       //!< Coefficient 3 of a cubic equation representing the amplitude of the vertical delay [s(semi-circle)^3]
    double d_beta0;        //!< Coefficient 0 of a cubic equation representing the period of the model [s]
    double d_beta1;        //!< Coefficient 1 of a cubic equation representing the period of the model [s/semi-circle]
    double d_beta2;        //!< Coefficient 2 of a cubic equation representing the period of the model [s(semi-circle)^2]
    double d_beta3;        //!< Coefficient 3 of a cubic equation representing the period of the model [s(semi-circle)^3]

    // UTC parameters
    bool flag_utc_model_valid;  //!< If set, it indicates that the UTC model parameters are filled
    double d_A0;                //!< Constant of a model that relates IRNSS and UTC time [s]
    double d_A1;                //!< 1st order term of a model that relates IRNSS and UTC time [s/s]
    double d_A2;                //!< 2nd order term of a model that relates IRNSS and UTC time [s/s]
    int32_t d_t_OT;             //!< Reference time for UTC data [s]
    int32_t i_WN_T;             //!< UTC reference week number [weeks]
    int32_t d_DeltaT_LS;        //!< delta time due to leap seconds [s]. Number of leap seconds since 6-Jan-1980 as transmitted by the IRNSS almanac.
    int32_t i_WN_LSF;           //!< Week number at the end of which the leap second becomes effective [weeks]
    int32_t i_DN;               //!< Day number (IRNSS_DN) at the end of which the leap second becomes effective [days]
    int32_t d_DeltaT_LSF;       //!< Scheduled future or recent past (relative to NAV message upload) value of the delta time due to leap seconds [s]

    // Satellite velocity
    double d_satvel_X;  //!< Earth-fixed velocity coordinate x of the satellite [m]
    double d_satvel_Y;  //!< Earth-fixed velocity coordinate y of the satellite [m]
    double d_satvel_Z;  //!< Earth-fixed velocity coordinate z of the satellite [m]



    //Earth Orientation parameters
    int32_t d_t_EOP;
    double d_pm_x;
    double d_pm_x_dot;
    double d_pm_y;
    double d_pm_y_dot;
    double d_delta_UT1;
    double d_delta_UT1_dot;


    // public functions
    void reset();

private:
    uint64_t read_navigation_unsigned(std::bitset<IRNSS_L5_DATAFRAME_BITS> bits, const std::vector<std::pair<int32_t, int32_t>>& parameter);
    int64_t read_navigation_signed(std::bitset<IRNSS_L5_DATAFRAME_BITS> bits, const std::vector<std::pair<int32_t, int32_t>>& parameter);
    bool read_navigation_bool(std::bitset<IRNSS_L5_DATAFRAME_BITS> bits, const std::vector<std::pair<int32_t, int32_t>>& parameter);
    void print_irnss_word_bytes(uint32_t IRNSS_word);
};

#endif
