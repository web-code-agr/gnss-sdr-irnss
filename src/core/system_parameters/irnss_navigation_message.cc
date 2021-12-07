/*!
m * \file irnss_navigation_message.cc
 * \brief  Implementation of a IRNSS NAV Data message decoder
 *
 * 
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

#include "irnss_navigation_message.h"
#include "gnss_satellite.h"
#include <cmath>     // for fmod, abs, floor
#include <cstring>   // for memcpy
#include <iostream>  // for operator<<, cout, endl
#include <limits>    // for std::numeric_limits


void Irnss_Navigation_Message::reset()
{
    b_valid_ephemeris_set_flag = false;
    d_TOW = 0;
    d_TOW_SF1 = 0;
    d_TOW_SF2 = 0;
    d_TOW_SF3 = 0;
    // d_TOW_SF4 = 0;

    d_IODEC_SF1 = 0;
    d_IODEC_SF3 = 0;
    d_IODEC_SF4 = 0;
    d_Crs = 0.0;
    d_Delta_n = 0.0;
    d_M_0 = 0.0;
    d_Cuc = 0.0;
    d_e_eccentricity = 0.0;
    d_Cus = 0.0;
    d_sqrt_A = 0.0;
    d_Toe = 0;
    d_Toc = 0;
    d_Cic = 0.0;
    d_OMEGA0 = 0.0;
    d_Cis = 0.0;
    d_i_0 = 0.0;
    d_Crc = 0.0;
    d_OMEGA = 0.0;
    d_OMEGA_DOT = 0.0;
    d_IDOT = 0.0;
    i_code_on_L2 = 0;
    i_IRNSS_week = 0;
    b_L5_P_data_flag = false;
    b_S_P_data_flag = false;
    i_SV_accuracy = 0;
    i_SV_health = 0;
    d_TGD = 0.0;
    d_IODC = -1;
  //i_AODO = 0;

  //b_fit_interval_flag = false;
    d_spare1 = 0.0;
    d_spare2 = 0.0;

    d_A_f0 = 0.0;
    d_A_f1 = 0.0;
    d_A_f2 = 0.0;

    // clock terms
    // d_master_clock=0;
    d_dtr = 0.0;
    d_satClkCorr = 0.0;
    d_satClkDrift = 0.0;

    // satellite positions
    d_satpos_X = 0.0;
    d_satpos_Y = 0.0;
    d_satpos_Z = 0.0;

    // info
    i_channel_ID = 0;
    i_satellite_PRN = 0U;

    // time synchro
    d_subframe_timestamp_ms = 0.0;

    // flags
    b_alert_flag = false;
    b_integrity_status_flag = false;
  //b_antispoofing_flag = false;

    // Ionosphere and UTC
    flag_iono_valid = false;
    flag_utc_model_valid = false;
    d_alpha0 = 0.0;
    d_alpha1 = 0.0;
    d_alpha2 = 0.0;
    d_alpha3 = 0.0;
    d_beta0 = 0.0;
    d_beta1 = 0.0;
    d_beta2 = 0.0;
    d_beta3 = 0.0;
    d_A2 = 0.0;
    d_A1 = 0.0;
    d_A0 = 0.0;
    d_t_OT = 0;
    i_WN_T = 0;
    d_DeltaT_LS = 0;
    i_WN_LSF = 0;
    i_DN = 0;
    d_DeltaT_LSF = 0;

    // Almanac
    i_Toa = 0;
    i_WN_A = 0;
    for (int32_t i = 1; i < 32; i++)
        {
            almanacHealth[i] = 0;
        }

    // Satellite velocity
    d_satvel_X = 0.0;
    d_satvel_Y = 0.0;
    d_satvel_Z = 0.0;

    // Earth oreintation parameters
    d_t_EOP = 0;
    d_pm_x = 0.0;
    d_pm_x_dot = 0.0;
    d_pm_y = 0.0;
    d_pm_y_dot = 0.0;
    d_delta_UT1 = 0.0;
    d_delta_UT1_dot = 0.0;

    auto gnss_sat = Gnss_Satellite();
    std::string _system("IRNSS");
    for (uint32_t i = 1; i < 33; i++)
        {
            satelliteBlock[i] = gnss_sat.what_block(_system, i);
        }
}


Irnss_Navigation_Message::Irnss_Navigation_Message()
{
    reset();
}


void Irnss_Navigation_Message::print_irnss_word_bytes(uint32_t IRNSS_word)
{
    std::cout << " Word =";
    std::cout << std::bitset<32>(IRNSS_word);
    std::cout << std::endl;
}


bool Irnss_Navigation_Message::read_navigation_bool(std::bitset<IRNSS_L5_DATAFRAME_BITS> bits, const std::vector<std::pair<int32_t, int32_t>>& parameter)
{
    bool value;

    if (static_cast<int>(bits[IRNSS_L5_DATAFRAME_BITS - parameter[0].first]) == 1)
        {
            value = true;
        }
    else
        {
            value = false;
        }
    return value;
}


uint64_t Irnss_Navigation_Message::read_navigation_unsigned(std::bitset<IRNSS_L5_DATAFRAME_BITS> bits, const std::vector<std::pair<int32_t, int32_t>>& parameter)
{
    uint64_t value = 0ULL;
    int32_t num_of_slices = parameter.size();
    for (int32_t i = 0; i < num_of_slices; i++)
        {
            for (int32_t j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1;  // shift left
                    if (static_cast<int>(bits[IRNSS_L5_DATAFRAME_BITS - parameter[i].first - j]) == 1)
                        {
                            value += 1ULL;  // insert the bit
                        }
                }
        }
    return value;
}


int64_t Irnss_Navigation_Message::read_navigation_signed(std::bitset<IRNSS_L5_DATAFRAME_BITS> bits, const std::vector<std::pair<int32_t, int32_t>>& parameter)
{
    int64_t value = 0LL;
    int32_t num_of_slices = parameter.size();

    // read the MSB and perform the sign extension
    if (static_cast<int>(bits[IRNSS_L5_DATAFRAME_BITS - parameter[0].first]) == 1)
        {
            value ^= 0xFFFFFFFFFFFFFFFFLL;  // 64 bits variable
        }
    else
        {
            value &= 0LL;
        }

    for (int32_t i = 0; i < num_of_slices; i++)
        {
            for (int32_t j = 0; j < parameter[i].second; j++)
                {
                    value *= 2;                     // shift left the signed integer
                    value &= 0xFFFFFFFFFFFFFFFELL;  // reset the corresponding bit (for the 64 bits variable)
                    if (static_cast<int>(bits[IRNSS_L5_DATAFRAME_BITS - parameter[i].first - j]) == 1)
                        {
                            value += 1LL;  // insert the bit
                        }
                }
        }
    return value;
}


int32_t Irnss_Navigation_Message::subframe_decoder(std::string& subframe)
{
    int32_t subframe_ID = 0;
    uint32_t irnss_word;

    // UNPACK BYTES TO BITS AND REMOVE THE IRNSS_CRC REDUNDANCE
    std::bitset<IRNSS_L5_DATAFRAME_BITS> subframe_bits(subframe);
    // std::bitset<IRNSS_WORD_BITS + 2> word_bits;
    // for (int32_t i = 0; i < 10; i++)
    //     {
    //         memcpy(&irnss_word, &subframe[i * 4], sizeof(char) * 4);
    //         word_bits = std::bitset<(IRNSS_WORD_BITS + 2)>(irnss_word);
    //         for (int32_t j = 0; j < IRNSS_WORD_BITS; j++)
    //             {
    //                 subframe_bits[IRNSS_WORD_BITS * (9 - i) + j] = word_bits[j];
    //             }
    //     }
    // memcpy(&subframe_bits, &subframe);

    subframe_ID = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_SUBFRAME_ID));

    // Decode all 5 sub-frames
    switch (subframe_ID)
        {
        // --- Decode the sub-frame id -----------------------------------------
        // ICD (Section 6) https://www.isro.gov.in/sites/default/files/irnss_sps_icd_version1.1-2017.pdf
        case 1:
            // --- It is subframe 1 -------------------------------------
            // Compute the time of week (IRNSS_TOW) of the first sub-frames in the array ====
            // The transmitted IRNSS_TOW is actual IRNSS_TOW of the next subframe
            // (the variable subframe at this point contains bits of the last subframe).
            // IRNSS_TOW = bin2dec(subframe(9:25)) * 6;
            d_TOW_SF1 = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_TOW));
            // we are in the first subframe (the transmitted IRNSS_TOW is the start time of the next subframe) !
            d_TOW_SF1 = d_TOW_SF1 * 6;
            d_TOW = d_TOW_SF1;  // Set transmission time
            
            b_alert_flag = read_navigation_bool(subframe_bits, IRNSS_ALERT_FLAG);
            
            i_IRNSS_week = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_WEEK));
            d_A_f0 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_A_F0));
            d_A_f0 = d_A_f0 * IRNSS_A_F0_LSB;
            d_A_f1 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_A_F1));
            d_A_f1 = d_A_f1 * IRNSS_A_F1_LSB;
            d_A_f2 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_A_F2));
            d_A_f2 = d_A_f2 * IRNSS_A_F2_LSB;
            i_SV_accuracy = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_SV_ACCURACY));
            d_Toc = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_T_OC));
            d_Toc = d_Toc * IRNSS_T_OC_LSB;
            d_TGD = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_T_GD));
            d_TGD = d_TGD * IRNSS_T_GD_LSB;
            d_Delta_n = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_DELTA_N));
            d_Delta_n = d_Delta_n * IRNSS_DELTA_N_LSB;
            d_IODEC_SF1 = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_IODC));
            b_L5_P_data_flag = read_navigation_bool(subframe_bits, IRNSS_L5_FLAG);
            b_S_P_data_flag = read_navigation_bool(subframe_bits, IRNSS_S_FLAG); 
            d_Cus = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_C_US));
            d_Cus = d_Cus * IRNSS_C_US_LSB;
            d_Cuc = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_C_UC));
            d_Cuc = d_Cuc * IRNSS_C_UC_LSB;
            d_Cic = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_C_IC));
            d_Cic = d_Cic * IRNSS_C_IC_LSB;
            d_Cis = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_C_IS));
            d_Cis = d_Cis * IRNSS_C_IS_LSB;
            d_Crc = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_C_RC));
            d_Crc = d_Crc * IRNSS_C_RC_LSB;
            d_Crs = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_C_RS));
            d_Crs = d_Crs * IRNSS_C_RS_LSB;
            d_IDOT = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_I_DOT));
            d_IDOT = d_IDOT * IRNSS_I_DOT_LSB;

            
              // 
            //i_SV_health = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, SV_HEALTH));
             //
            
            // i_code_on_S = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, L5_OR_P_ON_S));
            
            // d_IODEC = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IODEC));
            
            
            
            
            
            

            break;

        case 2:  // --- It is subframe 2 -------------------
            d_TOW_SF2 = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_TOW));
            d_TOW_SF2 = d_TOW_SF2 * 6;
            d_TOW = d_TOW_SF2;  // Set transmission time
            b_alert_flag = read_navigation_bool(subframe_bits, IRNSS_ALERT_FLAG);
            d_M_0 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_M_0));
            d_M_0 = d_M_0 * IRNSS_M_0_LSB;
            d_Toe = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_T_OE));
            d_Toe = d_Toe * IRNSS_T_OE_LSB;
            d_e_eccentricity = static_cast<double>(read_navigation_unsigned(subframe_bits, IRNSS_ECCENTRICITY));
            d_e_eccentricity = d_e_eccentricity * IRNSS_ECCENTRICITY_LSB;
            d_sqrt_A = static_cast<double>(read_navigation_unsigned(subframe_bits, IRNSS_SQRT_A));
            d_sqrt_A = d_sqrt_A * IRNSS_SQRT_A_LSB;
            d_OMEGA0 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_OMEGA_0));
            d_OMEGA0 = d_OMEGA0 * IRNSS_OMEGA_0_LSB;
            d_OMEGA = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_OMEGA));
            d_OMEGA = d_OMEGA * IRNSS_OMEGA_LSB;
            d_OMEGA_DOT = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_OMEGA_DOT));
            d_OMEGA_DOT = d_OMEGA_DOT * IRNSS_OMEGA_DOT_LSB;
            d_i_0 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_I_0));
            d_i_0 = d_i_0 * IRNSS_I_0_LSB;
            break;

        case 3:
        case 4:  // --- It is subframe 4 o 5---------- Almanac, ionospheric model, UTC parameters, SV health
            int32_t d_prn_ID;
            int32_t d_msg_ID;
            d_TOW_SF4 = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_TOW));
            d_TOW_SF4 = d_TOW_SF4 * 6;
            d_TOW = d_TOW_SF4;  // Set transmission time
            //b_integrity_status_flag = read_navigation_bool(subframe_bits, INTEGRITY_STATUS_FLAG);
            b_alert_flag = read_navigation_bool(subframe_bits, IRNSS_ALERT_FLAG);
            //b_antispoofing_flag = read_navigation_bool(subframe_bits, ANTI_SPOOFING_FLAG);
            d_prn_ID = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_SV_PAGE));
            d_msg_ID = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_SV_DATA_ID));
            
                

            if (d_msg_ID == 5)  // ionospheric grid based parameters for Indian region.
                {
                    //! \TODO read section 6.1.3.1

                }


            if (d_msg_ID == 7) //almanac
                {
                    //TODO
                }

            if (d_msg_ID == 9)  // irnss time offset with utc and gps
                {
                    // UTC data
                    
                    d_A2 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_A_2));
                    d_A2 = d_A2 * IRNSS_A_2_LSB;
                    d_A1 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_A_1));
                    d_A1 = d_A1 * IRNSS_A_1_LSB;
                    d_A0 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_A_0));
                    d_A0 = d_A0 * IRNSS_A_0_LSB;
                    d_t_OT = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_T_OT));
                    d_t_OT = d_t_OT * IRNSS_T_OT_LSB;
                    i_WN_T = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_WN_T));
                    d_DeltaT_LS = static_cast<int32_t>(read_navigation_signed(subframe_bits, IRNSS_DELTAT_LS));
                    i_WN_LSF = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_WN_LSF));
                    i_DN = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_DN));  // Right-justified ?
                    d_DeltaT_LSF = static_cast<int32_t>(read_navigation_signed(subframe_bits, IRNSS_DELTAT_LSF));
                    flag_utc_model_valid = true;

                    // TODO GPS offset parameters
                }
            if (d_msg_ID == 11) //Earth observation parameters and ionospheric coefficients
                {
                    //Earth orientation parameters
                    d_t_EOP = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_T_EOP));
                    d_t_EOP = d_t_EOP * IRNSS_T_EOP_LSB;
                    d_pm_x = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_PM_X));
                    d_pm_x = d_pm_x * IRNSS_PM_X_LSB;
                    d_pm_x_dot = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_PM_X_DOT));
                    d_pm_x_dot = d_pm_x_dot * IRNSS_PM_X_DOT_LSB;
                    d_pm_y = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_PM_Y));
                    d_pm_y = d_pm_y * IRNSS_PM_Y_LSB;
                    d_pm_y_dot = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_PM_Y_DOT));
                    d_pm_y_dot = d_pm_y_dot * IRNSS_PM_Y_DOT_LSB;
                    d_delta_UT1 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_DELTA_UT1));
                    d_delta_UT1 = d_delta_UT1 * IRNSS_DELTA_UT1_LSB;
                    d_delta_UT1_dot = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_DELTA_UT1_DOT));
                    d_delta_UT1_dot = d_delta_UT1_dot * IRNSS_DELTA_UT1_DOT_LSB;

                    // Ionospheric coefficients
                    d_alpha0 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_ALPHA_0));
                    d_alpha0 = d_alpha0 * IRNSS_ALPHA_0_LSB;
                    d_alpha1 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_ALPHA_1));
                    d_alpha1 = d_alpha1 * IRNSS_ALPHA_1_LSB;
                    d_alpha2 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_ALPHA_2));
                    d_alpha2 = d_alpha2 * IRNSS_ALPHA_2_LSB;
                    d_alpha3 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_ALPHA_3));
                    d_alpha3 = d_alpha3 * IRNSS_ALPHA_3_LSB;
                    d_beta0 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_BETA_0));
                    d_beta0 = d_beta0 * IRNSS_BETA_0_LSB;
                    d_beta1 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_BETA_1));
                    d_beta1 = d_beta1 * IRNSS_BETA_1_LSB;
                    d_beta2 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_BETA_2));
                    d_beta2 = d_beta2 * IRNSS_BETA_2_LSB;
                    d_beta3 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_BETA_3));
                    d_beta3 = d_beta3 * IRNSS_BETA_3_LSB;
                    flag_iono_valid = true;

                }

            if (d_msg_ID == 14)  // Refer section 6.1.3.5
                {
                    // 
                    //! \TODO 
                }
            

            if (d_msg_ID == 18)
                {
                    // TODO Refer Sec 6.1.3.6
                }

            if (d_msg_ID == 26)
                {   
                    d_A2 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_A_2));
                    d_A2 = d_A2 * IRNSS_A_2_LSB;
                    d_A1 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_A_1));
                    d_A1 = d_A1 * IRNSS_A_1_LSB;
                    d_A0 = static_cast<double>(read_navigation_signed(subframe_bits, IRNSS_A_0));
                    d_A0 = d_A0 * IRNSS_A_0_LSB;
                    d_t_OT = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_T_OT));
                    d_t_OT = d_t_OT * IRNSS_T_OT_LSB;
                    i_WN_T = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_WN_T));
                    d_DeltaT_LS = static_cast<int32_t>(read_navigation_signed(subframe_bits, IRNSS_DELTAT_LS));
                    i_WN_LSF = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_WN_LSF));
                    i_DN = static_cast<int32_t>(read_navigation_unsigned(subframe_bits, IRNSS_DN));  // Right-justified ?
                    d_DeltaT_LSF = static_cast<int32_t>(read_navigation_signed(subframe_bits, IRNSS_DELTAT_LSF));
                    // flag_iono_valid = true;
                    // flag_utc_model_valid = true;
                }

            //         // TODO IRNSS- Other GNSS parameters
            //     }

            if (d_msg_ID == 0)
                {
                    //null message Refer 6.1.3.8
                }




            break;


        default:
            break;
          // switch subframeID ...
        }

    return subframe_ID;
}


double Irnss_Navigation_Message::utc_time(const double irnsstime_corrected)
{
    double t_utc;
    double t_utc_daytime;
    double Delta_t_UTC = d_DeltaT_LS + d_A0 + d_A1 * (irnsstime_corrected - d_t_OT + 604800 * static_cast<double>((i_IRNSS_week - i_WN_T)));

    // Determine if the effectivity time of the leap second event is in the past
    int32_t weeksToLeapSecondEvent = i_WN_LSF - i_IRNSS_week;

    if ((weeksToLeapSecondEvent) >= 0)  // is not in the past
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
                            /* 
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
                            /*
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
                            Delta_t_UTC = d_DeltaT_LSF + d_A0 + d_A1 * (irnsstime_corrected - d_t_OT + 604800 * static_cast<double>((i_IRNSS_week - i_WN_T)));
                            t_utc_daytime = fmod(irnsstime_corrected - Delta_t_UTC, 86400);
                        }
                }
        }
    else  // the effectivity time is in the past
        {
            /* 
             * Whenever the effectivity time of the leap second event, as indicated by the
             * WNLSF and IRNSS_DN values, is in the "past" (relative to the user's current time),
             * and the user�s current time does not fall in the time span as given above
             * in 20.3.3.5.2.4b,*/
            Delta_t_UTC = d_DeltaT_LSF + d_A0 + d_A1 * (irnsstime_corrected - d_t_OT + 604800 * static_cast<double>((i_IRNSS_week - i_WN_T)));
            t_utc_daytime = fmod(irnsstime_corrected - Delta_t_UTC, 86400);
        }

    double secondsOfWeekBeforeToday = 43200 * floor(irnsstime_corrected / 43200);
    t_utc = secondsOfWeekBeforeToday + t_utc_daytime;
    return t_utc;
}


Irnss_Ephemeris Irnss_Navigation_Message::get_ephemeris()
{
    Irnss_Ephemeris ephemeris;
    ephemeris.i_satellite_PRN = i_satellite_PRN;
    ephemeris.d_TOW = d_TOW;
    ephemeris.d_Crs = d_Crs;
    ephemeris.d_Delta_n = d_Delta_n;
    ephemeris.d_M_0 = d_M_0;
    ephemeris.d_Cuc = d_Cuc;
    ephemeris.d_e_eccentricity = d_e_eccentricity;
    ephemeris.d_Cus = d_Cus;
    ephemeris.d_sqrt_A = d_sqrt_A;
    ephemeris.d_Toe = d_Toe;
    ephemeris.d_Toc = d_Toc;
    ephemeris.d_Cic = d_Cic;
    ephemeris.d_OMEGA0 = d_OMEGA0;
    ephemeris.d_Cis = d_Cis;
    ephemeris.d_i_0 = d_i_0;
    ephemeris.d_Crc = d_Crc;
    ephemeris.d_OMEGA = d_OMEGA;
    ephemeris.d_OMEGA_DOT = d_OMEGA_DOT;
    ephemeris.d_IDOT = d_IDOT;
    ephemeris.i_code_on_L2 = i_code_on_L2;
    ephemeris.i_IRNSS_week = i_IRNSS_week;
    ephemeris.b_L5_P_data_flag = b_L5_P_data_flag;
    ephemeris.i_SV_accuracy = i_SV_accuracy;
    ephemeris.i_SV_health = i_SV_health;
    ephemeris.d_TGD = d_TGD;
    ephemeris.d_IODC = d_IODC;
    // ephemeris.d_IODE_SF2 = d_IODE_SF2;
    // ephemeris.d_IODE_SF3 = d_IODE_SF3;
    // ephemeris.i_AODO = i_AODO;
    // ephemeris.b_fit_interval_flag = b_fit_interval_flag;
    ephemeris.d_spare1 = d_spare1;
    ephemeris.d_spare2 = d_spare2;
    ephemeris.d_A_f0 = d_A_f0;
    ephemeris.d_A_f1 = d_A_f1;
    ephemeris.d_A_f2 = d_A_f2;
    ephemeris.b_integrity_status_flag = b_integrity_status_flag;
    ephemeris.b_alert_flag = b_alert_flag;
    // ephemeris.b_antispoofing_flag = b_antispoofing_flag;
    ephemeris.d_satClkDrift = d_satClkDrift;
    ephemeris.d_dtr = d_dtr;
    ephemeris.d_satpos_X = d_satpos_X;
    ephemeris.d_satpos_Y = d_satpos_Y;
    ephemeris.d_satpos_Z = d_satpos_Z;
    ephemeris.d_satvel_X = d_satvel_X;
    ephemeris.d_satvel_Y = d_satvel_Y;
    ephemeris.d_satvel_Z = d_satvel_Z;

    return ephemeris;
}


Irnss_Iono Irnss_Navigation_Message::get_iono()
{
    Irnss_Iono iono;
    iono.d_alpha0 = d_alpha0;
    iono.d_alpha1 = d_alpha1;
    iono.d_alpha2 = d_alpha2;
    iono.d_alpha3 = d_alpha3;
    iono.d_beta0 = d_beta0;
    iono.d_beta1 = d_beta1;
    iono.d_beta2 = d_beta2;
    iono.d_beta3 = d_beta3;
    iono.valid = flag_iono_valid;
    // WARNING: We clear flag_utc_model_valid in order to not re-send the same information to the ionospheric parameters queue
    flag_iono_valid = false;
    return iono;
}


Irnss_Utc_Model Irnss_Navigation_Message::get_utc_model()
{
    Irnss_Utc_Model utc_model;
    utc_model.valid = flag_utc_model_valid;
    // UTC parameters
    utc_model.d_A1 = d_A1;
    utc_model.d_A0 = d_A0;
    utc_model.d_t_OT = d_t_OT;
    utc_model.i_WN_T = i_WN_T;
    utc_model.d_DeltaT_LS = d_DeltaT_LS;
    utc_model.i_WN_LSF = i_WN_LSF;
    utc_model.i_DN = i_DN;
    utc_model.d_DeltaT_LSF = d_DeltaT_LSF;
    // warning: We clear flag_utc_model_valid in order to not re-send the same information to the ionospheric parameters queue
    flag_utc_model_valid = false;
    return utc_model;
}


bool Irnss_Navigation_Message::satellite_validation()
{
    bool flag_data_valid = false;
    b_valid_ephemeris_set_flag = false;

    // First Step:
    // check Issue Of Ephemeris Data (IODE IRNSS_IODC..) to find a possible interrupted reception
    // and check if the data have been filled (!=0)
    if (d_TOW_SF1 != 0.0 and d_TOW_SF2 != 0.0 and d_TOW_SF3 != 0.0)
        {
            // if (d_IODE_SF2 == d_IODE_SF3 and d_IODC == d_IODE_SF2 and d_IODC != -1.0)
            //     {
                    flag_data_valid = true;
                    b_valid_ephemeris_set_flag = true;
                }
        // }
    return flag_data_valid;
}
