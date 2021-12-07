/*!
 * \file IRNSS_at_1.h
 * \brief  Defines system parameters for IRNSS L5 signal
 * \
 *
 * -------------------------------------------------------------------------
 * \link to track sheet:
 * \
 *
 * 
 *          
 *
 * 
 *
 * 
 * 
 * 
 * 
 *
 * 
 * 
 * 
 * 
 *
 * 
 * 
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_IRNSS_L5_H_
#define GNSS_SDR_IRNSS_L5_H_

// #include "GPS_CNAV.h"
#include "MATH_CONSTANTS.h"
#include "gnss_frequencies.h"
#include <cstdint>
#include <string>
#include <vector>


// Physical constants
const double IRNSS_L5_C_M_S = 299792458.0;                //!< The speed of light, [m/s]
const double IRNSS_L5_C_M_MS = 299792.4580;               //!< The speed of light, [m/ms]
const double IRNSS_L5_PI = 3.1415926535898;               //!< Pi as defined in IS-GPS-200E
const double IRNSS_L5_TWO_PI = 6.283185307179586;         //!< 2Pi as defined in IS-GPS-200E
const double IRNSS_L5_OMEGA_EARTH_DOT = 7.2921151467e-5;  //!< Earth rotation rate, [rad/s]
const double IRNSS_L5_GM = 3.986005e14;                   //!< Universal gravitational constant times the mass of the Earth, [m^3/s^2]
const double IRNSS_L5_F = -4.442807633e-10;               //!< Constant, [s/(m)^(1/2)]

// carrier and code frequencies
const double IRNSS_L5_FREQ_HZ = FREQ5;  //!< L5 [Hz]

const double IRNSS_L5I_CODE_RATE_HZ = 1.023e6;      //!< IRNSS L5i code rate [chips/s]
const int32_t IRNSS_L5I_CODE_LENGTH_CHIPS = 1023;  //!< IRNSS L5i  code length [chips]
const double IRNSS_L5I_CHIP_PERIOD_S = 0.001;              //!< IRNSS L5 code period [seconds]
const int32_t IRNSS_L5I_PERIOD_MS = 1;              //!< IRNSS L5 code period [ms]
const double IRNSS_L5I_SYMBOL_PERIOD = 0.02;        //!< GPS L5 symbol period [seconds]
const int32_t IRNSS_L5I_SYMBOL_PERIOD_MS = 20;      //!< GPS L5 symbol period [ms]

const double IRNSS_L5Q_CODE_RATE_HZ = 10.23e6;      //!< GPS L5i code rate [chips/s]
const int32_t IRNSS_L5Q_CODE_LENGTH_CHIPS = 1023;  //!< GPS L5i code length [chips]
const double IRNSS_L5Q_PERIOD = 0.001;              //!< GPS L5 code period [seconds]

//const int32_t GPS_L5_HISTORY_DEEP = 5;

//optimum parameters
const uint32_t IRNSS_L5_OPT_ACQ_FS_HZ = 2000000;  //!< Sampling frequency that maximizes the acquisition SNR while using a non-multiple of chip rate

const int32_t IRNSS_L5I_INIT_REG[14] =
	{935, 38, 564, 370,
		944, 107, 20, 304,
		152, 868, 76, 892,
		722, 490};


const int32_t IRNSS_NAV_INTERLEAVER_ROWS = 8;
const int32_t IRNSS_NAV_INTERLEAVER_COLS = 73;

constexpr double IRNSS_L5_PREAMBLE_DURATION_S = 0.32;
constexpr int32_t IRNSS_L5_PREAMBLE_LENGTH_BITS = 16;
constexpr int32_t IRNSS_L5_PREAMBLE_LENGTH_SYMBOLS = 320;
constexpr int32_t IRNSS_L5_PREAMBLE_DURATION_MS = 320;
constexpr uint32_t IRNSS_L5_BIT_PERIOD_MS = 20U; 
constexpr int32_t IRNSS_L5_TELEMETRY_RATE_BITS_SECOND = 25;  //!< NAV message bit rate [bits/s]
constexpr int32_t IRNSS_L5_TELEMETRY_SYMBOLS_PER_BIT = 2;
constexpr int32_t IRNSS_L5_TELEMETRY_RATE_SYMBOLS_SECOND = IRNSS_L5_TELEMETRY_RATE_BITS_SECOND * IRNSS_L5_TELEMETRY_SYMBOLS_PER_BIT;  //!< NAV message bit rate [symbols/s]
constexpr int32_t IRNSS_L5_WORD_LENGTH = 4;                                                                                          //!< IRNSS_CRC + IRNSS_L5 WORD (-2 -1 0 ... 29) Bits = 4 bytes
constexpr int32_t IRNSS_L5_SUBFRAME_LENGTH = 40;                                                                                     //!< IRNSS_L5_WORD_LENGTH x 10 = 40 bytes
constexpr int32_t IRNSS_L5_SUBFRAME_BITS = 600; 
constexpr int32_t IRNSS_L5_DATAFRAME_BITS = 292;                                                                                        //!< Number of bits per subframe in the NAV message [bits]
constexpr int32_t IRNSS_L5_SUBFRAME_SECONDS = 12;                                                                                     //!< Subframe duration [seconds]
constexpr int32_t IRNSS_L5_SUBFRAME_MS = 12000;                                                                                       //!< Subframe duration [seconds]
constexpr int32_t IRNSS_L5_WORD_BITS = 30;                                                                                           //!< Number of bits per word in the NAV message [bits]
constexpr char IRNSS_L5_PREAMBLE[17] = "1110101110010000";
// constexpr char IRNSS_L5_PREAMBLE_SYMBOLS_STR[321] = "11111111111111111111111111111111111111111111111111111111111100000000000000000000111111111111111111110000000000000000000011111111111111111111111111111111111111111111111111111111111100000000000000000000000000000000000000001111111111111111111100000000000000000000000000000000000000000000000000000000000000000000000000000000";
constexpr char IRNSS_L5_PREAMBLE_SYMBOLS_STR[33] = "11111100110011111100001100000000";
// GPS NAVIGATION MESSAGE STRUCTURE
// NAVIGATION MESSAGE FIELDS POSITIONS (from IS-GPS-200K Appendix II)

// SUBFRAME 1-5 (TLM and HOW)

const std::vector<std::pair<int32_t, int32_t>> IRNSS_TOW({{9, 17}});
const std::vector<std::pair<int32_t, int32_t>> IRNSS_ALERT_FLAG({{26, 1}});
const std::vector<std::pair<int32_t, int32_t>> IRNSS_AUTO_NAV({{27, 1}});
const std::vector<std::pair<int32_t, int32_t>> IRNSS_SUBFRAME_ID({{28, 2}});
const std::vector<std::pair<int32_t, int32_t>> IRNSS_CRC({{263, 24}});
const std::vector<std::pair<int32_t, int32_t>> IRNSS_TAIL({{287, 6}});

// SUBFRAME 1
const std::vector<std::pair<int32_t, int32_t>> IRNSS_WEEK({{31, 10}});
const std::vector<std::pair<int32_t, int32_t>> IRNSS_A_F0({{41, 22}});
constexpr double IRNSS_A_F0_LSB = TWO_N31;  
const std::vector<std::pair<int32_t, int32_t>> IRNSS_A_F1({{63, 16}});
constexpr double IRNSS_A_F1_LSB = TWO_N43;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_A_F2({{79, 8}});
constexpr double IRNSS_A_F2_LSB = TWO_N55;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_SV_ACCURACY({{87, 4}});
const std::vector<std::pair<int32_t, int32_t>> IRNSS_T_OC({{91, 16}});
constexpr int32_t IRNSS_T_OC_LSB = static_cast<int32_t>(TWO_P4);
const std::vector<std::pair<int32_t, int32_t>> IRNSS_T_GD({{107, 8}});
constexpr double IRNSS_T_GD_LSB = TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_DELTA_N({{115, 22}});
constexpr double IRNSS_DELTA_N_LSB = PI_TWO_N41;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_IODC({{137, 8}});
const std::vector<std::pair<int32_t, int32_t>> IRNSS_L5_FLAG({{155, 1}});
const std::vector<std::pair<int32_t, int32_t>> IRNSS_S_FLAG({{156, 1}});

const std::vector<std::pair<int32_t, int32_t>> IRNSS_C_UC({{157, 15}});
constexpr double IRNSS_C_UC_LSB = TWO_N28;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_C_US({{172, 15}});
constexpr double IRNSS_C_US_LSB = TWO_N28;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_C_IC({{187, 15}});
constexpr double IRNSS_C_IC_LSB = TWO_N28;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_C_IS({{202, 15}});
constexpr double IRNSS_C_IS_LSB = TWO_N28;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_C_RC({{217, 15}});
constexpr double IRNSS_C_RC_LSB = TWO_N4;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_C_RS({{232, 15}});
constexpr double IRNSS_C_RS_LSB = TWO_N4;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_I_DOT({{247, 14}});
constexpr double IRNSS_I_DOT_LSB = PI_TWO_N43;


// SUBFRAME 2
const std::vector<std::pair<int32_t, int32_t>> IRNSS_M_0({{31, 32}});
constexpr double IRNSS_M_0_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_T_OE({{63, 16}});
constexpr int32_t IRNSS_T_OE_LSB = static_cast<int32_t>(TWO_P4);
const std::vector<std::pair<int32_t, int32_t>> IRNSS_ECCENTRICITY({{79, 32}});
constexpr double IRNSS_ECCENTRICITY_LSB = TWO_N33;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_SQRT_A({{111, 32}});
constexpr double IRNSS_SQRT_A_LSB = TWO_N19;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_OMEGA_0({{143, 32}});
constexpr double IRNSS_OMEGA_0_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_OMEGA({{143, 32}});
constexpr double IRNSS_OMEGA_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_OMEGA_DOT({{207, 22}});
constexpr double IRNSS_OMEGA_DOT_LSB = PI_TWO_N41;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_I_0({{229, 32}});
constexpr double IRNSS_I_0_LSB = PI_TWO_N31;


// SUBFRAME 3-4
const std::vector<std::pair<int32_t, int32_t>> IRNSS_SV_DATA_ID({{31, 6}});
const std::vector<std::pair<int32_t, int32_t>> IRNSS_SV_PAGE({{257, 6}});


// SUBFRAME 3-4
// MESSAGE TYPE-9
const std::vector<std::pair<int32_t, int32_t>> IRNSS_A_0({{37, 16}});
constexpr double IRNSS_A_0_LSB = TWO_N35;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_A_1({{53, 13}});
constexpr double IRNSS_A_1_LSB = TWO_N51;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_A_2({{66, 7}});
constexpr double IRNSS_A_2_LSB = TWO_N68;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_DELTAT_LS({{73, 8}});
constexpr double IRNSS_DELTAT_LS_LSB = 1;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_T_OT({{81, 16}});
constexpr double IRNSS_T_OT_LSB = TWO_P4;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_WN_T({{97, 10}});
constexpr double IRNSS_WN_T_LSB = 1;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_WN_LSF({{107, 10}});
constexpr double IRNSS_WN_LSF_LSB = 1;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_DN({{117, 4}});
constexpr double IRNSS_DN_LSB = 1;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_DELTAT_LSF({{121, 8}});
constexpr double IRNSS_DELTAT_LSF_LSB = 1;

// MESSAGE TYPE-11
const std::vector<std::pair<int32_t, int32_t>> IRNSS_T_EOP({{37, 16}});
constexpr double IRNSS_T_EOP_LSB = TWO_P4;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_PM_X({{53, 21}});
constexpr double IRNSS_PM_X_LSB = TWO_N20;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_PM_X_DOT({{74, 15}});
constexpr double IRNSS_PM_X_DOT_LSB = TWO_N21;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_PM_Y({{89, 21}});
constexpr double IRNSS_PM_Y_LSB = TWO_N20;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_PM_Y_DOT({{110, 15}});
constexpr double IRNSS_PM_Y_DOT_LSB = TWO_N21;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_DELTA_UT1({{125, 31}});
constexpr double IRNSS_DELTA_UT1_LSB = TWO_N24;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_DELTA_UT1_DOT({{156, 19}});
constexpr double IRNSS_DELTA_UT1_DOT_LSB = TWO_N25;



const std::vector<std::pair<int32_t, int32_t>> IRNSS_ALPHA_0({{175, 8}});
constexpr double IRNSS_ALPHA_0_LSB = TWO_N30;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_ALPHA_1({{183, 8}});
constexpr double IRNSS_ALPHA_1_LSB = TWO_N27;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_ALPHA_2({{191, 8}});
constexpr double IRNSS_ALPHA_2_LSB = TWO_N24;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_ALPHA_3({{199, 8}});
constexpr double IRNSS_ALPHA_3_LSB = TWO_N24;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_BETA_0({{207, 8}});
constexpr double IRNSS_BETA_0_LSB = TWO_P11;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_BETA_1({{215, 8}});
constexpr double IRNSS_BETA_1_LSB = TWO_P14;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_BETA_2({{223, 8}});
constexpr double IRNSS_BETA_2_LSB = TWO_P16;
const std::vector<std::pair<int32_t, int32_t>> IRNSS_BETA_3({{231, 8}});
constexpr double IRNSS_BETA_3_LSB = TWO_P16;







// // SUBFRAME 4
// //! \todo read all pages of subframe 4
// // Page 18 - Ionospheric and UTC data







// // Page 25 - Antispoofing, SV config and SV health (PRN 25 -32)
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV25({{229, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV26({{241, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV27({{247, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV28({{253, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV29({{259, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV30({{271, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV31({{277, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV32({{283, 6}});


// // SUBFRAME 5
// //! \todo read all pages of subframe 5

// // page 25 - Health (PRN 1 - 24)
// const std::vector<std::pair<int32_t, int32_t>> T_OA({{69, 8}});
// constexpr int32_t T_OA_LSB = TWO_P12;
// const std::vector<std::pair<int32_t, int32_t>> WN_A({{77, 8}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV1({{91, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV2({{97, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV3({{103, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV4({{109, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV5({{121, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV6({{127, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV7({{133, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV8({{139, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV9({{151, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV10({{157, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV11({{163, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV12({{169, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV13({{181, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV14({{187, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV15({{193, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV16({{199, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV17({{211, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV18({{217, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV19({{223, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV20({{229, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV21({{241, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV22({{247, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV23({{253, 6}});
// const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV24({{259, 6}});


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L1_CA_H
