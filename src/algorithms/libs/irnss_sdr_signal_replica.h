/*!
 * \file irnss_sdr_signal_processing.h
 * \brief This class implements various functions for IRNSS L5 SPS signals
 * 
 *
 * 
 *
 * -------------------------------------------------------------------------
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
 * 
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_IRNSS_SDR_SIGNAL_REPLICA_H_
#define GNSS_SDR_IRNSS_SDR_SIGNAL_REPLICA_H_

#include <gsl/gsl>
#include <complex>
#include <cstdint>

//! Generates int IRNSS L5 SPS code for the desired SV ID and code shift
void irnss_l5_sps_code_gen_int(gsl::span<int32_t> _dest, int32_t _prn, uint32_t _chip_shift);

//! Generates float IRNSS L5 SPS code for the desired SV ID and code shift
void irnss_l5_sps_gen_float(gsl::span<float> _dest, int32_t _prn, uint32_t _chip_shift);

//! Generates complex IRNSS L5 SPS code for the desired SV ID and code shift, and sampled to specific sampling frequency
void irnss_l5_sps_code_gen_complex(gsl::span<std::complex<float>> _dest, int32_t _prn, uint32_t _chip_shift);

//! Generates N complex IRNSS L5 SPS codes for the desired SV ID and code shift
// void irnss_l5_sps_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs, uint32_t _chip_shift, uint32_t _ncodes);

//! Generates complex IRNSS L5 SPS code for the desired SV ID and code shift
void irnss_l5_sps_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs, uint32_t _chip_shift);

#endif /* GNSS_SDR_IRNSS_SDR_SIGNAL_REPLICA_H_ */
