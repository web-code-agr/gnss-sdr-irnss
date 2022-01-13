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

#include "irnss_sdr_signal_replica.h"
#include <array>
#include <bitset>

auto auxCeil = [](float x) { return static_cast<int32_t>(static_cast<int64_t>((x) + 1)); };

void irnss_l5_sps_code_gen_int(gsl::span<int32_t> _dest, int32_t _prn, uint32_t _chip_shift)
{
    const uint32_t _code_length = 1023;
    std::bitset<_code_length> G1{};
    std::bitset<_code_length> G2{};
    std::bitset<10> G1_register{};
    std::bitset<10> G2_register{};
    bool feedback1, feedback2;
    bool aux;
    uint32_t lcv, lcv2;
    uint32_t delay;
    int32_t prn_idx;

    // G2 Delays as defined in IRNSS doc (Pg 10-11): "Initial state of G2 provides chip delay"
    const std::array<int32_t,10> delays = {935 /*PRN1*/, 38, 564, 370, 944, 107, 20, 304, 152, 868 /*, 76, 892, 722, 490 PRN14*/};

    // compute delay array index for given PRN number 
    prn_idx = _prn - 1;

    // A simple error check
    if ((prn_idx < 0) || (prn_idx > 14))
        {
            return;
        }

    for (lcv = 0; lcv < 10; lcv++)
        {
            G1_register[lcv] = true;
            G2_register[lcv] = true;
        }

    // Generate G1 & G2 Register
    for (lcv = 0; lcv < _code_length; lcv++)
        {
            G1[lcv] = G1_register[0];
            G2[lcv] = G2_register[0];

            feedback1 = G1_register[7] xor G1_register[0];
            feedback2 = G2_register[8] xor G2_register[7] xor G2_register[4] xor G2_register[2] xor G2_register[1] xor G2_register[0];

            for (lcv2 = 0; lcv2 < 9; lcv2++)
                {
                    G1_register[lcv2] = G1_register[lcv2 + 1];
                    G2_register[lcv2] = G2_register[lcv2 + 1];
                }

            G1_register[9] = feedback1;
            G2_register[9] = feedback2;
        }

    // Set the delay
    delay = _code_length - delays[prn_idx];
    delay += _chip_shift;
    delay %= _code_length;

    // Generate PRN from G1 and G2 Registers
    for (lcv = 0; lcv < _code_length; lcv++)
        {
            aux = G1[(lcv + _chip_shift) % _code_length] xor G2[delay];
            if (aux == true)
                {
                    _dest[lcv] = 1;
                }
            else
                {
                    _dest[lcv] = -1;
                }
            delay++;
            delay %= _code_length;
        }
}


void irnss_l5_sps_gen_float(gsl::span<float> _dest, int32_t _prn, uint32_t _chip_shift)
{
    const uint32_t _code_length = 1023;
    std::array<int32_t, _code_length> ca_code_int{};

    irnss_l5_sps_code_gen_int(ca_code_int, _prn, _chip_shift);

    for (uint32_t ii = 0; ii < _code_length; ++ii)
        {
            _dest[ii] = static_cast<float>(ca_code_int[ii]);
        }
}


void irnss_l5_sps_code_gen_complex(gsl::span<std::complex<float>> _dest, int32_t _prn, uint32_t _chip_shift)
{
    const uint32_t _code_length = 1023;
    std::array<int32_t, _code_length> ca_code_int{};

    irnss_l5_sps_code_gen_int(ca_code_int, _prn, _chip_shift);

    for (uint32_t ii = 0; ii < _code_length; ++ii)
        {
            _dest[ii] = std::complex<float>(static_cast<float>(ca_code_int[ii]), 0.0F);
        }
}


/*
 *  Generates complex IRNSS L5 SPS code for the desired SV ID and sampled to specific sampling frequency
 *  NOTICE: the number of samples is rounded towards zero (integer truncation)
 */
void irnss_l5_sps_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs, uint32_t _chip_shift)
{
    // This function is based on the GNU software GPS for MATLAB in the Kay Borre book
    std::array<std::complex<float>, 1023> _code{};
    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    float aux;
    const int32_t _codeFreqBasis = 1023000;  // Hz
    const int32_t _codeLength = 1023;

    // --- Find number of samples per spreading code ---------------------------
    _samplesPerCode = static_cast<int32_t>(static_cast<double>(_fs) / static_cast<double>(_codeFreqBasis / _codeLength));

    // --- Find time constants -------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                   // Sampling period in sec
    _tc = 1.0 / static_cast<float>(_codeFreqBasis);        // SPS chip period in sec
    irnss_l5_sps_code_gen_complex(_code, _prn, _chip_shift);  // generate SPS code 1 sample per chip

    for (int32_t i = 0; i < _samplesPerCode; i++)
        {
            // === Digitizing ==================================================

            // --- Make index array to read SPS code values --------------------
            // The length of the index array depends on the sampling frequency -
            // number of samples per millisecond (because one SPS code period is one
            // millisecond).

            aux = (_ts * (i + 1)) / _tc;
            _codeValueIndex = auxCeil(aux) - 1;

            // --- Make the digitized version of the SPS code -------------------
            // The "upsampled" code is made by selecting values form the CA code
            // chip array (caCode) for the time instances of each sample.
            if (i == _samplesPerCode - 1)
                {
                    // --- Correct the last index (due to number rounding issues)
                    _dest[i] = _code[_codeLength - 1];
                }
            else
                {
                    _dest[i] = _code[_codeValueIndex];  // repeat the chip -> upsample
                }
        }
}
