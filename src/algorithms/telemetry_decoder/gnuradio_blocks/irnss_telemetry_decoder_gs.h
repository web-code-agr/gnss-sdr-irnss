/*!
 * \file irnss_telemetry_decoder_gs.h
 * \brief Implementation of a IRNSS unified INAV and FNAV message demodulator
 * block
 * \author Javier Arribas 2018. jarribas(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_IRNSS_TELEMETRY_DECODER_GS_H
#define GNSS_SDR_IRNSS_TELEMETRY_DECODER_GS_H

#include "IRNSS_at_1.h"
#include "irnss_navigation_message.h"
#include "gnss_synchro.h"
#include "gnss_block_interface.h"
#include "gnss_satellite.h"
#include "tlm_conf.h"
#include <boost/circular_buffer.hpp>
#include <gnuradio/block.h>  // for block
#include <gnuradio/types.h>  // for gr_vector_const_void_star
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_gnuradio_blocks
 * \{ */


class irnss_telemetry_decoder_gs;

using irnss_telemetry_decoder_gs_sptr = gnss_shared_ptr<irnss_telemetry_decoder_gs>;

irnss_telemetry_decoder_gs_sptr irnss_make_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    const Tlm_Conf &conf);

/*!
 * \brief This class implements a block that decodes the INAV and FNAV data defined in Galileo ICD
 */
class irnss_telemetry_decoder_gs : public gr::block
{
public:
    ~irnss_telemetry_decoder_gs();
    void set_satellite(const Gnss_Satellite &satellite);  //!< Set satellite PRN
    void set_channel(int32_t channel);                    //!< Set receiver's channel
    void reset();

    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);


private:
    friend irnss_telemetry_decoder_gs_sptr irnss_make_telemetry_decoder_gs(
        const Gnss_Satellite &satellite,
        const Tlm_Conf &conf);

    irnss_telemetry_decoder_gs(const Gnss_Satellite &satellite, const Tlm_Conf &conf);

    const int32_t d_nn = 2;  // Coding rate 1/n
    const int32_t d_KK = 7;  // Constraint Length

    void viterbi_decoder(float *page_part_symbols, int32_t *page_part_bits);
    void deinterleaver(int32_t rows, int32_t cols, const float *in, float *out);
    bool decode_subframe(float *page_symbols, int32_t frame_length);

    Irnss_Navigation_Message d_nav;
    Gnss_Satellite d_satellite;

    // vars for Viterbi decoder
    std::vector<int32_t> d_preamble_samples;
    std::vector<float> d_page_part_symbols;
    std::vector<int32_t> d_out0;
    std::vector<int32_t> d_out1;
    std::vector<int32_t> d_state0;
    std::vector<int32_t> d_state1;

    std::string d_dump_filename;
    std::ofstream d_dump_file;

    boost::circular_buffer<float> d_symbol_history;


    double d_delta_t;  // GPS-GALILEO time offset

    uint64_t d_sample_counter;
    uint64_t d_preamble_index;
    uint64_t d_last_valid_preamble;

    int32_t d_mm = d_KK - 1;
    int32_t d_codelength;
    int32_t d_datalength;
    int32_t d_bits_per_preamble;
    int32_t d_samples_per_preamble;
    int32_t d_preamble_period_symbols;
    int32_t d_CRC_error_counter;
    int32_t d_channel;

    uint32_t d_PRN_code_period_ms;
    uint32_t d_required_symbols;
    uint32_t d_frame_length_symbols;
    uint32_t d_stat;
    uint32_t d_TOW_at_Preamble_ms;
    uint32_t d_TOW_at_current_symbol_ms;
    uint32_t d_max_symbols_without_valid_frame;
    
// This variable will store which band we are dealing with (Galileo E1 or E5b)

    bool d_sent_tlm_failed_msg;
    bool d_flag_frame_sync;
    bool d_flag_PLL_180_deg_phase_locked;
    bool d_flag_parity;
    bool d_flag_TOW_set;
    bool d_flag_preamble;
    bool d_dump;
    bool d_dump_mat;
    bool d_remove_dat;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_irnss_TELEMETRY_DECODER_GS_H
