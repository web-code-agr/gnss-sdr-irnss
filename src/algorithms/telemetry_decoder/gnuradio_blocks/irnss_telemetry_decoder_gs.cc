/*!
 * \file irnss_telemetry_decoder_gs.cc
 * \brief Implementation of a Galileo unified INAV and FNAV message demodulator
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


#include "irnss_telemetry_decoder_gs.h"
#include "IRNSS_at_1.h"   // for GALILEO_E1_CODE_PERIOD_MS // for GALILEO_E6_CODE_PERIOD_MS
#include "convolutional.h"
#include "display.h"
// #include "galileo_almanac_helper.h"  // for Galileo_Almanac_Helper
#include "irnss_ephemeris.h"       // for irnss_Ephemeris
#include "irnss_iono.h"            // for irnss_Iono
#include "irnss_utc_model.h"       // for Galileo_Utc_Model
#include "gnss_synchro.h"
#include "tlm_utils.h"
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>        // for make_any
#include <pmt/pmt_sugar.h>  // for mp
#include <array>
#include <cmath>      // for fmod
#include <cstddef>    // for size_t
#include <cstdlib>    // for abs
#include <exception>  // for exception
#include <iostream>   // for cout
#include <memory>     // for make_shared

#define CRC_ERROR_LIMIT 6


irnss_telemetry_decoder_gs_sptr
irnss_make_telemetry_decoder_gs(const Gnss_Satellite &satellite, const Tlm_Conf &conf)
{
    return irnss_telemetry_decoder_gs_sptr(new irnss_telemetry_decoder_gs(satellite, conf));
}


irnss_telemetry_decoder_gs::irnss_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    const Tlm_Conf &conf) : gr::block("irnss_telemetry_decoder_gs", gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // prevent telemetry symbols accumulation in output buffers
    this->set_max_noutput_items(1);
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // Control messages to tracking block
    this->message_port_register_out(pmt::mp("telemetry_to_trk"));
    d_last_valid_preamble = 0;
    d_sent_tlm_failed_msg = false;
    // d_band = '1';

    // initialize internal vars
    d_dump_filename = conf.dump_filename;
    d_dump = conf.dump;
    d_dump_mat = conf.dump_mat;
    d_remove_dat = conf.remove_dat;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    // d_frame_type = frame_type;
    DLOG(INFO) << "Initializing IRNSS TELEMETRY DECODER";

    
    // d_PRN_code_period_ms = GALILEO_E1_CODE_PERIOD_MS;  // for Galileo E5b is also 4 ms
    d_bits_per_preamble = IRNSS_L5_PREAMBLE_LENGTH_BITS;
    // set the preamble
    d_samples_per_preamble = d_bits_per_preamble;
    d_preamble_period_symbols = IRNSS_L5_SUBFRAME_BITS;
    d_required_symbols = IRNSS_L5_SUBFRAME_BITS;
    // preamble bits to sampled symbols
    d_preamble_samples.reserve(d_samples_per_preamble);
    d_frame_length_symbols = IRNSS_L5_SUBFRAME_BITS - IRNSS_L5_PREAMBLE_LENGTH_BITS;
    d_codelength = static_cast<int32_t>(d_frame_length_symbols);
    d_datalength = (d_codelength / d_nn) - d_mm;
    d_max_symbols_without_valid_frame = IRNSS_L5_SUBFRAME_BITS * 20;  // rise alarm 60 seconds without valid tlm

               

    d_page_part_symbols.reserve(d_frame_length_symbols);
    for (int32_t i = 0; i < d_bits_per_preamble; i++)
        {
            
                        if (IRNSS_L5_PREAMBLE[i] == '1')
                            {
                                d_preamble_samples[i] = 1;
                            }
                        else
                            {
                                d_preamble_samples[i] = -1;
                            }
                    
               
        }
    d_sample_counter = 0ULL;
    d_stat = 0;
    d_preamble_index = 0ULL;

    d_flag_frame_sync = false;

    d_flag_parity = false;
    d_TOW_at_current_symbol_ms = 0;
    d_TOW_at_Preamble_ms = 0;
    d_delta_t = 0;
    d_CRC_error_counter = 0;
    d_flag_TOW_set = false;

    // flag_even_word_arrived = 0;
    d_flag_preamble = false;
    d_channel = 0;
    d_flag_PLL_180_deg_phase_locked = false;
    d_symbol_history.set_capacity(d_required_symbols + 1);

    // vars for Viterbi decoder
    const int32_t max_states = 1U << static_cast<uint32_t>(d_mm);  // 2^d_mm
    std::array<int32_t, 2> g_encoder{{121, 91}};                   // Polynomial G1 and G2
    d_out0.reserve(max_states);
    d_out1.reserve(max_states);
    d_state0.reserve(max_states);
    d_state1.reserve(max_states);

    // create appropriate transition matrices
    nsc_transit(d_out0.data(), d_state0.data(), 0, g_encoder.data(), d_KK, d_nn);
    nsc_transit(d_out1.data(), d_state1.data(), 1, g_encoder.data(), d_KK, d_nn);
}


irnss_telemetry_decoder_gs::~irnss_telemetry_decoder_gs()
{
    DLOG(INFO) << "IRNSS Telemetry decoder block (channel " << d_channel << ") destructor called.";
    size_t pos = 0;
    if (d_dump_file.is_open() == true)
        {
            pos = d_dump_file.tellp();
            try
                {
                    d_dump_file.close();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception in destructor closing the dump file " << ex.what();
                }
            if (pos == 0)
                {
                    if (!tlm_remove_file(d_dump_filename))
                        {
                            LOG(WARNING) << "Error deleting temporary file";
                        }
                }
        }
    if (d_dump && (pos != 0) && d_dump_mat)
        {
            save_tlm_matfile(d_dump_filename);
            if (d_remove_dat)
                {
                    if (!tlm_remove_file(d_dump_filename))
                        {
                            LOG(WARNING) << "Error deleting temporary file";
                        }
                }
        }
}


void irnss_telemetry_decoder_gs::viterbi_decoder(float *page_part_symbols, int32_t *page_part_bits)
{
    Viterbi(page_part_bits, d_out0.data(), d_state0.data(), d_out1.data(), d_state1.data(),
        page_part_symbols, d_KK, d_nn, d_datalength);
}


void irnss_telemetry_decoder_gs::deinterleaver(int32_t rows, int32_t cols, const float *in, float *out)
{
    for (int32_t r = 0; r < rows; r++)
        {
            for (int32_t c = 0; c < cols; c++)
                {
                    out[c * rows + r] = in[r * cols + c];
                }
        }
}


bool irnss_telemetry_decoder_gs::decode_subframe(float *page_symbols, int32_t frame_length)
{
    // 1. De-interleave
    std::vector<float> page_symbols_deint(frame_length);
    deinterleaver(IRNSS_NAV_INTERLEAVER_ROWS, IRNSS_NAV_INTERLEAVER_COLS, page_symbols, page_symbols_deint.data());

    // 2. Viterbi decoder
    // 2.1 Take into account the NOT gate in G2 polynomial (Galileo ICD Figure 13, FEC encoder)
    // 2.2 Take into account the possible inversion of the polarity due to PLL lock at 180 degrees
    for (int32_t i = 0; i < frame_length; i++)
        {
            if ((i + 1) % 2 == 0)
                {
                    page_symbols_deint[i] = -page_symbols_deint[i];
                }
        }

    const int32_t decoded_length = frame_length / 2;
    std::vector<int32_t> page_bits(decoded_length);
    viterbi_decoder(page_symbols_deint.data(), page_bits.data());

    // 3. Call the Galileo page decoder
    std::string page_String;
    page_String.reserve(decoded_length);
    for (int32_t i = 0; i < decoded_length; i++)
        {
            if (page_bits[i] > 0)
                {
                    page_String.push_back('1');
                }
            else
                {
                    page_String.push_back('0');
                }
        }

    // DECODE COMPLETE WORD (even + odd) and TEST CRC
    const int32_t subframe_ID = d_nav.subframe_decoder(page_String);
    

    // 4. Push the new navigation data to the queues
    // if (subframe_synchro_confirmation)
    //     {
            // const int32_t subframe_ID = d_nav.subframe_decoder(subframe.data());  // decode the subframe
            if (subframe_ID > 0 and subframe_ID < 5)
                {
                    std::cout << "New IRNSS NAV message received in channel " << this->d_channel << ": "
                              << "subframe "
                              << subframe_ID << " from satellite "
                              << Gnss_Satellite(std::string("IRNSS"), d_nav.get_satellite_PRN()) << '\n';

                    switch (subframe_ID)
                        {
                        case 2:  // we have a new set of ephemeris data for the current SV
                            if (d_nav.satellite_validation() == true)
                                {
                                    // get ephemeris object for this SV (mandatory)
                                    const std::shared_ptr<Irnss_Ephemeris> tmp_obj = std::make_shared<Irnss_Ephemeris>(d_nav.get_ephemeris());
                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                }
                            break;
                        case 3 or 4:  // Possible IONOSPHERE and UTC model update (page 18)
                            if (d_nav.get_flag_iono_valid() == true)
                                {
                                    const std::shared_ptr<Irnss_Iono> tmp_obj = std::make_shared<Irnss_Iono>(d_nav.get_iono());
                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                }
                            if (d_nav.get_flag_utc_model_valid() == true)
                                {
                                    const std::shared_ptr<Irnss_Utc_Model> tmp_obj = std::make_shared<Irnss_Utc_Model>(d_nav.get_utc_model());
                                    this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
                                }
                            break;
                        case 5:
                        // get almanac (if available)
                        // TODO: implement almanac reader in navigation_message
                        default:
                            break;
                        }
                    return true;
                }
        // }
    return false;
}





void irnss_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
    // gr::thread::scoped_lock lock(d_setlock);
    LOG(INFO)<<"Entering telemetry decoder";
    LOG(INFO)<<"Entering telemetry decoder";
    LOG(INFO)<<"Entering telemetry decoder";
    LOG(INFO)<<"Entering telemetry decoder";
    LOG(INFO)<<"Entering telemetry decoder";
    LOG(INFO)<<"Entering telemetry decoder";
    LOG(INFO)<<"Entering telemetry decoder";


    d_nav = Irnss_Navigation_Message();
    LOG(INFO)<<"D NAV MESSAGE CREATED";
    LOG(INFO)<<"D NAV MESSAGE CREATED";
    LOG(INFO)<<"D NAV MESSAGE CREATED";
    LOG(INFO)<<"D NAV MESSAGE CREATED";
    LOG(INFO)<<"D NAV MESSAGE CREATED";


    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO)<<"GOT D SATTELITE : "<<d_satellite;
    LOG(INFO)<<"GOT D SATTELITE : "<<d_satellite;
    LOG(INFO)<<"GOT D SATTELITE : "<<d_satellite;
    LOG(INFO)<<"GOT D SATTELITE : "<<d_satellite;
    LOG(INFO)<<"GOT D SATTELITE : "<<d_satellite;
    LOG(INFO)<<"GOT D SATTELITE : "<<d_satellite;


    // d_last_valid_preamble = d_sample_counter;
    // d_sent_tlm_failed_msg = false;
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;

    d_nav.set_satellite_PRN(d_satellite.get_PRN());
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
    std::cout<<"navigation satellite set to "<<d_satellite<< '\n';

}


void irnss_telemetry_decoder_gs::reset()
{
    gr::thread::scoped_lock lock(d_setlock);
    d_last_valid_preamble = d_sample_counter;
    d_sent_tlm_failed_msg = false;
    d_stat = 0;
    DLOG(INFO) << "Telemetry decoder reset for satellite " << d_satellite;
}


void irnss_telemetry_decoder_gs::set_channel(int32_t channel)
{
    d_channel = channel;
    d_nav.set_channel(channel);
    DLOG(INFO) << "Navigation channel set to " << channel;
    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                        {
                            d_dump_filename.append(std::to_string(d_channel));
                            d_dump_filename.append(".dat");
                            d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Telemetry decoder dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str();
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what();
                        }
                }
        }
}


int irnss_telemetry_decoder_gs::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    auto **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);            // Get the output buffer pointer
    const auto **in = reinterpret_cast<const Gnss_Synchro **>(&input_items[0]);  // Get the input buffer pointer

    Gnss_Synchro current_symbol{};  // structure to save the synchronization information and send the output object to the next block
    // 1. Copy the current tracking output
    current_symbol = in[0][0];
    d_symbol_history.push_back(current_symbol.Prompt_I);
    d_sample_counter++;  // count for the processed symbols
    consume_each(1);
    d_flag_preamble = false;

    // check if there is a problem with the telemetry of the current satellite
    if (d_sent_tlm_failed_msg == false && d_stat < 2)
        {
            if ((d_sample_counter - d_last_valid_preamble) > d_max_symbols_without_valid_frame)
                {
                    const int message = 1;  // bad telemetry
                    DLOG(INFO) << "sent msg sat " << this->d_satellite;
                    this->message_port_pub(pmt::mp("telemetry_to_trk"), pmt::make_any(message));
                    d_sent_tlm_failed_msg = true;
                }
        }

    // ******* frame sync ******************
    switch (d_stat)
        {
        case 0:  // no preamble information
            {
                // correlate with preamble
                int32_t corr_value = 0;
                if (d_symbol_history.size() >= IRNSS_L5_PREAMBLE_LENGTH_BITS)
                    {
                        // ******* preamble correlation ********
                        for (int32_t i = 0; i < IRNSS_L5_PREAMBLE_LENGTH_BITS; i++)
                            {
                                if (d_symbol_history[i] < 0.0)  // symbols clipping
                                    {
                                        corr_value -= d_preamble_samples[i];
                                    }
                                else
                                    {
                                        corr_value += d_preamble_samples[i];
                                    }
                            }
                    }
                        if (abs(corr_value) >= d_samples_per_preamble)
                            {
                                d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                                DLOG(INFO) << "Preamble detection for IRNSS satellite " << this->d_satellite;
                                decode_subframe(d_page_part_symbols.data(), d_frame_length_symbols);
                                d_stat = 1;  // enter into frame pre-detection status
                            }
                    
                d_flag_TOW_set = false;
                break;
            }
        case 1:  // possible preamble lock
            {
                // correlate with preamble
                int32_t corr_value = 0;
                if (d_symbol_history.size() > d_required_symbols)
                    {
                        // ******* preamble correlation ********
                        for (int32_t i = 0; i < d_samples_per_preamble; i++)
                            {
                                if (d_symbol_history[i] < 0.0)  // symbols clipping
                                    {
                                        corr_value -= d_preamble_samples[i];
                                    }
                                else
                                    {
                                        corr_value += d_preamble_samples[i];
                                    }
                            }
                        if (abs(corr_value) >= d_samples_per_preamble)
                            {
                                // check preamble separation
                                const auto preamble_diff = static_cast<int32_t>(d_sample_counter - d_preamble_index);
                                if (abs(preamble_diff - d_preamble_period_symbols) == 0)
                                    {
                                        // try to decode frame
                                        DLOG(INFO) << "Starting page decoder for irnss satellite " << this->d_satellite;
                                        d_preamble_index = d_sample_counter;  // record the preamble sample stamp
                                        d_CRC_error_counter = 0;
                                        if (corr_value < 0)
                                            {
                                                d_flag_PLL_180_deg_phase_locked = true;
                                            }
                                        else
                                            {
                                                d_flag_PLL_180_deg_phase_locked = false;
                                            }
                                        decode_subframe(d_page_part_symbols.data(), d_frame_length_symbols);
                                        
                                        d_stat = 2;
                                    }
                                else
                                    {
                                        if (preamble_diff > d_preamble_period_symbols)
                                            {
                                                d_stat = 0;  
                                                d_flag_TOW_set = false;// start again
                                            }
                                    }
                            }
                    }
                break;
            }
        case 2:  // preamble acquired
            {
                if (d_sample_counter == d_preamble_index + static_cast<uint64_t>(d_preamble_period_symbols))
                    {
                        DLOG(INFO) << "Preamble received for SAT " << this->d_satellite << "d_sample_counter=" << d_sample_counter << "\n";
                        // call the decoder
                        // 0. fetch the symbols into an array
                        d_preamble_index = d_sample_counter;  // record the preamble sample stamp (t_P)

                        if (decode_subframe(d_page_part_symbols.data(), d_frame_length_symbols))
                            {
                                d_CRC_error_counter = 0;
                                d_flag_preamble = true;  // valid preamble indicator (initialized to false every work())
                                gr::thread::scoped_lock lock(d_setlock);
                                d_last_valid_preamble = d_sample_counter;
                                if (!d_flag_frame_sync)
                                    {
                                        d_flag_frame_sync = true;
                                        DLOG(INFO) << " Frame sync SAT " << this->d_satellite;
                                    }
                            }
                        else
                            {
                                d_CRC_error_counter++;
                                if (d_CRC_error_counter > 2)
                                    {
                                        DLOG(INFO) << "Lost of frame sync SAT " << this->d_satellite;
                                        d_flag_frame_sync = false;
                                        d_stat = 0;
                                        d_TOW_at_current_symbol_ms = 0;
                                        d_TOW_at_Preamble_ms = 0;
                                        d_CRC_error_counter = 0;
                                        d_flag_TOW_set = false;
                                    }
                            }
                    }
                break;
            }
        }

    // UPDATE GNSS SYNCHRO DATA
    // 2. Add the telemetry decoder information
    if (this->d_flag_preamble == true)
        // update TOW at the preamble instant
        {
            if (!(d_nav.get_TOW() == 0))
                {
                    d_TOW_at_current_symbol_ms = static_cast<uint32_t>(d_nav.get_TOW() * 1000.0);
                    d_TOW_at_Preamble_ms = static_cast<uint32_t>(d_nav.get_TOW() * 1000.0);
                    d_flag_TOW_set = true;
                }
            else
                {
                    DLOG(INFO) << "Received IRNSS L1 TOW equal to zero at sat " << d_nav.get_satellite_PRN();
                }
        }
    else  // if there is not a new preamble, we define the TOW of the current symbol
        {
            if (d_flag_TOW_set == true)
                {
                    d_TOW_at_current_symbol_ms += IRNSS_L5_PREAMBLE_DURATION_MS;
                }
        }

    if (d_flag_TOW_set == true)
        {
            current_symbol.TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
            current_symbol.Flag_valid_word = d_flag_TOW_set;

            if (d_flag_PLL_180_deg_phase_locked == true)
                {
                    // correct the accumulated phase for the Costas loop phase shift, if required
                    current_symbol.Carrier_phase_rads += GNSS_PI;
                }

            if (d_dump == true)
                {
                    // MULTIPLEXED FILE RECORDING - Record results to file
                    try
                        {
                            double tmp_double;
                            uint64_t tmp_ulong_int;
                            int32_t tmp_int;
                            tmp_double = static_cast<double>(d_TOW_at_current_symbol_ms) / 1000.0;
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                            tmp_ulong_int = current_symbol.Tracking_sample_counter;
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_ulong_int), sizeof(uint64_t));
                            tmp_double = static_cast<double>(d_TOW_at_Preamble_ms) / 1000.0;
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                            tmp_int = (current_symbol.Prompt_I > 0.0 ? 1 : -1);
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_int), sizeof(int32_t));
                            tmp_int = static_cast<int32_t>(current_symbol.PRN);
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_int), sizeof(int32_t));
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            LOG(WARNING) << "Exception writing observables dump file " << e.what();
                        }
                }

            // 3. Make the output (copy the object contents to the GNU Radio reserved memory)
            *out[0] = current_symbol;

            return 1;
        }

    
    return 0;
}
