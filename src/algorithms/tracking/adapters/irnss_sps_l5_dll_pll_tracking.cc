/*!
 * \file irnss_sps_l5_dll_pll_tracking.cc
 * \brief Implementation of an adapter of a DLL+PLL tracking loop block
 * for IRNSS SPS L5 to a TrackingInterface
 * 
 *         
 *
 *
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
 * -------------------------------------------------------------------------
 */

#include "irnss_sps_l5_dll_pll_tracking.h"
#include "IRNSS_at_1.h"
#include "configuration_interface.h"
#include "display.h"
#include "dll_pll_conf.h"
#include "gnss_sdr_flags.h"
#include <glog/logging.h>
#include <array>

Irnssspsl5DllPllTracking::Irnssspsl5DllPllTracking(
    const ConfigurationInterface* configuration, const std::string& role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    Dll_Pll_Conf trk_params = Dll_Pll_Conf();
    DLOG(INFO) << "role " << role;
    trk_params.SetFromConfiguration(configuration, role);

    int vector_length = std::round(trk_params.fs_in / (IRNSS_L5I_CODE_RATE_HZ / IRNSS_L5I_CODE_LENGTH_CHIPS));
    trk_params.vector_length = vector_length;
    if (trk_params.extend_correlation_symbols < 1)
        {
            trk_params.extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: IRNSS SPS L5. extend_correlation_symbols must be bigger than 1. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << std::endl;
        }
    else if (trk_params.extend_correlation_symbols > 20)
        {
            trk_params.extend_correlation_symbols = 20;
            std::cout << TEXT_RED << "WARNING: IRNSS SPS L5. extend_correlation_symbols must be lower than 21. Coherent integration has been set to 20 symbols (20 ms)" << TEXT_RESET << std::endl;
        }
    trk_params.track_pilot = configuration->property(role + ".track_pilot", false);
    if (trk_params.track_pilot)
        {
            trk_params.track_pilot = false;
            std::cout << TEXT_RED << "WARNING: IRNSS SPS L5 does not have pilot signal. Data tracking has been enabled" << TEXT_RESET << std::endl;
        }
    if ((trk_params.extend_correlation_symbols > 1) and (trk_params.pll_bw_narrow_hz > trk_params.pll_bw_hz or trk_params.dll_bw_narrow_hz > trk_params.dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: IRNSS SPS L5. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << std::endl;
        }

    trk_params.system = 'I';
    std::array<char, 3> sig_{'1', 'I', '\0'};
    std::memcpy(trk_params.signal, sig_.data(), 3);

    // ################# Make a GNU Radio Tracking block object ################
    if (trk_params.item_type == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
            tracking_ = dll_pll_veml_make_tracking(trk_params);
        }
    else
        {
            item_size_ = sizeof(gr_complex);
            LOG(WARNING) << trk_params.item_type << " unknown tracking item type.";
        }
    channel_ = 0;
    DLOG(INFO) << "tracking(" << tracking_->unique_id() << ")";
    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void Irnssspsl5DllPllTracking::stop_tracking()
{
    tracking_->stop_tracking();
}


void Irnssspsl5DllPllTracking::start_tracking()
{
    tracking_->start_tracking();
}


/*
 * Set tracking channel unique ID
 */
void Irnssspsl5DllPllTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_->set_channel(channel);
}


void Irnssspsl5DllPllTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_->set_gnss_synchro(p_gnss_synchro);
}


void Irnssspsl5DllPllTracking::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to connect, now the tracking uses gr_sync_decimator
}


void Irnssspsl5DllPllTracking::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr Irnssspsl5DllPllTracking::get_left_block()
{
    return tracking_;
}


gr::basic_block_sptr Irnssspsl5DllPllTracking::get_right_block()
{
    return tracking_;
}
