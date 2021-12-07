/*!
 * \file irnss_sps_l5_telemetry_decoder.cc
 * \brief Implementation of an adapter of a IRNSS SPS L5 NAV data decoder block
 * to a TelemetryDecoderInterface
 * \
 *
 * -------------------------------------------------------------------------
 *
 * 
 * 
 * 
 *         
 *
 * T
 *
 * 
 *
 * -------------------------------------------------------------------------
 */


#include "irnss_sps_l5_telemetry_decoder.h"
#include "configuration_interface.h"
#include <glog/logging.h>


Irnssspsl5TelemetryDecoder::Irnssspsl5TelemetryDecoder(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    std::string default_dump_filename = "./navigation.dat";
    DLOG(INFO) << "role " << role;
    dump_.SetFromConfiguration(configuration, role);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);
    // make telemetry decoder object
    telemetry_decoder_ = irnss_make_telemetry_decoder_gs(satellite_, dump_);  // TODO fix me
    LOG(INFO)<<"TELEMETRY DECODER GS CREATED";
    DLOG(INFO) << "telemetry_decoder(" << telemetry_decoder_->unique_id() << ")";
    channel_ = 0;
    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void Irnssspsl5TelemetryDecoder::set_satellite(const Gnss_Satellite& satellite)
{
    satellite_ = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO)<<"gOT THE SATELLITE "<<satellite_;
    telemetry_decoder_->set_satellite(satellite_);
    DLOG(INFO) << "TELEMETRY DECODER: satellite set to " << satellite_;
}


void Irnssspsl5TelemetryDecoder::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}


void Irnssspsl5TelemetryDecoder::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to disconnect
}


gr::basic_block_sptr Irnssspsl5TelemetryDecoder::get_left_block()
{
    return telemetry_decoder_;
}


gr::basic_block_sptr Irnssspsl5TelemetryDecoder::get_right_block()
{
    return telemetry_decoder_;
}
