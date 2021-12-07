/*!
 * \file irnss_sdr_l5_telemetry_decoder.h
 * \
 * t
 * \
 *
 * -------------------------------------------------------------------------
 *this uses gps_l1_ca_telemetry_decoder_gs.h (and gps_navigation_message.h) variables and values
 * TO BE REPLACED!!
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


#ifndef GNSS_SDR_IRNSS_SPS_L5_TELEMETRY_DECODER_H
#define GNSS_SDR_IRNSS_SPS_L5_TELEMETRY_DECODER_H

#include "gnss_satellite.h"  // for Gnss_Satellite
#include "gnss_synchro.h"
#include "irnss_telemetry_decoder_gs.h"
#include "telemetry_decoder_interface.h"
#include "tlm_conf.h"

#include <gnuradio/runtime_types.h>  // for basic_block_sptr, top_block_sptr
#include <cstddef>                   // for size_t
#include <string>

class ConfigurationInterface;

/*!
 * \brief This class implements a NAV data decoder for IRNSS SPS L5
 */
class Irnssspsl5TelemetryDecoder : public TelemetryDecoderInterface
{
public:
    Irnssspsl5TelemetryDecoder(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~Irnssspsl5TelemetryDecoder() = default;

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "IRNSS_SPS_L5_Telemetry_Decoder"
    inline std::string implementation() override
    {
        return "IRNSS_SPS_L5_Telemetry_Decoder";
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    void set_satellite(const Gnss_Satellite& satellite) override;
    inline void set_channel(int channel) override { telemetry_decoder_->set_channel(channel); }
    inline void reset() override
    {
        telemetry_decoder_->reset();
        return;
    }

    inline size_t item_size() override
    {
        return 0;
    }

private:
    irnss_telemetry_decoder_gs_sptr telemetry_decoder_;
    Gnss_Satellite satellite_;
    int channel_;
    Tlm_Conf dump_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif
