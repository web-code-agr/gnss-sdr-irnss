/*!
 * \file irnss_sps_l5_pcps_acquisition.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  IRNSS SPS L5 signals
 * \
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

#ifndef IRNSS_SPS_L1_ACQUISITION_H
#define IRNSS_SPS_L1_ACQUISITION_H

#include "acq_conf.h"
#include "channel_fsm.h"
#include "complex_byte_to_float_x2.h"
#include "gnss_synchro.h"
#include "pcps_acquisition.h"
#include <gnuradio/blocks/float_to_complex.h>
#include <memory>
#include <string>
#include <vector>


class ConfigurationInterface;

/*!
 * \brief This class adapts a PCPS acquisition block to an AcquisitionInterface
 *  for IRNSS L1 C/A signals
 */
class IrnssSpsL5PcpsAcquisition : public AcquisitionInterface
{
public:
    IrnssSpsL5PcpsAcquisition(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~IrnssSpsL5PcpsAcquisition() = default;

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "IRNSS_L1_CA_PCPS_Acquisition"
     */
    inline std::string implementation() override
    {
        return "IRNSS_L1_CA_PCPS_Acquisition";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to efficiently exchange synchronization data between acquisition and
     *  tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;

    /*!
     * \brief Set acquisition channel unique ID
     */
    inline void set_channel(unsigned int channel) override
    {
        channel_ = channel;
        acquisition_->set_channel(channel_);
    }

    /*!
     * \brief Set channel fsm associated to this acquisition instance
     */
    inline void set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm) override
    {
        channel_fsm_ = channel_fsm;
        acquisition_->set_channel_fsm(channel_fsm);
    }

    /*!
     * \brief Set statistics threshold of PCPS algorithm
     */
    void set_threshold(float threshold) override;

    /*!
     * \brief Set maximum Doppler off grid search
     */
    void set_doppler_max(unsigned int doppler_max) override;

    /*!
     * \brief Set Doppler steps for the grid search
     */
    void set_doppler_step(unsigned int doppler_step) override;

    /*!
     * \brief Set Doppler center for the grid search
     */
    void set_doppler_center(int doppler_center) override;

    /*!
     * \brief Initializes acquisition algorithm.
     */
    void init() override;

    /*!
     * \brief Sets local code for IRNSS SPS-L5 PCPS acquisition algorithm.
     */
    void set_local_code() override;

    /*!
     * \brief Returns the maximum peak of grid search
     */
    signed int mag() override;

    /*!
     * \brief Restart acquisition algorithm
     */
    void reset() override;

    /*!
     * \brief If state = 1, it forces the block to start acquiring from the first sample
     */
    void set_state(int state) override;

    /*!
     * \brief Stop running acquisition
     */
    void stop_acquisition() override;

    /*!
     * \brief Sets the resampler latency to account it in the acquisition code delay estimation
     */
    void set_resampler_latency(uint32_t latency_samples) override;

private:
    const ConfigurationInterface* configuration_;
    pcps_acquisition_sptr acquisition_;
    Acq_Conf acq_parameters_;
    gr::blocks::float_to_complex::sptr float_to_complex_;
    complex_byte_to_float_x2_sptr cbyte_to_float_x2_;
    size_t item_size_;
    std::string item_type_;
    unsigned int vector_length_;
    unsigned int code_length_;
    unsigned int channel_;
    std::weak_ptr<ChannelFsm> channel_fsm_;
    float threshold_;
    unsigned int doppler_max_;
    unsigned int doppler_step_;
    int doppler_center_;
    unsigned int sampled_ms_;
    std::string dump_filename_;
    std::vector<std::complex<float>> code_;
    Gnss_Synchro* gnss_synchro_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif  // IRNSS_SPS_L1_ACQUISITION_H
