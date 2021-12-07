/*!
 * \file irnss_sps_l5_dll_pll_tracking.h
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for IRNSS SPS L5 to a TrackingInterface
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

#ifndef IRNSS_SPS_L5_DLL_PLL_TRACKING_H
#define IRNSS_SPS_L5_DLL_PLL_TRACKING_H

#include "dll_pll_veml_tracking.h"
#include "tracking_interface.h"
#include <string>

class ConfigurationInterface;

/*!
 * \brief This class implements a code DLL + carrier PLL tracking loop
 */
class Irnssspsl5DllPllTracking : public TrackingInterface
{
public:
    Irnssspsl5DllPllTracking(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~Irnssspsl5DllPllTracking() = default;

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "IRNSS_SPS_L5_DLL_PLL_Tracking"
    inline std::string implementation() override
    {
        return "IRNSS_L5_CA_DLL_PLL_Tracking";
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
     * \brief Set tracking channel unique ID
     */
    void set_channel(unsigned int channel) override;

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to efficiently exchange synchronization data between acquisition and tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;

    void start_tracking() override;

    /*!
     * \brief Stop running tracking
     */
    void stop_tracking() override;

private:
    dll_pll_veml_tracking_sptr tracking_;
    size_t item_size_;
    unsigned int channel_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif  // IRNSS_SPS_L5_DLL_PLL_TRACKING_H
