#pragma once
#include <boost/signals2/signal.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/system/error_code.hpp>
#include <memory>
#include <array>

namespace cav
{

/**
 * @brief CANFrame representation to be used with CANInterface
 */
struct CANFrameStamped
{
    boost::posix_time::ptime stamp;
    uint32_t id;
    bool is_rtr;
    bool is_extended;
    bool is_error;
    uint8_t dlc;
    std::array<uint8_t,8>data;

    void setData(uint64_t d)
    {
        data[0] = (d >> 56) & 0xFF;
        data[1] = (d >> 48) & 0xFF;
        data[2] = (d >> 40) & 0xFF;
        data[3] = (d >> 32) & 0xFF;
        data[4] = (d >> 24) & 0xFF;
        data[5] = (d >> 16) & 0xFF;
        data[6] = (d >> 8) & 0xFF;
        data[7] = (d >> 0) & 0xFF;
    }
};


/**
 * @brief Represents an interface to a CAN Device.
 */
class CANInterface {
public:

    typedef boost::signals2::signal<void (std::shared_ptr<CANFrameStamped>)> _onFrameReceived_signal_type;
    typedef boost::signals2::signal<void (std::shared_ptr<CANFrameStamped>)> _onErrorFrameReceived_signal_type;
    typedef boost::signals2::signal<void (const boost::system::error_code&)> _onError_signal_type;
    typedef boost::signals2::signal<void (const boost::system::error_code&)> _onClosed_signal_type;
    typedef boost::signals2::signal<void ()> _onOpen_signal_type;

    /**
     * @brief signal called when a frame is received
     * @param std::shared_ptr<CANFrameStamped> frame - the frame received
     */
    _onFrameReceived_signal_type onFrameReceived;

    /**
     * @brief signal called when an error frame is received
     * @param std::shared_ptr<CANFrameStamped> frame - the frame received
     */
    _onErrorFrameReceived_signal_type onErrorFrameReceived;

    /**
     * @brief signal called when device is closed
     * @param boost:;system::error_code& ec - if set was the error that caused the device to close
     */
    _onClosed_signal_type onClosed;

    /**
     * @brief signal called when device is opened
     */
    _onOpen_signal_type onOpen;

    /**
     * @brief signal called when device experiences an error
     * @param boost:;system::error_code& ec - the corresponding error
     */
    _onError_signal_type onError;

    virtual ~CANInterface() = default;

    /**
     * @brief Closes the interface to the device
     */
    virtual void close() = 0;

    /**
     * @brief removes all CAN filters
     */
    virtual void removeFilters() = 0;

    /**
     * @brief Sets CAN filters on passed IDs. This will cause only the IDs specified to signal
     * an onFrameReceived event
     * @param can_ids
     */
    virtual void setFilters(const std::vector<uint16_t> &can_ids) = 0;

    /**
     * @brief writes the frame out to the can bus
     * @param frame
     */
    virtual void write(const CANFrameStamped &frame) = 0;

    /**
     * @brief returns true if device is open
     * @return
     */
    virtual inline bool is_open() = 0;

};

}
