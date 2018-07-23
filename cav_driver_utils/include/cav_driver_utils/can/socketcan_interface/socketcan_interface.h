#pragma once
#include <cav_driver_utils/can/can_interface.h>

#include <memory>

namespace cav
{

class SocketCANInterface : public CANInterface {
    class SocketCANInterfacePimpl;
    std::unique_ptr<SocketCANInterfacePimpl> pimpl_;

public:

    SocketCANInterface();
    explicit SocketCANInterface(const std::string &device_name);

    SocketCANInterface(const SocketCANInterface &) = delete;
    SocketCANInterface(SocketCANInterface&& other) = delete;

    virtual ~SocketCANInterface();

    /**
     * @brief Closes the interface to the device
     */
    virtual void close();

    /**
     * @brief removes all CAN filters
     */
    virtual void removeFilters();

    /**
     * @brief Sets CAN filters on passed IDs. This will cause only the IDs specified to signal
     * an onFrameReceived event. Sets the can_filter on the socketcan driver
     * @param can_ids
     */
    virtual void setFilters(const std::vector<uint32_t> &can_ids);

    /**
     * @brief writes the frame out to the can bus
     * @param frame
     */
    virtual void write(const CANFrameStamped& frame);

    /**
     * @brief returns true if device is open
     * @return
     */
    virtual inline bool is_open();

};

}
