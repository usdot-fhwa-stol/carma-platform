#include <cav_driver_utils/can/socketcan_interface/socketcan_interface.h>

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>

#include <boost/asio.hpp>
#include <boost/date_time/posix_time/conversion.hpp>

#include <thread>

namespace cav
{

/**
 * @brief Private implementation of the SocketCANInterface
 */
class SocketCANInterface::SocketCANInterfacePimpl
{
    /**
     * @brief Uses the linux can representation
     */
    struct CANFrameStamped
    {
        boost::posix_time::ptime stamp;
        ::can_frame frame;
    };

    /**
     * @brief Converts internal stamped can message to the CANInterface type
     * @param msg
     * @return
     */
    cav::CANFrameStamped toCANFrameStamped(const SocketCANInterfacePimpl::CANFrameStamped& msg)
    {
        cav::CANFrameStamped out;
        out.stamp = msg.stamp;
        out.id = msg.frame.can_id;
        out.dlc = msg.frame.can_dlc;
        out.is_error = (msg.frame.can_id & CAN_ERR_FLAG) != 0;
        out.is_extended = (msg.frame.can_id & CAN_EFF_FLAG) != 0;
        out.is_rtr = (msg.frame.can_id & CAN_RTR_FLAG) != 0;

        return out;
    }

    std::string device_name_;
    std::shared_ptr<boost::asio::io_service> ios_;
    std::unique_ptr<std::thread> ios_thread_;
    std::unique_ptr<boost::asio::posix::stream_descriptor> can_stream_descriptor_;
    ::can_frame rx_;
    int socket_;
    bool is_open_;

    /**
     * @brief Async reads socketcan descriptor
     */
    void start_async_reading() {
        if (is_open_)
            can_stream_descriptor_->async_read_some(boost::asio::buffer(&rx_, sizeof(rx_)),
                                                    [this](const boost::system::error_code &error,
                                                           size_t bytes_transferred) {
                                                        handle_async_read(error, bytes_transferred);
                                                    });
    }


    /**
     * @brief On frame received, extracts the frame and fires the appropriate frame signal
     * @param error
     * @param bytes_transferred
     */
    void handle_async_read(const boost::system::error_code &error, size_t bytes_transferred) {
        if (!error || bytes_transferred < sizeof(rx_)) {
            std::shared_ptr<cav::CANFrameStamped> frameStamped = std::make_shared<cav::CANFrameStamped>();
            frameStamped->id = rx_.can_id;
            frameStamped->dlc = rx_.can_dlc;
            frameStamped->is_error = (rx_.can_id & CAN_ERR_FLAG) != 0;
            frameStamped->is_rtr = (rx_.can_id & CAN_RTR_FLAG) != 0;
            frameStamped->is_extended = (rx_.can_id & CAN_EFF_FLAG) != 0;
            std::copy(std::begin(rx_.data),std::begin(rx_.data)+rx_.can_dlc,frameStamped->data.begin());
            ::timeval tv;
            ::ioctl(socket_, SIOCGSTAMP, &tv);
            start_async_reading();

            frameStamped->stamp = boost::posix_time::from_time_t(tv.tv_sec);
            frameStamped->stamp += boost::posix_time::microseconds(tv.tv_usec);
            if (frameStamped->is_error & CAN_ERR_FLAG) {
                onErrorFrameReceived(frameStamped);
            } else {
                onFrameReceived(frameStamped);
            }
        } else {
            close(error);
        }
    }

public:

    CANInterface::_onFrameReceived_signal_type& onFrameReceived;
    CANInterface::_onErrorFrameReceived_signal_type& onErrorFrameReceived;
    CANInterface::_onClosed_signal_type& onClosed;
    CANInterface::_onOpen_signal_type& onOpen;
    CANInterface::_onError_signal_type& onError;

    SocketCANInterfacePimpl(CANInterface::_onFrameReceived_signal_type& fr,
                            CANInterface::_onErrorFrameReceived_signal_type&efr,
                            CANInterface::_onClosed_signal_type&cl,
                            CANInterface::_onOpen_signal_type& op,
                            CANInterface::_onError_signal_type& oe) : onFrameReceived(fr),onErrorFrameReceived(efr),onClosed(cl),onOpen(op), onError(oe), is_open_(false)
    {

    }

    explicit SocketCANInterfacePimpl(const std::string &device_name,
                                     CANInterface::_onFrameReceived_signal_type& fr,
                                     CANInterface::_onErrorFrameReceived_signal_type&efr,
                                     CANInterface::_onClosed_signal_type&cl,
                                     CANInterface::_onOpen_signal_type& op,
                                     CANInterface::_onError_signal_type& oe) : SocketCANInterfacePimpl(fr,efr,cl,op,oe)
    {
        open(device_name);
    }

    SocketCANInterfacePimpl(const SocketCANInterface &) = delete;

    virtual ~SocketCANInterfacePimpl() {
        close();
    }


    /**
     * @brief Opens a SocketCAN device with device_name
     * @param device_name
     */
    void open(const std::string &device_name) {
        if (is_open_) {
            throw std::runtime_error("SocketCAN Interface is already open");
        }

        device_name_ = device_name;
        sockaddr_can addr;
        ifreq ifr;

        if ((socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            throw std::system_error(errno, std::generic_category(), "Unabled to open socket");
        }

        std::strcpy(ifr.ifr_name, device_name_.c_str());
        ::ioctl(socket_, SIOCGIFINDEX, &ifr);

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        int err;
        if ((err = ::bind(socket_, (struct sockaddr *) &addr, sizeof(addr))) < 0) {
            ::close(socket_);
            throw std::system_error(errno, std::generic_category(), "Failed to bind socket");
        }

        //setup error frames
        ::can_err_mask_t err_mask = (CAN_ERR_TX_TIMEOUT | CAN_ERR_CRTL | CAN_ERR_BUSOFF | CAN_ERR_RESTARTED);
        ::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

        ios_.reset(new boost::asio::io_service());

        can_stream_descriptor_.reset(new boost::asio::posix::stream_descriptor(*ios_));

        can_stream_descriptor_->assign(socket_);

        is_open_ = true;
        onOpen();
        start_async_reading();

        ios_thread_.reset(new std::thread([this]() { ios_->run(); }));
    }

    /**
     * @brief closes the internal socket and the ios_thread
     */
    void close() {
        close(boost::system::error_code());
    }

    /**
     * @brief closes the internal socket and the ios_thread
     * @param ec - the error to pass to the onClosed signal
     */
    void close(const boost::system::error_code &ec) {
        if(!is_open_) return;
        is_open_ = false;
        can_stream_descriptor_->close();
        ios_->stop();
        if (std::this_thread::get_id() != ios_thread_->get_id()) {
            ios_thread_->join();
        }
        onClosed(ec);
    }


    /**
     * @brief resets filters to accept all can frames
     */
    void removeFilters() {
        if (!is_open_) return;

        ::can_filter rfilter[1];
        rfilter[0].can_id = 0x7FF;
        rfilter[0].can_mask = 0x0;
        ::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    }

    /**
     * @brief Sets the appropriate can filter
     * @param can_ids
     */
    void setFilters(const std::vector<uint16_t> &can_ids) {
        if (!is_open_) return;

        std::vector<::can_filter> rfilter(can_ids.size());
        for (size_t i = 0; i < can_ids.size(); i++) {
            rfilter[i].can_id = can_ids[i] & 0x7FF;
            rfilter[i].can_mask = CAN_SFF_MASK;
        }

        ::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter[0], sizeof(can_filter) * rfilter.size());
    }

    /**
     * @brief Writes the frame to the can device
     * @param frame
     */
    void write(can_frame &frame) {
        if (!is_open_) return;
        boost::system::error_code ec;
        boost::asio::write(*can_stream_descriptor_, boost::asio::buffer(&frame, sizeof(frame)), boost::asio::transfer_all(), ec);
        if(ec)
        {
            onError(ec);
        }
    }


    /**
     * @brief Returns whether the device is open or not
     * @return
     */
    inline bool is_open() { return is_open_; }

};



SocketCANInterface::SocketCANInterface() :
        pimpl_(new SocketCANInterface::SocketCANInterfacePimpl(onFrameReceived,onErrorFrameReceived,onClosed,onOpen,onError))
{
}


SocketCANInterface::SocketCANInterface(const std::string &device_name) :
        pimpl_(new SocketCANInterface::SocketCANInterfacePimpl(device_name,onFrameReceived,onErrorFrameReceived,onClosed,onOpen,onError))
{
}

SocketCANInterface::~SocketCANInterface() {
}

void SocketCANInterface::close() {
    pimpl_->close();
}

void SocketCANInterface::removeFilters() {
    pimpl_->removeFilters();

}

void SocketCANInterface::setFilters(const std::vector<uint16_t> &can_ids) {
    pimpl_->setFilters(can_ids);
}

void SocketCANInterface::write(const CANFrameStamped& frame) {
    can_frame out;
    memcpy(&out.data[0],&frame.data[0],frame.dlc);
    out.can_dlc = frame.dlc;
    out.can_id = frame.id;

    pimpl_->write(out);
}

bool SocketCANInterface::is_open() { return pimpl_->is_open(); }


}//namespace cav
