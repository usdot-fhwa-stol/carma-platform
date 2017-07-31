#pragma once
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Torc Robotics, LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Torc Robotics, LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/asio.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/lambda/lambda.hpp>

#include <memory>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <vector>

using boost::asio::ip::tcp;
using boost::asio::ip::udp;


namespace torc
{

//TODO: Add doxygen
enum class ProtocolFlag : uint8_t
{
    ServiceMessage = 0,
    ProtocolMessage = 1
};

/**
 * @brief The PinPoint message types
 */
enum class MessageType : uint8_t
{
    ReturnValue = 0,
    Invoke = 1,
    InvokeWithReturn = 2,
    SignalEmitted = 3,
    ScheduleReturn = 4,
    Error = 5,
    InvokeWithReturnCode = 6,
    ReturnValueWithCode = 7,
    Exception = 8,
    ScheduleException = 9,
    ExceptionWithCode = 10
};

/**
 * @brief Status Conditions of status codes
 */
enum class StatusCondition : uint8_t
{
    Clear = 0,
    Info = 1,
    Warning = 2,
    Error = 3

};

/**
 * @class PinPoint
 * @brief A generic PinPoint class that all clients should inherit for base functionality
 *
 * Performs the shared PinPoint connection protocol with the pinpoint hardware
 */
class PinPoint
{


public:

    /**
     * @brief Initializes the base PinPoint class
     *
     * The underlying io service is setup
     */
    PinPoint();
    ~PinPoint();


    /**
     * @brief Connects the PinPoint driver to the hardware at the provided IPv4 address and Port
     * @param address
     * @param port of client service
     * @return true on sucessful connect, false otherwise
     */
    bool Connect(std::string address, std::string port);

    /**
     * @brief Connects the PinPoint driver to the hardware at the provided IPv4 address and Port
     * @param address IPv4 address of pinpoint hardware
     * @param port of client service
     * @param ec error code set during connect
     * @return true on sucessful connect, false otherwise
     */
    bool Connect(std::string address, std::string port, boost::system::error_code& ec);


    /**
     * @brief Closes PinPoint connection
     *
     * closes all sockets, and stops all threads used for io and processing, blocks until threads are closed
     */
    void Close();

    /**
     * @brief returns connected state
     * @return returns true if connected, false otherwise
     */
    bool connected() { return running_; }

    /**
     * @brief Signaled when PinPoint class is connected to a PinPoint device
     */
    boost::signals2::signal<void ()> onConnect;

    /**
     * @brief Signaled when PinPoint class is disconnected from hardware
     */
    boost::signals2::signal<void ()> onDisconnect;

    /**
     * @brief Signaled on heartbeat received
     */
    boost::signals2::signal<void ()> onHeartbeat;


    /**
     * @brief Getter for address
     * @return the address connected to, returns "" if not connected
     */
    inline std::string getAddress() { return address_; }

    /**
     * @brief Getter for port
     * @return the port connected to, returns "" if not connected
     */
    inline std::string getPort(){return port_;}




protected:

    /**
     * @struct MessageHeader_t
     * @brief helper data structure for parsing pinpoint messages
     */
    typedef struct
    {
        uint8_t control_byte_;
        uint8_t protocol_flag_;
        MessageType message_type_;
        uint8_t size_length_;
        uint8_t message_id_;
    } MessageHeader_t ;

    /**
     * @brief This method is called by the process thread.
     *
     * Classes that inherit from PinPoint should override this method to process the methods
     * themselves.
     *
     * @param msg_type MessageType of the message
     * @param msg_id The id of the message
     * @param msg contents of the message
     */
    virtual void processMessage(MessageType msg_type, uint8_t msg_id, std::vector<uint8_t>& msg);

    /**
     * @brief Connects the give signal, so that the PinPoint device emits the signal
     * @param signal
     */
    void connectSignal(uint8_t signal);


    /**
     * @brief Invokes the getStatus function of the PinPoint device to get all status codes/conditions
     */
    void getStatus();

    std::string address_, port_;

private:

    uint8_t protocolVersion_ = 0x42;
    MessageHeader_t msg_header_;

    boost::asio::io_service io_;
    std::shared_ptr<boost::asio::io_service::work> work_;
    boost::asio::deadline_timer async_deadline_;

    std::shared_ptr<std::thread> io_thread_;
    std::shared_ptr<std::thread> process_thread_;
    bool running_;

    //tcp
    tcp::socket tcp_socket_;
    boost::asio::streambuf tcp_rx_;

    //udp
    udp::socket udp_socket_;
    std::shared_ptr<udp::endpoint> udp_recv_ep_;
    std::vector<uint8_t> udp_rx_buffer_vec_;
    udp::endpoint pinpoint_udp_ep_;

    //process
    std::queue<std::pair<MessageHeader_t, std::vector<uint8_t >>> recv_q_;
    std::condition_variable cv_;
    std::mutex mutex_;

    /**
     * @brief Used to provide timeout to synchronous functions
     *
     * Primarily used by the connect_socket method so that we can timeout gracefully
     */
    void check_deadline();

    /**
     * @brief Provides a syncrronous connection to host and service allowing us to timeout
     * @param host the host to connect to
     * @param service port of the service to connect to
     * @param timeout timeout period
     * @param ec_out error code out
     * @return
     */
    bool connect_socket(const std::string &host, const std::string &service,
                        boost::posix_time::time_duration timeout, boost::system::error_code &ec_out);

    /**
     * @brief Reads header of PinPoint message
     *
     * After succesfully reading the header it will attempt to read size
     * @param err
     */
    void handle_read_header(const boost::system::error_code& err);

    /**
     * @brief Extracts the MessageHeader_t structure from raw data (header field)
     * @param header_start pointer to first byte in header field
     * @return
     */
    MessageHeader_t extract_message_header(uint8_t const * header_start);


    /**
     * @brief Reads the size field of a PinPoint message
     *
     * After sucessfully reading the size if non-zero will read the msg,
     * otherwise attempts to read header of next message
     *
     * @param err
     */
    void handle_read_size(const boost::system::error_code& err);

    /**
     * @brief Extracts the size of bytes to read for the message from raw data
     * @param size_length the length of the size field
     * @param size_start pointer to first byte in size field
     * @return
     */
    uint32_t extract_size_to_read(uint8_t const size_length, uint8_t const *size_start);

    /**
     * @brief Reads the msg field of a PinPoint messsage
     *
     * After reading the message will queue the message for processing,
     * and continue to read next header
     *
     * @param err
     */
    void handle_read_msg(const boost::system::error_code& err);


    /**
     * @brief Reads message arrived on udp socket, will add message to process queue
     * and continue to read next message
     *
     * @param err
     */
    void handle_udp_read(const boost::system::error_code& err, const size_t&);

    /**
     * @brief Starts the udp recv thread
     */
    void start_recv_udp();


    /**
     * @brief maintains the process thread
     *
     * Reads items from queue and processes the message. This thread will block on
     * the condition variable cv_ until an item is added to the queue
     */
    void process();


    /**
     * @brief Sends the getUDP response to pinpoint
     *
     * Informs the pinpoint device of the port we are listening on
     */
    void send_get_udp_response();



};


}
