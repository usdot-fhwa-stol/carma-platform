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

#include <memory>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <vector>


#include "udp_listener.h"

class DSRCOBUClient
{

public:
    /**
     * @brief Initializes the DSRC client, sets up i/o services
     */
    DSRCOBUClient();
    ~DSRCOBUClient();

    /**
    * @brief Connects the driver to the OBU at the provided IPv4 address and Port
    * @param address IPv4 address of OBU
    * @param port of client service
    * @param ec error code set during connect
    * @return true on sucessful connect, false otherwise
    */
    bool connect(const std::string &remote_address,unsigned short remote_port,
                        unsigned short local_port,boost::system::error_code &ec);


    bool connect(const std::string &remote_address, unsigned short remote_port,
                 unsigned short local_port);

    /**
    * @brief Closes connection
    *
    * closes all sockets, and stops all threads used for io and processing, blocks until threads are closed
    */
    void close();

    /**
    * @brief returns connected state
    * @return returns true if connected, false otherwise
    */
    inline bool connected() { return running_; }

    /**
    * @brief Signaled when client is connected to an DSRC device
    */
    boost::signals2::signal<void()> onConnect;

    /**
    * @brief Signaled when client is disconnected from hardware
    */
    boost::signals2::signal<void()> onDisconnect;

    /**
     * @brief Signaled when client experiences a fatal error. Client should be closed and reconnected
     */
    boost::signals2::signal<void(const boost::system::error_code&)> onError;

    /**
    * @brief Signaled when message received
    */
    boost::signals2::signal<void(std::vector<uint8_t> const &, uint16_t id)> onMessageReceived;

    /**
     * @brief sends a udp message
     */
    bool sendDsrcMessage(const std::shared_ptr<std::vector<uint8_t>>&message);


private:
    std::unique_ptr<boost::asio::io_service> io_;
    std::unique_ptr<boost::asio::io_service::strand> output_strand_;
    std::shared_ptr<boost::asio::io_service::work> work_;

    std::shared_ptr<std::thread> io_thread_;
    volatile bool running_;

    //udp
    std::unique_ptr<boost::asio::ip::udp::socket> udp_out_socket_;
    boost::asio::ip::udp::endpoint remote_udp_ep_;
    std::unique_ptr<cav::UDPListener> udp_listener_;

    /**
    * @brief maintains the process thread
    *
    * This will parse through the incoming UDP packets to find what looks like a
    * valid J2735 message. The UPER scheme makes it hard to definitively know if
    * a message is valid or just noise, so all possible messages from a packet are
    * sent. This will create some false positives, but will ensure that a falsely
    * identified message doesn't keep a real one from getting through.
    */
    void process(const std::shared_ptr<const std::vector<uint8_t>> &data);
};
