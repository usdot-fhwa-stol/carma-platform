#pragma once

/*
 * Copyright (C) 2019-2022 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <boost/asio/ip/udp.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/asio/strand.hpp>

#include <iostream>
#include <string>
#include <deque>

namespace cav
{

/**
 * @brief UDPListener is a helper class that manages listening on a UDP socket
 * and passing received packets to subscribers
 */
class UDPListener : public std::enable_shared_from_this<UDPListener>
{
    boost::asio::io_service& io_;
    boost::asio::ip::udp::socket socket_;
    std::atomic<bool> running_;
    bool started_;

    /**
     * @brief starts asynchronous listening on udp socket. Uses
     * reactor-style listening so that we can dynamically detect the length of the packet
     * received
     */
    void start_recv_udp()
    {
        socket_.async_receive(boost::asio::null_buffers(),[this](const boost::system::error_code&ec,size_t bytes){handle_recv_udp(ec,bytes);});
    }

    /**
     * @brief Handles received packets
     * @param ec
     * @param bytes_transferred
     */
    void handle_recv_udp(const boost::system::error_code& ec, std::size_t bytes_transferred)
    {
        if(!ec || ec == boost::asio::error::operation_aborted)
        {
            //Fetch size
            size_t size = socket_.available();

            //Create buffer
            std::shared_ptr<std::vector<uint8_t>> buf = std::make_shared<std::vector<uint8_t>>(size);

            //Perform read
            boost::system::error_code errorCode;
            socket_.receive(boost::asio::buffer(*buf),0,errorCode);
            if(!errorCode)
            {
                //Post signal to handle this received packet
                io_.post([this,buf](){onReceive(buf);});
            }

            //Should only continue if we are still running
            if(running_)
                start_recv_udp();

        }
        else
        {
            onError(ec);
        }

    }

public:

    /**
     * @brief Constructs a UDPListener
     * @param io - an io_service object to service the asynchronous IO
     * @param host - the address to listen on
     * @param port - the port to listen on
     */
    UDPListener(boost::asio::io_service& io, unsigned short port) : io_(io), socket_(io_),running_(false), started_(false)
    {
        socket_.open(boost::asio::ip::udp::v6());
        socket_.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v6(),port));
    }

    virtual ~UDPListener()
    {
        try{
            stop();
        }catch(...)
        {

        }
    }

    /**
     * @brief Called when the internal socket is closed and the UDPListener is no longer listening
     */
    boost::signals2::signal<void ()> onClose;

    /**
     * @brief Called on socket asio error
     */
    boost::signals2::signal<void (const boost::system::error_code&)> onError;

    /**
     * @brief Called on packet received
     */
    boost::signals2::signal<void (const std::shared_ptr< const std::vector<uint8_t>>&)> onReceive;


    /**
     * @brief Starts the
     * @return
     */
    bool start()
    {
        if(running_.exchange(true)) return false;
        start_recv_udp();
        return true;
    }

    void stop()
    {
        if(!running_.exchange(false)) return;
        onClose();
    }
};


};