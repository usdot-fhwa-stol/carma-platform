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

#include <pinpoint_driver/unpack_macros.h>
#include <pinpoint_driver/pinpoint_driver.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/endian/conversion.hpp>

torc::PinPoint::PinPoint() :
        tcp_socket_(io_),
        udp_socket_(io_),
        async_deadline_(io_),
        running_(false)
{
    async_deadline_.expires_from_now(boost::posix_time::pos_infin);
    check_deadline();
}

torc::PinPoint::~PinPoint() 
{
    Close();
}

bool torc::PinPoint::Connect(std::string address, std::string port) 
{
    boost::system::error_code ec;
    return Connect(address,port,ec);
}

bool torc::PinPoint::Connect(std::string addr, std::string port, boost::system::error_code& ec) 
{
    std::cout << "Attempting to connect: " << addr << ":" << port << std::endl;

    if(!connect_socket(addr, port, boost::posix_time::seconds(5), ec) || ec)
    {
        tcp_socket_.close();
        return false;
    }

    // PinPoint expects the first message received to be the protocol version
    boost::asio::write(tcp_socket_,boost::asio::buffer(&protocolVersion_,1),ec);
    if(ec)
    {
        tcp_socket_.close();
        return false;
    }

    uint8_t recvdProtocolVersion;
    boost::asio::read(tcp_socket_,boost::asio::buffer(&recvdProtocolVersion,1),ec);

    if(ec)
    {
        tcp_socket_.close();
        return false;
    }

    // Verify major version matches
    if((0xf0 & recvdProtocolVersion) != (0xf0 & protocolVersion_))
    {
        tcp_socket_.close();
        return false;
    }

    // Send our api, which is empty
    std::vector<uint8_t> empty_api = {0x89, 0x01, 0x03, 0, 0, 0};
    boost::asio::write(tcp_socket_,boost::asio::buffer(empty_api),ec);

    if(ec)
    {
        tcp_socket_.close();
        return false;
    }

    udp_socket_.open(boost::asio::ip::udp::v4(),ec);

    if(ec)
    {
        tcp_socket_.close();
        udp_socket_.close();
        return false;
    }

    udp_recv_ep_.reset(new boost::asio::ip::udp::endpoint());
    udp_socket_.bind(*udp_recv_ep_);

    // We need to get the udp loc_port of PinPoint
    uint8_t send_get_udp_port[2] = {0x90, 0x05};
    boost::asio::write(tcp_socket_, boost::asio::buffer(send_get_udp_port, 2),boost::asio::transfer_all(), ec);


    boost::asio::async_read(tcp_socket_,
                            tcp_rx_,
                            boost::asio::transfer_exactly(2),
                            [this](const boost::system::error_code& e, const unsigned long int&){ handle_read_header(e); });

    start_recv_udp();

    work_.reset(new boost::asio::io_service::work(io_));
    io_thread_.reset(new std::thread([this](){io_.run();}));

    address_ = addr;
    port_ = port;

    running_ = true;
    process_thread_.reset(new std::thread([this](){process();}));
    onConnect();
    return true;
}

void torc::PinPoint::check_deadline() 
{
    if(async_deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now())
    {
        boost::system::error_code ec;
        tcp_socket_.close(ec);

        async_deadline_.expires_at(boost::posix_time::pos_infin);
    }

    async_deadline_.async_wait(boost::bind(&PinPoint::check_deadline, this));
}

bool torc::PinPoint::connect_socket(const std::string &host, const std::string &service,
                                    boost::posix_time::time_duration timeout, boost::system::error_code &ec_out) 
{
    tcp::resolver::query query(host, service);
    tcp::resolver::iterator iter = tcp::resolver(io_).resolve(query);

    async_deadline_.expires_from_now(timeout);
    boost::system::error_code ec = boost::asio::error::would_block;

    boost::asio::async_connect(tcp_socket_, iter, boost::lambda::var(ec) = boost::lambda::_1);

    do io_.run_one(); while (ec == boost::asio::error::would_block);

    async_deadline_.expires_from_now(boost::posix_time::pos_infin);
    if (ec || !tcp_socket_.is_open())
    {
        ec_out = ec;
        return false;
    }

    return true;
}

void torc::PinPoint::handle_read_header(const boost::system::error_code &err) 
{
    if(!err)
    {
        // extract data
        std::vector<uint8_t> data(tcp_rx_.size());
        boost::asio::buffer_copy(boost::asio::buffer(data),tcp_rx_.data());
        tcp_rx_.consume(tcp_rx_.size());
        msg_header_ = extract_message_header(&data[0]);

        // if the size length is not zero we need to read the size
        if(msg_header_.size_length_ != 0)
        {
            boost::asio::async_read(tcp_socket_,
                                    tcp_rx_,
                                    boost::asio::transfer_exactly((size_t)0x01 << (msg_header_.size_length_ - 1)),
                                    [this](const boost::system::error_code& e, const unsigned long int&){ handle_read_size(e); });
        }
        else
        {
            // Push the empty message onto the queue
            {
                std::lock_guard<std::mutex> lock(mutex_);
                recv_q_.push(std::make_pair(msg_header_, std::vector<uint8_t>()));
            }
            cv_.notify_all();

            // begin reading the next header
            boost::asio::async_read(tcp_socket_,
                                    tcp_rx_,
                                    boost::asio::transfer_exactly(2),
                                    [this](const boost::system::error_code& e, const unsigned long int&){ handle_read_header(e); });
        }
    }
    else if (err != boost::asio::error::eof)
    {
        // Do nothing
    }
}

torc::PinPoint::MessageHeader_t torc::PinPoint::extract_message_header(uint8_t const *header_start)
{
    MessageHeader_t msg_header;

    msg_header.control_byte_ = header_start[0];
    msg_header.message_id_ = header_start[1];

    msg_header.protocol_flag_ = ( (uint8_t)0x80 & msg_header.control_byte_ ) >> 0x07;
    msg_header.message_type_  = (MessageType)(( 0x78 & msg_header.control_byte_ ) >> 0x03);
    msg_header.size_length_   = ( (uint8_t)0x03 & msg_header.control_byte_);

    return msg_header;
}


void torc::PinPoint::handle_read_size(const boost::system::error_code &err) 
{
    if(!err)
    {
        // extract the size message into usable buffer
        std::vector<uint8_t> data(tcp_rx_.size());
        boost::asio::buffer_copy(boost::asio::buffer(data),tcp_rx_.data());
        tcp_rx_.consume(tcp_rx_.size());

        // determine the size of the message to read
        uint32_t sizeToRead = extract_size_to_read(msg_header_.size_length_, &data[0]);

        // read the message
        boost::asio::async_read(tcp_socket_,
                                tcp_rx_,
                                boost::asio::transfer_exactly(sizeToRead),
                                [this](const boost::system::error_code& e, const unsigned long int&){ handle_read_msg(e); });
    }
    else if (err != boost::asio::error::eof)
    {
        // Do nothing
    }
}

void torc::PinPoint::handle_read_msg(const boost::system::error_code &err) 
{
    if(!err)
    {
        std::vector<uint8_t> data(tcp_rx_.size());
        boost::asio::buffer_copy(boost::asio::buffer(data),tcp_rx_.data());
        tcp_rx_.consume(tcp_rx_.size());

        {
            std::lock_guard<std::mutex> lock(mutex_);
            recv_q_.push(std::make_pair(msg_header_,data));

        }
        cv_.notify_all();

        boost::asio::async_read(tcp_socket_,
                                tcp_rx_,
                                boost::asio::transfer_exactly(2),
                                [this](const boost::system::error_code& e, const unsigned long int&){ handle_read_header(e); });
    }
    else if (err != boost::asio::error::eof)
    {
        // Do nothing
    }
}

void torc::PinPoint::start_recv_udp()
{
    udp_rx_buffer_vec_.resize(2048);
    udp_socket_.async_receive_from(boost::asio::buffer(udp_rx_buffer_vec_),
                                   *udp_recv_ep_,
                                   [this](const boost::system::error_code& ec,const size_t& n){ handle_udp_read(ec, n); });
}

void torc::PinPoint::handle_udp_read(const boost::system::error_code &err, const size_t &n) 
{
    if(!err)
    {
        if(n <= 0)
        {
            udp_socket_.async_receive_from(boost::asio::buffer(udp_rx_buffer_vec_),
                                           *udp_recv_ep_,
                                           [this](const boost::system::error_code& ec,const size_t& n){ handle_udp_read(ec, n); });
            return;
        }

        MessageHeader_t msg_header = extract_message_header(&udp_rx_buffer_vec_[0]);

        uint32_t sizeToRead = extract_size_to_read(msg_header.size_length_, &udp_rx_buffer_vec_[2]);

        std::vector<uint8_t> data(udp_rx_buffer_vec_.begin()+2+msg_header.size_length_,udp_rx_buffer_vec_.begin()+2+msg_header.size_length_+sizeToRead);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            recv_q_.push(std::make_pair(msg_header,data));
        }
        cv_.notify_all();

        udp_socket_.async_receive_from(boost::asio::buffer(udp_rx_buffer_vec_),
                                       *udp_recv_ep_,
                                       [this](const boost::system::error_code& ec,const size_t& n){ handle_udp_read(ec, n); });
    }
    else if (err != boost::asio::error::eof)
    {
        // Do nothing
    }
}

void torc::PinPoint::process() 
{
    bool received_udp_port = false; // todo: this is a bit hacky , state machine may be overkill tho
    while(running_)
    {
        std::pair<MessageHeader_t, std::vector<uint8_t >> entry;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if(recv_q_.empty())
            {
                cv_.wait(lock, [this](){ return !recv_q_.empty() || !running_; });
            }

            if(!running_) break;
            entry = recv_q_.front();
            recv_q_.pop();
        }

        if(entry.first.protocol_flag_ == 0)
        {
            processMessage(entry.first.message_type_, entry.first.message_id_, entry.second);
        }

        switch(entry.first.message_type_)
        {
            case MessageType::ReturnValue: // Return values from PinPoint
            { 
                switch(entry.first.message_id_)
                {
                    case 0x05: // getUDPPort - Return
                    {
                        uint16_t port;
                        std::memcpy(&port, &entry.second[0], 2);
                        port = boost::endian::little_to_native(port);
                        udp::resolver resolver(io_);
                        udp::resolver::query query(udp::v4(), address_,std::to_string(port));
                        udp::resolver::iterator it = resolver.resolve(query);
                        pinpoint_udp_ep_ = *it;
                        received_udp_port = true;
                    }
                    default:
                    {
                        break;
                    }
                }
                break;
            }
            case MessageType::Invoke:
            {
                break;
            }
            case MessageType::InvokeWithReturn:
            {
                switch (entry.first.message_id_)
                {
                    case 0x05: // getUDPPort
                    {
                        send_get_udp_response();
                        break;
                    }
                    case 0x06: // UDPPing
                    {
                        onHeartbeat();
                        if(!received_udp_port)
                        {
                            break;
                        }

                        std::vector<uint8_t> resp = {0x81, 0x06, (uint8_t)entry.second.size()};
                        resp.insert(resp.end(), entry.second.begin(), entry.second.end());

                        // let the io_service send this asynchronous while simultaneously capturing the message
                        io_.post([this, resp](){udp_socket_.send_to(boost::asio::buffer(resp), pinpoint_udp_ep_); });
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
                break;
            }
            case MessageType::SignalEmitted:
            case MessageType::ScheduleReturn:
            case MessageType::Error:
            case MessageType::InvokeWithReturnCode:
            case MessageType::ReturnValueWithCode:
            case MessageType::Exception:
            case MessageType::ScheduleException:
            case MessageType::ExceptionWithCode:
            default:
            {
                break;
            }
        }
    }
}

void torc::PinPoint::send_get_udp_response()
{
    uint8_t send_udp_response[5] = {0x81, 0x05, 0x02, 0, 0};
    uint16_t local_udp_port = boost::endian::native_to_little(udp_socket_.local_endpoint().port());

    memcpy(&send_udp_response[3], &local_udp_port, sizeof(uint8_t)*2);

    // We don't want to send this from the process thread queue this to be sent on io thread
    io_.post([this, send_udp_response](){boost::asio::write(tcp_socket_, boost::asio::buffer(send_udp_response, 5), boost::asio::transfer_all()); });
}

void torc::PinPoint::processMessage(MessageType msg_type, uint8_t msg_id, std::vector<uint8_t>& msg)  
{
    // nothing to do here, inheriting classes should override this method
}


uint32_t torc::PinPoint::extract_size_to_read(uint8_t const size_length, uint8_t const *size_start) 
{
    switch(size_length)
    {
        case 1:
        {
            return UNPACK_UINT8(size_start);
        }
        case 2:
        {
            return UNPACK_UINT16(size_start);
        }
        case 3:
        {
            return UNPACK_UINT32(size_start);
        }
        default:
        {
            return 0;
        }
    }
}

void torc::PinPoint::connectSignal(uint8_t signal)
{
    if(!running_) 
    {
        return;
    }

    io_.post([this,signal](){
        std::vector<uint8_t> msg = {0x91, 0x02, 0x02, signal, 0x01};
        boost::asio::write(tcp_socket_, boost::asio::buffer(msg, 5), boost::asio::transfer_all());
    });
}

void torc::PinPoint::Close() 
{
    // check to see if we have closed or are closing
    if(!running_) 
    {
        return;
    }

    running_ = false;

    io_.stop();
    work_.reset();

    // check to see if the thread that called closed is the io thread if so we should't join here
    if(io_thread_ && io_thread_->get_id() != std::this_thread::get_id())
    {
        io_thread_->join();
    }

    tcp_socket_.close();
    udp_socket_.close();
    cv_.notify_all();

    if(process_thread_)
    {
        process_thread_->join();
    }

    while(!recv_q_.empty()) 
    {
        recv_q_.pop();
    }

    io_.reset();
    address_.clear();
    port_.clear();

    onDisconnect();
}

void torc::PinPoint::getStatus()
{
    if(!running_)
    {
        return;
    }

    static std::vector<uint8_t> msg = {0b00010001, 0x02, 0x01, 0x0};

    // We don't want to send this from the process thread queue this to be sent on io thread
    io_.post([this](){
        boost::asio::write(tcp_socket_, boost::asio::buffer(msg, 4), boost::asio::transfer_all());
    });
}