#ifndef H_ASIO_BASE
#define H_ASIO_BASE

#include <socketcan_interface/interface.h>
#include <socketcan_interface/dispatcher.h>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

namespace can{


template<typename Socket> class AsioDriver : public DriverInterface{
    typedef FilteredDispatcher<const unsigned int, CommInterface::FrameListener> FrameDispatcher;
    typedef SimpleDispatcher<StateInterface::StateListener> StateDispatcher;
    FrameDispatcher frame_dispatcher_;
    StateDispatcher state_dispatcher_;
  
    State state_;
    boost::mutex state_mutex_;
    boost::mutex socket_mutex_;
    
protected:
    boost::asio::io_service io_service_;
    boost::asio::strand strand_;
    Socket socket_;
    Frame input_;
    
    virtual void triggerReadSome() = 0;
    virtual bool enqueue(const Frame & msg) = 0;
    
    void dispatchFrame(const Frame &msg){
        strand_.post(boost::bind(&FrameDispatcher::dispatch, &frame_dispatcher_, msg)); // copies msg
    }
    void setErrorCode(const boost::system::error_code& error){
        boost::mutex::scoped_lock lock(state_mutex_);
        if(state_.error_code != error){
            state_.error_code = error;
            state_dispatcher_.dispatch(state_);
        }
    }
    void setInternalError(unsigned int internal_error){
        boost::mutex::scoped_lock lock(state_mutex_);
        if(state_.internal_error != internal_error){
            state_.internal_error = internal_error;
            state_dispatcher_.dispatch(state_);
        }
    }

    void setDriverState(State::DriverState state){
        boost::mutex::scoped_lock lock(state_mutex_);
        if(state_.driver_state != state){
            state_.driver_state = state;
            state_dispatcher_.dispatch(state_);
        }
    }
    void setNotReady(){
        setDriverState(socket_.is_open()?State::open : State::closed);
    }
    
    void frameReceived(const boost::system::error_code& error){
        if(!error){
            dispatchFrame(input_);
            triggerReadSome();
        }else{
            setErrorCode(error);
            setNotReady();
        }
    }

    AsioDriver()
    : strand_(io_service_), socket_(io_service_)
    {}

public:
    virtual ~AsioDriver() { shutdown(); }
    
    State getState(){
        boost::mutex::scoped_lock lock(state_mutex_);
        return state_;
    }
    virtual void run(){
        setNotReady();
        
        if(getState().driver_state == State::open){
            io_service_.reset();
            boost::asio::io_service::work work(io_service_);
            setDriverState(State::ready);

            boost::thread post_thread(boost::bind(&boost::asio::io_service::run, &io_service_));
            
            triggerReadSome();
            
            boost::system::error_code ec;
            io_service_.run(ec);
            setErrorCode(ec);
            
            setNotReady();
        }   
        state_dispatcher_.dispatch(getState());
    }
    virtual bool send(const Frame & msg){
        return getState().driver_state == State::ready && enqueue(msg);
    }
    
    virtual void shutdown(){
        if(socket_.is_open()){
            socket_.cancel();
            socket_.close();
        }
        io_service_.stop();
    }
    
    virtual FrameListener::Ptr createMsgListener(const FrameDelegate &delegate){
        return frame_dispatcher_.createListener(delegate);
    }
    virtual FrameListener::Ptr createMsgListener(const Frame::Header&h , const FrameDelegate &delegate){
        return frame_dispatcher_.createListener(h, delegate);
    }
    virtual StateListener::Ptr createStateListener(const StateDelegate &delegate){
        return state_dispatcher_.createListener(delegate);
    }

};

} // namespace can
#endif
