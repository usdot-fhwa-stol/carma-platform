#ifndef H_CAN_INTERFACE
#define H_CAN_INTERFACE

#include <boost/array.hpp>
#include <boost/system/error_code.hpp>
#include <boost/shared_ptr.hpp>

#include "FastDelegate.h"

namespace can{

/** Header for CAN id an meta data*/
struct Header{
    static const unsigned int ID_MASK = (1u << 29)-1;
    static const unsigned int ERROR_MASK = (1u << 29);
    static const unsigned int RTR_MASK = (1u << 30);
    static const unsigned int EXTENDED_MASK = (1u << 31);
    
    unsigned int id:29; ///< CAN ID (11 or 29 bits valid, depending on is_extended member
    unsigned int is_error:1; ///< marks an error frame (only used internally)
    unsigned int is_rtr:1; ///< frame is a remote transfer request
    unsigned int is_extended:1; ///< frame uses 29 bit CAN identifier 
    /** check if frame header is valid*/
    bool isValid() const{
        return id < (is_extended?(1<<29):(1<<11));
    }
    /** constructor with default parameters
        * @param[in] i: CAN id, defaults to 0
        * @param[in] extended: uses 29 bit identifier, defaults to false
        * @param[in] rtr: is rtr frame, defaults to false
        */
    
    operator const unsigned int() const { return is_error ? (ERROR_MASK) : *(unsigned int*) this; }

    Header()
    : id(0),is_error(0),is_rtr(0), is_extended(0) {}
    
    Header(unsigned int i, bool extended, bool rtr, bool error)
    : id(i),is_error(error?1:0),is_rtr(rtr?1:0), is_extended(extended?1:0) {}
};


struct MsgHeader : public Header{
    MsgHeader(unsigned int i=0, bool rtr = false) : Header(i, false, rtr, false) {}
};
struct ExtendedHeader : public Header{
    ExtendedHeader(unsigned int i=0, bool rtr = false) : Header(i, true, rtr, false) {}
};
struct ErrorHeader : public Header{
    ErrorHeader(unsigned int i=0) : Header(i, false, false, true) {}
};

    
    
/** reprentation of a CAN frame */   
struct Frame: public Header{
    boost::array<unsigned char, 8> data; ///< array for 8 data bytes with bounds checking
    unsigned char dlc; ///< len of data
    
    /** check if frame header and length are valid*/
    bool isValid() const{
        return (dlc <= 8)  &&  Header::isValid();
    }
    /** 
     * constructor with default parameters
     * @param[in] i: CAN id, defaults to 0
     * @param[in] l: number of data bytes, defaults to 0
     * @param[in] extended: uses 29 bit identifier, defaults to false
     * @param[in] rtr: is rtr frame, defaults to false
     */
    Frame() : Header(), dlc(0) {}
    Frame(const Header &h, unsigned char l = 0) : Header(h), dlc(l) {}
};

/** extended error information */
class State{
public:
    enum DriverState{
        closed, open, ready
    } driver_state;
    boost::system::error_code error_code; ///< device access error
    unsigned int internal_error; ///< driver specific error 
    
    State() : driver_state(closed), internal_error(0) {}
    virtual bool isReady() const { return driver_state == ready; }
    virtual ~State() {}
};

/** template for Listener interface */
template <typename T,typename U> class Listener{
    const T callable_;
public:
    typedef U Type;
    typedef T Callable;
    typedef boost::shared_ptr<const Listener> Ptr;
    
    Listener(const T &callable):callable_(callable){ }
    void operator()(const U & u) const { if(callable_) callable_(u); }
    virtual ~Listener() {} 
};

class StateInterface{
public:
    typedef fastdelegate::FastDelegate1<const State&> StateDelegate;
    typedef Listener<const StateDelegate, const State&> StateListener;

    /**
     * acquire a listener for the specified delegate, that will get called for all state changes
     * 
     * @param[in] delegate: delegate to be bound by the listener
     * @return managed pointer to listener
     */
    virtual StateListener::Ptr createStateListener(const StateDelegate &delegate) = 0;

    virtual ~StateInterface() {}
};

class CommInterface{
public:
    typedef fastdelegate::FastDelegate1<const Frame&> FrameDelegate;
    typedef Listener<const FrameDelegate, const Frame&> FrameListener;

    /**
     * enqueue frame for sending
     * 
     * @param[in] msg: message to be enqueued
     * @return true if frame was enqueued succesfully, otherwise false
     */
    virtual bool send(const Frame & msg) = 0;
    
    /**
     * acquire a listener for the specified delegate, that will get called for all messages
     * 
     * @param[in] delegate: delegate to be bound by the listener
     * @return managed pointer to listener
     */
    virtual FrameListener::Ptr createMsgListener(const FrameDelegate &delegate) = 0;
    
    /**
     * acquire a listener for the specified delegate, that will get called for messages with demanded ID
     * 
     * @param[in] header: CAN header to restrict listener on
     * @param[in] delegate: delegate to be bound listener
     * @return managed pointer to listener
     */
    virtual FrameListener::Ptr createMsgListener(const Frame::Header&, const FrameDelegate &delegate) = 0;
    
    virtual ~CommInterface() {}
};

class DriverInterface : public CommInterface, public StateInterface {
public:
    /**
     * initialize interface
     * 
     * @param[in] device: driver-specific device name/path
     * @param[in] loopback: loop-back own messages
     * @return true if device was initialized succesfully, false otherwise
     */
    virtual bool init(const std::string &device, bool loopback) = 0;
    
    /**
     * Recover interface after errors and emergency stops
     * 
     * @return true if device was recovered succesfully, false otherwise
     */
    virtual bool recover() = 0;

    /**
     * @return current state of driver
     */
    virtual State getState() = 0;

    /**
     * shutdown interface
     *
     * @return true if shutdown was succesful, false otherwise
     */
    virtual void shutdown() = 0;
    
    virtual bool translateError(unsigned int internal_error, std::string & str) = 0;
    
    virtual bool doesLoopBack() const = 0;
    
    virtual void run()  = 0;
    
    virtual ~DriverInterface() {}
};


} // namespace can

#include <boost/thread/mutex.hpp>

struct _cout_wrapper{
    static boost::mutex& get_cout_mutex(){
        static boost::mutex mutex;
        return mutex;
    }
};

#define LOG(log) { boost::mutex::scoped_lock _cout_lock(_cout_wrapper::get_cout_mutex()); std::cout << log << std::endl; }


#endif
