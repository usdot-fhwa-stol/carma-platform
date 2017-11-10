#ifndef H_SOCKETCAN_DRIVER
#define H_SOCKETCAN_DRIVER

#include <socketcan_interface/asio_base.h>
#include <boost/bind.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
 
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>

#include <socketcan_interface/dispatcher.h>

namespace can {

class SocketCANInterface : public AsioDriver<boost::asio::posix::stream_descriptor> {
    bool loopback_;
    int sc_;
public:    
    SocketCANInterface()
    : loopback_(false), sc_(-1)
    {}
    
    virtual bool doesLoopBack() const{
        return loopback_;
    }

    virtual bool init(const std::string &device, bool loopback){
        State s = getState();
        if(s.driver_state == State::closed){
            sc_ = 0;
            device_ = device;
            loopback_ = loopback;

            int sc = socket( PF_CAN, SOCK_RAW, CAN_RAW );
            if(sc < 0){
                setErrorCode(boost::system::error_code(sc,boost::system::system_category()));
                return false;
            }
            
            struct ifreq ifr;
            strcpy(ifr.ifr_name, device_.c_str());
            int ret = ioctl(sc, SIOCGIFINDEX, &ifr);

            if(ret != 0){
                setErrorCode(boost::system::error_code(ret,boost::system::system_category()));
                close(sc);
                return false;
            }
            can_err_mask_t err_mask =
                ( CAN_ERR_TX_TIMEOUT   /* TX timeout (by netdevice driver) */
                | CAN_ERR_LOSTARB      /* lost arbitration    / data[0]    */
                | CAN_ERR_CRTL         /* controller problems / data[1]    */
                | CAN_ERR_PROT         /* protocol violations / data[2..3] */
                | CAN_ERR_TRX          /* transceiver status  / data[4]    */
                | CAN_ERR_ACK           /* received no ACK on transmission */
                | CAN_ERR_BUSOFF        /* bus off */
                //CAN_ERR_BUSERROR      /* bus error (may flood!) */
                | CAN_ERR_RESTARTED     /* controller restarted */
            ); 

            ret = setsockopt(sc, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
               &err_mask, sizeof(err_mask));
            
            if(ret != 0){
                setErrorCode(boost::system::error_code(ret,boost::system::system_category()));
                close(sc);
                return false;
            }
            
            if(loopback_){
                int recv_own_msgs = 1; /* 0 = disabled (default), 1 = enabled */
                ret = setsockopt(sc, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));
                
                if(ret != 0){
                    setErrorCode(boost::system::error_code(ret,boost::system::system_category()));
                    close(sc);
                    return false;
                }
            }
            
            struct sockaddr_can addr = {0};
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;
            ret = bind( sc, (struct sockaddr*)&addr, sizeof(addr) );            

            if(ret != 0){
                setErrorCode(boost::system::error_code(ret,boost::system::system_category()));
                close(sc);
                return false;
            }
            
            boost::system::error_code ec;
            socket_.assign(sc,ec);
            
            setErrorCode(ec);
            
            if(ec){
                close(sc);
                return false;
            }
            setInternalError(0);
            setDriverState(State::open);
            sc_ = sc;
            return true;
        }
        return getState().isReady();
    }
    virtual bool recover(){
        if(!getState().isReady()){
            shutdown();
            return init(device_, doesLoopBack());
        }
        return getState().isReady();
    }
    virtual bool translateError(unsigned int internal_error, std::string & str){

        bool ret = false;
        if(!internal_error){
            str = "OK";
            ret = true;
        }
        if( internal_error & CAN_ERR_TX_TIMEOUT){
            str += "TX timeout (by netdevice driver);";
            ret = true;
        }
        if( internal_error & CAN_ERR_LOSTARB){
            str += "lost arbitration;";
            ret = true;
        }
        if( internal_error & CAN_ERR_CRTL){
            str += "controller problems;";
            ret = true;
        }
        if( internal_error & CAN_ERR_PROT){
            str += "protocol violations;";
            ret = true;
        }
        if( internal_error & CAN_ERR_TRX){
            str += "transceiver status;";
            ret = true;
        }
        if( internal_error & CAN_ERR_BUSOFF){
            str += "bus off;";
            ret = true;
        }
        if( internal_error & CAN_ERR_RESTARTED){
            str += "ontroller restarted;";
            ret = true;
        }
        return ret;
    }
    int getInternalSocket() {
        return sc_;
    }
protected:
    std::string device_;
    can_frame frame_;
    
    virtual void triggerReadSome(){
        boost::mutex::scoped_lock lock(send_mutex_);
        socket_.async_read_some(boost::asio::buffer(&frame_, sizeof(frame_)), boost::bind( &SocketCANInterface::readFrame,this, boost::asio::placeholders::error));
    }
    
    virtual bool enqueue(const Frame & msg){
        boost::mutex::scoped_lock lock(send_mutex_); //TODO: timed try lock

        can_frame frame = {0};
        frame.can_id = msg.id | (msg.is_extended?CAN_EFF_FLAG:0) | (msg.is_rtr?CAN_RTR_FLAG:0);;
        frame.can_dlc = msg.dlc;
        
        
        for(int i=0; i < frame.can_dlc;++i)
            frame.data[i] = msg.data[i];
        
        boost::system::error_code ec;
        boost::asio::write(socket_, boost::asio::buffer(&frame, sizeof(frame)),boost::asio::transfer_all(), ec);
        if(ec){
            LOG("FAILED " << ec);
            setErrorCode(ec);
            setNotReady();
            return false;
        }
        
        return true;
    }
    
    void readFrame(const boost::system::error_code& error){
        if(!error){
            input_.dlc = frame_.can_dlc;
            for(int i=0;i<frame_.can_dlc && i < 8; ++i){
                input_.data[i] = frame_.data[i];
            }
            
            if(frame_.can_id & CAN_ERR_FLAG){ // error message
                input_.id = frame_.can_id & CAN_EFF_MASK;
                input_.is_error = 1;

                LOG("error: " << input_.id);
                setInternalError(input_.id);
                setNotReady();

            }else{
                input_.is_extended = (frame_.can_id & CAN_EFF_FLAG) ? 1 :0;
                input_.id = frame_.can_id & (input_.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
                input_.is_error = 0;
                input_.is_rtr = (frame_.can_id & CAN_RTR_FLAG) ? 1 : 0;
            }

        }
        frameReceived(error);
    }
private:
    boost::mutex send_mutex_;
};

typedef SocketCANInterface SocketCANDriver;

template <typename T> class ThreadedInterface;
typedef ThreadedInterface<SocketCANInterface> ThreadedSocketCANInterface;


} // namespace can
#endif
