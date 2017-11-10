#ifndef H_CAN_BCM
#define H_CAN_BCM

#include <socketcan_interface/interface.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
 
#include <linux/can.h>
#include <linux/can/bcm.h>
#include <linux/can/error.h>

namespace can {

class BCMsocket{
    int s_;
    struct Message {
        size_t size;
        uint8_t *data;
        Message(size_t n)
        : size(sizeof(bcm_msg_head) + sizeof(can_frame)*n), data(new uint8_t[size])
        {
            assert(n<=256);
            memset(data, 0, size);
            head().nframes = n;
        }
        bcm_msg_head& head() {
            return *(bcm_msg_head*)data;
        }
        template<typename T> void setIVal2(T period){
            long long usec = boost::chrono::duration_cast<boost::chrono::microseconds>(period).count();
            head().ival2.tv_sec = usec / 1000000;
            head().ival2.tv_usec = usec % 1000000;
        }
        void setHeader(Header header){
            head().can_id = header.id | (header.is_extended?CAN_EFF_FLAG:0);
        }
        bool write(int s){
            return ::write(s, data, size) > 0;
        }
        ~Message(){
            delete[] data;
            data = 0;
            size = 0;
        }
    };
public:
    BCMsocket():s_(-1){
    }
    bool init(const std::string &device){
        s_ = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);

        if(s_ < 0 ) return false;
        struct ifreq ifr;
        strcpy(ifr.ifr_name, device.c_str());
        int ret = ioctl(s_, SIOCGIFINDEX, &ifr);

        if(ret != 0){
            shutdown();
            return false;
        }

        struct sockaddr_can addr = {0};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        ret = connect(s_, (struct sockaddr *)&addr, sizeof(addr));

        if(ret < 0){
            shutdown();
            return false;
        }
        return true;
    }
    template<typename DurationType> bool startTX(DurationType period, Header header, size_t num, Frame *frames) {
        Message msg(num);
        msg.setHeader(header);
        msg.setIVal2(period);

        bcm_msg_head &head = msg.head();

        head.opcode = TX_SETUP;
        head.flags |= SETTIMER | STARTTIMER;

        for(size_t i=0; i < num; ++i){ // msg nr
            head.frames[i].can_dlc = frames[i].dlc;
            head.frames[i].can_id = head.can_id;
            for(size_t j = 0; j < head.frames[i].can_dlc; ++j){ // byte nr
                head.frames[i].data[j] = frames[i].data[j];
            }
        }
        return msg.write(s_);
    }
    bool stopTX(Header header){
        Message msg(0);
        msg.head().opcode = TX_DELETE;
        msg.setHeader(header);
        return msg.write(s_);
    }
    void shutdown(){
        if(s_ > 0){
            close(s_);
            s_ = -1;
        }
    }

    virtual ~BCMsocket(){
        shutdown();
    }
};

}

#endif