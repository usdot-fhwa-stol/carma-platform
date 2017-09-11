#pragma once
#include <sstream>
#include <string>
namespace cav
{
namespace can
{


/**
 * @brief structure to store the socket can ErrorCode and produce a valid message from them
 */
struct ErrorCode_t
{
    unsigned int code = 0;
    const std::string what() const {
        std::stringstream ss;

        if(code & 0x1)
            ss << "tx timeout|";
        if(code & 0x2)
            ss << "lost arbitration|";
        if(code & 0x4)
            ss << "controller problems|";
        if(code & 0x8)
            ss << "protocol violation|";
        if(code & 0x10)
            ss << "transceiver error|";
        if(code & 0x20)
            ss << "no ack received|";
        if(code & 0x40)
            ss << "bus off|";
        if(code & 0x80)
            ss << "bus error|";
        if(code & 0x100)
            ss << "controller restarted|";


        std::string ret(ss.str());
        ret.pop_back();
        return ret;
    }
};

}

}