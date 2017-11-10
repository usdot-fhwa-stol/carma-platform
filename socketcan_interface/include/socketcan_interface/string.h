#ifndef SOCKETCAN_INTERFACE_STRING_H
#define SOCKETCAN_INTERFACE_STRING_H

#include "interface.h"
#include "filter.h"
#include <sstream>

namespace can {

bool hex2dec(uint8_t& d, const char& h);

bool hex2buffer(std::string& out, const std::string& in_raw, bool pad);

bool dec2hex(char& h, const uint8_t& d, bool lc);

std::string byte2hex(const uint8_t& d, bool pad, bool lc);


std::string buffer2hex(const std::string& in, bool lc);

std::string tostring(const Header& h, bool lc);

Header toheader(const std::string& s);

std::string tostring(const Frame& f, bool lc);

Frame toframe(const std::string& s);

template<class T> FrameFilter::Ptr tofilter(const T  &ct);
template<> FrameFilter::Ptr tofilter(const std::string &s);
template<> FrameFilter::Ptr tofilter(const uint32_t &id);

FrameFilter::Ptr tofilter(const char* s);

template <typename T> FilteredFrameListener::FilterVector tofilters(const T& v) {
    FilteredFrameListener::FilterVector filters;
    for(size_t i = 0; i < static_cast<size_t>(v.size()); ++i){
        filters.push_back(tofilter(v[i]));
    }
    return filters;
}

}

std::ostream& operator <<(std::ostream& stream, const can::Header& h);
std::ostream& operator <<(std::ostream& stream, const can::Frame& f);

#endif
