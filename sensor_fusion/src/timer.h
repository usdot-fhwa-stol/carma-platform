#pragma once

#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

namespace cav {

class Timer {
public:
    Timer(){}
    virtual ~Timer(){}
    virtual boost::posix_time::ptime getTime() {
        return boost::posix_time::microsec_clock::local_time();
    }
};

}