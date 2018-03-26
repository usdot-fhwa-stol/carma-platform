#pragma once
#include <boost/signals2/signal.hpp>
#include <vector>
#include <string>

namespace cav
{
enum class CommandMode_t {
    None,
    Wrench,
    ClosedLoop,
    DisableRobotic
};

const double max_commanded_speed = 35.76; //m/s
const double max_commanded_accel = 2.5; //m/s/s

template<class Command>
class CommandProvider
{
public:
    boost::signals2::signal<void (const Command&)> onNewCommand;
    virtual std::vector<std::string>& get_api() = 0;

};


}
