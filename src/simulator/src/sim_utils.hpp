#pragma once

#include "common/messages.hpp"
#include "common/pose.hpp"

namespace rexquad {

void print_msg(const MeasurementMsg& msg);
void print_msg(const PoseMsg& msg);
void print_msg(const ControlMsg& msg);
void print_msg(const StateControlMsg& msg);

}