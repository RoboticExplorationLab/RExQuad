#include "sim_utils.hpp"

#include <fmt/core.h>

namespace rexquad {
void print_msg(const MeasurementMsg& msg) {
  fmt::print("  Translation:  [{:.3f}, {:.3f}, {:.3f}]\n", msg.x, msg.y, msg.z);
  fmt::print("  Orientation:  [{:.3f}, {:.3f}, {:.3f}, {:.3f}]\n", msg.qw, msg.qx, msg.qy, msg.qz);
  fmt::print("  Lin Velocity: [{:.3f}, {:.3f}, {:.3f}]\n", msg.vx, msg.vy, msg.vz);
  fmt::print("  Lin Accel:    [{:.3f}, {:.3f}, {:.3f}]\n", msg.ax, msg.ay, msg.az);
  fmt::print("  Ang Velocity: [{:.3f}, {:.3f}, {:.3f}]\n", msg.wx, msg.wy, msg.wz);
}

void print_msg(const PoseMsg& msg) {
  fmt::print("  Translation:  [{:.3f}, {:.3f}, {:.3f}]\n", msg.x, msg.y, msg.z);
  fmt::print("  Orientation:  [{:.3f}, {:.3f}, {:.3f}, {:.3f}]\n", msg.qw, msg.qx, msg.qy, msg.qz);
}

void print_msg(const ControlMsg& msg) {
  fmt::print("  Controls:  [{:.3f}, {:.3f}, {:.3f}, {:.3f}]\n", msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
}

void print_msg(const StateControlMsg& msg) {
  fmt::print("  Translation:  [{:.3f}, {:.3f}, {:.3f}]\n", msg.x, msg.y, msg.z);
  fmt::print("  Orientation:  [{:.3f}, {:.3f}, {:.3f}, {:.3f}]\n", msg.qw, msg.qx, msg.qy, msg.qz);
  fmt::print("  Lin Velocity: [{:.3f}, {:.3f}, {:.3f}]\n", msg.vx, msg.vy, msg.vz);
  fmt::print("  Ang Velocity: [{:.3f}, {:.3f}, {:.3f}]\n", msg.wx, msg.wy, msg.wz);
  fmt::print("  Controls:  [{:.3f}, {:.3f}, {:.3f}, {:.3f}]\n", msg.u[0], msg.u[1], msg.u[2], msg.u[3]);
}
}