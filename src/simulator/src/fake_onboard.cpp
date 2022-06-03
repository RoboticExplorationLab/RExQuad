#include <fmt/core.h>
#include <zmq.h>

#include <chrono>
#include <string>

#include "common/constants.hpp"
#include "common/control.hpp"
#include "common/estimator.hpp"
#include "common/lqr_constants.hpp"
#include "common/messages.hpp"
#include "common/pose.hpp"
#include "common/workspace.h"
#include "common/problem_data.h"
// #include "EmbeddedMPC.h"
#include "common/osqpsolver.hpp"

// Options
// const int kHeartbeatTimeoutMs = 200;
constexpr int kMaxBufferSize = 100;
// const bool kUseGroundTruth = 0;
std::string pubport = "5555";
std::string subport = "5556";
bool g_verbose = 1;

// Aliases
using Time = uint64_t;
using Pose = rexquad::PoseMsg;
using Control = rexquad::ControlMsg;
using StateControl = rexquad::StateControlMsg;
using Measurement = rexquad::MeasurementMsg;
using IMUMeasurement = rexquad::IMUMeasurementMsg;
constexpr int kStateControlSize = sizeof(StateControl) + 1;

// Controller
rexquad::FeedbackGain K;
rexquad::StateVector xhat;  // state estimate
rexquad::InputVector u;
rexquad::ErrorVector e;  // error state
rexquad::StateVector xeq;
rexquad::InputVector ueq;

// State Estimation
rexquad::StateEstimator filter;

// Globals
Pose posedata;
StateControl statecontrolmsg;
Measurement measurementmsg;
IMUMeasurement imudata;
rexquad::OSQPSolver osqpsolver(nstates, ninputs, nhorizon);  // from problem_data.h
void* context;
void* pub;
void* sub;
auto tstart = std::chrono::high_resolution_clock::now();


// Buffers
uint8_t buf_recv[kMaxBufferSize];
uint8_t buf_send[kStateControlSize];

template <class Vector>
void PrintVector(const Vector& x) {
  fmt::print("[ ");
  for (const auto& xi : x) {
    fmt::print("{:.3f} ", xi);
  }
  fmt::print("]\n");
}

int setup_subscriber(void* context, const std::string& subport) {
  // Set up subscriber
  sub = zmq_socket(context, ZMQ_SUB);
  int rc;

  // Set socket options
  int conflate = 1;
  rc = zmq_setsockopt(sub, ZMQ_CONFLATE, &conflate, sizeof(conflate));
  if (rc != 0) {
    printf("Failed to set conflate.\n");
    return 1;
  }
  std::string tcpaddress_sub = "tcp://127.0.0.1:" + subport;
  rc = zmq_connect(sub, tcpaddress_sub.c_str());
  if (rc != 0) {
    printf("Failed to connect subscriber.\n");
    return 1;
  } else {
    fmt::print("Connected subscriber at {}\n", tcpaddress_sub);
  }
  rc = zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "", 0);
  if (rc != 0) {
    printf("Failed to set subscriber.\n");
    return 1;
  }
  return 0;
}

int setup_publisher(void* context, const std::string& pubport) {
  // Set up ZMQ publisher
  pub = zmq_socket(context, ZMQ_PUB);
  std::string tcpaddress_pub = "tcp://127.0.0.1:" + pubport;
  int rc = zmq_connect(pub, tcpaddress_pub.c_str());
  if (rc != 0) {
    printf("Failed to connect publisher.\n");
    return 1;
  } else {
    fmt::print("Connected publisher at {}\n", tcpaddress_pub);
  }
  return 0;
}

void UpdateStateControlMsg(StateControl& msg, const rexquad::StateVector& x,
                           const rexquad::InputVector& u) {
  msg.x = x[0];
  msg.y = x[1];
  msg.z = x[2];
  msg.qw = x[3];
  msg.qx = x[4];
  msg.qy = x[5];
  msg.qz = x[6];
  msg.vx = x[7];
  msg.vy = x[8];
  msg.vz = x[9];
  msg.wx = x[10];
  msg.wy = x[11];
  msg.wz = x[12];
  msg.u[0] = u[0];
  msg.u[1] = u[1];
  msg.u[2] = u[2];
  msg.u[3] = u[3];
}

void MeasurementMsgToStateVector(rexquad::StateVector& x,
                                 const rexquad::MeasurementMsg measurementmsg) {
  x[0] = measurementmsg.x;
  x[1] = measurementmsg.y;
  x[2] = measurementmsg.z;
  x[3] = measurementmsg.qw;
  x[4] = measurementmsg.qx;
  x[5] = measurementmsg.qy;
  x[6] = measurementmsg.qz;
  x[7] = measurementmsg.vx;
  x[8] = measurementmsg.vy;
  x[9] = measurementmsg.vz;
  x[10] = measurementmsg.wx;
  x[11] = measurementmsg.wy;
  x[12] = measurementmsg.wz;
}

void setup() {
  // Initialize LQR controller
  for (int i = 0; i < K.size(); ++i) {
    K(i) = rexquad::kFeedbackGain[i];
  }
  for (int i = 0; i < xeq.size(); ++i) {
    xeq(i) = rexquad::kStateEquilibrium[i];
  }
  for (int i = 0; i < ueq.size(); ++i) {
    ueq(i) = rexquad::kInputEquilibrium[i];
  }

  // Filter settings
  filter.SetIntegrateLinearAccel(false);

  // Set up ZMQ Subscriber to get info from simulator
  fmt::print("Setting up ZMQ connections...\n");
  context = zmq_ctx_new();
  setup_subscriber(context, subport);
  setup_publisher(context, pubport);

  // Initialize controller
  fmt::print("Initializing controller...\n");
  rexquad::MPCProblem& prob = osqpsolver.GetProblem();
  prob.SetDynamics(dynamics_Adata, dynamics_Bdata, dynamics_fdata);
  prob.SetCostTerminal(cost_Qfdata, cost_qfdata);
  prob.SetCostState(cost_Qdata, cost_qdata);
  prob.SetCostInput(cost_Rdata, cost_rdata);
  prob.SetCostConstant(cost_c);
  osqpsolver.Initialize(&workspace);
  fmt::print("Controller Initialized!\n");

  // Start timer
  tstart = std::chrono::high_resolution_clock::now();

  fmt::print("Starting loop...\n");
}

void loop() {
  bool send_message = false;
  bool use_ground_truth = false;

  // Receive measurement message from simulator
  if (g_verbose) {
    fmt::print("Waiting for ZMQ Message...\n");
  }
  int zmq_bytes_received = zmq_recv(sub, buf_recv, kMaxBufferSize, 0);
  int msgid = buf_recv[0];
  bool good_conversion;
  if (g_verbose) {
    fmt::print("\nReceived {} bytes over ZMQ with MsgID {}\n", zmq_bytes_received,
               buf_recv[0]);
  }

  auto tnow = std::chrono::high_resolution_clock::now();
  auto t_msg_us = std::chrono::duration_cast<std::chrono::microseconds>(tnow - tstart);
  switch (msgid) {
    case rexquad::MeasurementMsg::MsgID:
      good_conversion = MeasurementMsgFromBytes(measurementmsg, buf_recv);

      if (g_verbose) {
        fmt::print("  Successful conversion to MeasurementMsg: {}\n", good_conversion);
      }

      // Extract Pose information
      posedata.x = measurementmsg.x;
      posedata.y = measurementmsg.y;
      posedata.z = measurementmsg.z;
      posedata.qw = measurementmsg.qw;
      posedata.qx = measurementmsg.qx;
      posedata.qy = measurementmsg.qy;
      posedata.qz = measurementmsg.qz;

      // Extract IMU Measurement
      imudata.ax = measurementmsg.ax;
      imudata.ay = measurementmsg.ay;
      imudata.az = measurementmsg.az;
      imudata.wx = measurementmsg.wx;
      imudata.wy = measurementmsg.wy;
      imudata.wz = measurementmsg.wz;

      // Update Filter
      filter.IMUMeasurement(imudata, t_msg_us.count());
      filter.PoseMeasurement(posedata, t_msg_us.count());
      send_message = true;
      use_ground_truth = true;
      break;
    case rexquad::IMUMeasurementMsg::MsgID:
      good_conversion = IMUMeasurementMsgFromBytes(imudata, buf_recv);
      if (g_verbose) {
        fmt::print("  Successful conversion to IMUMeasurementMsg: {}\n", good_conversion);
      }

      // Update Filter
      filter.IMUMeasurement(imudata, t_msg_us.count());
      send_message = true;
      break;
    case rexquad::PoseMsg::MsgID:
      good_conversion = rexquad::PoseFromBytes(posedata, (char*)buf_recv);
      if (g_verbose) {
        fmt::print("  Successful conversion to PoseMsg: {}\n", good_conversion);
        fmt::print("  position = [{:.3}, {:.3}, {:.3}]\n", posedata.x, posedata.y,
                   posedata.z);
      }

      // Update Filter
      filter.PoseMeasurement(posedata, t_msg_us.count());
      send_message = false;  // Doesn't send a message back if a pose message is received
      break;
  }

  if (send_message) {
    // Get current state estimate
    if (use_ground_truth) {
      MeasurementMsgToStateVector(xhat, measurementmsg);
    } else {
      filter.GetStateEstimate(xhat);
    }

    // Calculate control
    rexquad::ErrorState(e, xhat, xeq);
    // osqpsolver.SetInitialState(xhat.data());
    // bool solve_successful = osqpsolver.Solve();
    // if (!solve_successful) {
    //   fmt::print("OSQP Solve Failed!\n");
    // }
    // osqpsolver.GetInput(u.data(), 0);
    
    // for (int i = 0; i < ninputs; ++i) {
    //   u[i] += rexquad::kHoverInput;
    // }
    u = -K * e + ueq;

    // Send control and state estimate back over ZMQ to simulator
    UpdateStateControlMsg(statecontrolmsg, xhat, u);
    if (g_verbose) {
      if (use_ground_truth) {
        fmt::print("  Using ground truth estimate\n");
      }
      fmt::print("State Estimate: ");
      PrintVector(xhat);
      fmt::print("State error: ");
      PrintVector(e);
      fmt::print("Control: ");
      PrintVector(u);
    }
    rexquad::StateControlMsgToBytes(statecontrolmsg, buf_send);
    zmq_send(pub, buf_send, kStateControlSize, 0);
  }
}

int main(int argc, char** argv) {
  // Read inputs
  if (argc > 1) {
    subport = argv[1];
  }
  if (argc > 2) {
    pubport = argv[2];
  }

  setup();

  while (1) {
    loop();
  }

  zmq_close(sub);
  zmq_close(pub);
  fmt::print("Closed publisher and subscriber\n");
  zmq_ctx_shutdown(context);
  fmt::print("ZMQ shut down\n");
  zmq_ctx_term(context);
  return 0;
}