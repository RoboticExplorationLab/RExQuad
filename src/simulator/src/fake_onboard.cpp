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
#include "common/riccati.hpp"
// #include "common/workspace.h"
#include "common/problem_data.h"

extern "C" {
#include "common/delayed_mekf.h"
}

// #include "EmbeddedMPC.h"
// #include "common/osqpsolver.hpp"

// Options
// const int kHeartbeatTimeoutMs = 200;
constexpr int kMaxBufferSize = 100;
// const bool kUseGroundTruth = 0;
std::string pubport = "5555";
std::string subport = "5556";
bool g_verbose = 0;
const double kTimestep = 0.01;
const double kInitialImuBias[6] = {0, 0, 0, 0, 0, 0};
const int kDelayComp = 5;

// Aliases
using Time = uint64_t;
using Pose = rexquad::PoseMsg;
using ControlMsg = rexquad::ControlMsg;
using StateControl = rexquad::StateControlMsg;
using Measurement = rexquad::MeasurementMsg;
using IMUMeasurement = rexquad::IMUMeasurementMsg;
using StateMsg = rexquad::StateMsg;
constexpr int kStateControlSize = sizeof(StateControl) + 1;
constexpr int kStateMsgSize = sizeof(StateMsg) + 1;
constexpr int kControlMsgSize = sizeof(ControlMsg) + 1;

// Controller
rexquad::FeedbackGain K;
rexquad::StateVector xhat;  // state estimate
rexquad::InputVector u;
rexquad::ErrorVector e;  // error state
rexquad::StateVector xeq;
rexquad::InputVector ueq;

// State Estimation
rexquad::StateEstimator filter;
rexquad_DelayedMEKF mekf;

// Globals
Pose posedata;
StateControl statecontrolmsg;
Measurement measurementmsg;
IMUMeasurement imudata;
StateMsg g_statemsg;
ControlMsg g_controlmsg;
bool g_is_filter_initialized = false;

// rexquad::OSQPSolver osqpsolver(nstates, ninputs, nhorizon);  // from problem_data.h
rexquad::RiccatiSolver g_mpc_controller(31);
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
  int delay_comp = 10;
  mekf = rexquad_NewDelayedMEKF(delay_comp);
  g_is_filter_initialized = false;

  // Set up ZMQ Subscriber to get info from simulator
  fmt::print("Setting up ZMQ connections...\n");
  context = zmq_ctx_new();
  setup_subscriber(context, subport);
  setup_publisher(context, pubport);

  // Initialize controller
  fmt::print("Initializing Riccati controller with N = {}...\n",
             g_mpc_controller.GetHorizonLength());
  g_mpc_controller.SetDynamics(dynamics_Adata, dynamics_Bdata, dynamics_fdata, dynamics_xe,
                               dynamics_ue);
  g_mpc_controller.SetCost(cost_Qdata, cost_Rdata, cost_Qfdata);
  g_mpc_controller.SetGoalState(dynamics_xg);
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
  double t = 0.0;  // TODO: get actual time from start of loop

  double y_imu[6];
  double y_mocap[7];
  const double* xhat_;

  switch (msgid) {
    // Update State Estimate
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

      // Save data to arrays
      y_imu[0] = imudata.ax;
      y_imu[1] = imudata.ay;
      y_imu[2] = imudata.az;
      y_imu[3] = imudata.wx;
      y_imu[4] = imudata.wy;
      y_imu[5] = imudata.wz;
      y_mocap[0] = posedata.x;
      y_mocap[1] = posedata.y;
      y_mocap[2] = posedata.z;
      y_mocap[3] = posedata.qw;
      y_mocap[4] = posedata.qx;
      y_mocap[5] = posedata.qy;
      y_mocap[6] = posedata.qz;

      // Initialize filter with first measurement
      if (!g_is_filter_initialized) {
        double x0[13];
        for (int i = 0; i < 7; ++i) {
          x0[i] = y_mocap[i];
        }
        for (int i = 0; i < 6; ++i) {
          x0[7 + i] = 0.0;
        }
        if (g_verbose || 1) {
          fmt::print("Initializing filter with x = [{},{},{}], q = [{},{},{},{}]\n.", x0[0],
                     x0[1], x0[2], x0[3], x0[4], x0[5], x0[6]);
        }
        rexquad_InitializeDelayedMEKF(&mekf, kDelayComp, x0, NULL, NULL, kInitialImuBias,
                                      NULL);
        g_is_filter_initialized = true;
      }

      // Process measurements
      rexquad_UpdateStateEstimate(&mekf, y_imu, y_mocap, kTimestep);

      // Get state estimate from filter
      xhat_ = rexquad_GetStateEstimate(&mekf);
      for (int i = 0; i < 13; ++i) {
        xhat[i] = xhat_[i];
      }

      // Send back state estimate
      if (g_verbose) {
        fmt::print("  Sending back xhat: [ ");
        for (int i = 0; i < 13; ++i) {
          fmt::print("{:.3f} ", xhat_[i]);
        }
        fmt::print("]\n");
      }
      rexquad::StateMsgFromVector(g_statemsg, xhat_);
      rexquad::StateMsgToBytes(g_statemsg, buf_send);
      zmq_send(pub, buf_send, kStateMsgSize, 0);

      break;

    // Calculate control
    case rexquad::StateMsg::MsgID:
      good_conversion = rexquad::StateMsgFromBytes(g_statemsg, buf_recv);
      if (g_verbose) {
        fmt::print("  Successful conversion to StateMsg: {}\n", good_conversion);
      }
      rexquad::StateMsgToVector(g_statemsg, xhat.data());

      // Calculate control
      u = g_mpc_controller.ControlPolicy(xhat, t);
      // rexquad::ErrorState(e, xhat, xeq);
      // u = -K * e + ueq;

      // Send control back over ZMQ
      for (int i = 0; i < 4; ++i) {
        g_controlmsg.data[i] = u[i];
      }
      if (g_verbose) {
        fmt::print("  Sending u = [");
        for (int i = 0; i < 4; ++i) {
          fmt::print(" {}", g_controlmsg.data[i]);
        }
        fmt::print(" ]\n");
      }
      rexquad::ControlMsgToBytes(g_controlmsg, buf_send);
      zmq_send(pub, buf_send, kControlMsgSize, 0);
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