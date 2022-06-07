#pragma once

#include <ArduinoEigen/Eigen/Dense>
#include <vector>

#include "control.hpp"
#include "constants.hpp"
#include "mpc_types.hpp"

namespace rexquad {

class RiccatiSolver {
 public:
  RiccatiSolver(int nhorizon);
  void SetDynamics(const StateMatrix& A, const InputMatrix& B, const ErrorVector& f);
  void SetCost(const StatePenalty& Q, const InputPenalty& R, const StatePenalty& Qf);
  void SetGoalState(const StateVector xf);
  void SetInitialState(const StateVector x0);
  void Solve();

 private:
  void BackwardPass();
  void ForwardPass();

  int nhorizon_;

  // Cost
  StatePenalty Q_;
  ErrorVector q_;
  InputPenalty R_;
  InputVector r_;
  StatePenalty Qf_;
  ErrorVector qf_;

  // Dynamics
  StateMatrix A_;
  InputMatrix B_;
  ErrorVector f_;

  // States and inputs
  StateVector xe_;
  InputVector ue_;
  StateVector x0_;
  StateVector xf_;
  ErrorVector dx0_;
  ErrorVector dxf_;

  // Storage
  std::vector<FeedbackGain> K_;
  std::vector<InputVector> d_;
  std::vector<StateMatrix> P_;
  std::vector<ErrorVector> p_;
  std::vector<ErrorVector> X_;
  std::vector<InputVector> U_;
  std::vector<ErrorVector> Y_;
};

}  // namespace rexquad