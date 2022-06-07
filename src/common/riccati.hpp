#pragma once

#include <ArduinoEigen/Eigen/Dense>
#include <vector>

#include "constants.hpp"
#include "control.hpp"
#include "mpc_types.hpp"

namespace rexquad {

class RiccatiSolver {
 public:
  RiccatiSolver(int nhorizon);
  void SetDynamics(const mpc_float* Adata, const mpc_float* Bdata, const mpc_float* fdata,
                   const mpc_float* xe, const mpc_float* ue);
  void SetCost(const mpc_float* Qdata, const mpc_float* Rdata, const mpc_float* Qfdata);
  void SetGoalState(const mpc_float* xf);
  void SetInitialState(const mpc_float* x0);
  void Solve();

  const ErrorVector& GetState(int k) const;
  const InputVector& GetInput(int k) const;

  InputVector ControlPolicy(const StateVector& x, double t);

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