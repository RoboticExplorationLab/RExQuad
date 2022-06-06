#pragma once

#include "osqp/qdldl_interface.h"
#include "problem.hpp"

namespace rexquad {
class OSQPSolver {
 public:
  OSQPSolver(int nstates, int ninputs, int nhorizon);
  void Initialize(OSQPWorkspace* p_work);
  bool Solve();
  void GetState(mpc_float* x, int k) const;
  void GetInput(mpc_float* u, int k) const;
  void SetInitialState(const mpc_float* x0);
  void SetReferenceState(const mpc_float* xr);
  MPCProblem& GetProblem();
  c_float* GetSolution() { return p_workspace_->solution->x; }
  bool GetControl(mpc_float* u, const mpc_float* dx, mpc_float t);
  void GetInitialState(mpc_float* x0) const;
  void GetBounds(mpc_float* l, mpc_float* u) const;
  void GetLinCost(mpc_float* q) const;
  int NumPrimals() const;
  int NumDuals() const;

 private:
  void BuildKKTSystem();
  void OSQPWorkspaceSetup(void* mem, int memsize);

  int nstates_;
  int ninputs_;
  int nhorizon_;
  MPCProblem prob_;
  OSQPWorkspace* p_workspace_;
};
}
