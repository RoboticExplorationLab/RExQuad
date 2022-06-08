#include "common/riccati.hpp"

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "common/problem_data.h"

namespace rexquad {

TEST(RiccatiSolver, Initialization) {
  int nhorizon = 11;
  RiccatiSolver solver(nhorizon);
  solver.SetDynamics(dynamics_Adata, dynamics_Bdata, dynamics_fdata, dynamics_xe,
                     dynamics_ue);
  solver.SetCost(cost_Qdata, cost_Rdata, cost_Qfdata);
  solver.SetGoalState(dynamics_xg);
  StateVector x0;
  x0 << 0, 0, 0.5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  solver.SetInitialState(x0.data());
}

TEST(RiccatiSolver, DefaultSolve) {
  int nhorizon = 11;
  RiccatiSolver solver(nhorizon);
  solver.SetDynamics(dynamics_Adata, dynamics_Bdata, dynamics_fdata, dynamics_xe,
                     dynamics_ue);
  solver.SetCost(cost_Qdata, cost_Rdata, cost_Qfdata);
  solver.SetGoalState(dynamics_xg);
  StateVector x0;
  x0 << 0, 0, 1.0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  solver.SetInitialState(x0.data());

  solver.Solve();
  const InputVector& u0 = solver.GetInput(0);
  EXPECT_LT(u0.norm(), 1e-6);
}

TEST(RiccatiSolver, ChangeInitialState) {
  int nhorizon = 11;
  RiccatiSolver solver(nhorizon);
  solver.SetDynamics(dynamics_Adata, dynamics_Bdata, dynamics_fdata, dynamics_xe,
                     dynamics_ue);
  solver.SetCost(cost_Qdata, cost_Rdata, cost_Qfdata);
  solver.SetGoalState(dynamics_xg);
  StateVector x0;
  InputVector u0;
  x0 << 0, 0, 1.0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  // Start at goal
  x0[2] = 1.0;
  solver.SetInitialState(x0.data());
  solver.Solve();
  u0 = solver.GetInput(0);
  EXPECT_LT(u0.norm(), 1e-6);

  // Change to lower than goal
  x0[2] = 0.5;
  solver.SetInitialState(x0.data());
  solver.Solve();
  u0 = solver.GetInput(0);
  EXPECT_GT(u0.minCoeff(), 10);

  // Change to higher than goal
  x0[2] = 1.5;
  solver.SetInitialState(x0.data());
  solver.Solve();
  u0 = solver.GetInput(0);
  EXPECT_LT(u0.maxCoeff(), -10);

  // Change back to initial state 
  x0[2] = 1.0;
  solver.SetInitialState(x0.data());
  solver.Solve();
  u0 = solver.GetInput(0);
  EXPECT_LT(u0.norm(), 1e-6);
}

TEST(RiccatiSolver, ChangeGoalState) {
  int nhorizon = 11;
  RiccatiSolver solver(nhorizon);
  solver.SetDynamics(dynamics_Adata, dynamics_Bdata, dynamics_fdata, dynamics_xe,
                     dynamics_ue);
  solver.SetCost(cost_Qdata, cost_Rdata, cost_Qfdata);
  StateVector x0;
  StateVector xg;
  InputVector u0;
  x0 << 0, 0, 1.0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  xg << 0, 0, 1.0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  solver.SetInitialState(x0.data());
  solver.SetGoalState(xg.data());

  // Get default solve
  solver.Solve();
  u0 = solver.GetInput(0);
  EXPECT_LT(u0.norm(), 1e-6);

  // Change goal to be higher
  xg[2] = 1.5;
  solver.SetGoalState(xg.data());
  solver.Solve();
  u0 = solver.GetInput(0);
  EXPECT_GT(u0.minCoeff(), 10);
  double u_higher = u0(0);

  // Change goal to be lower 
  xg[2] = 0.5;
  solver.SetGoalState(xg.data());
  solver.Solve();
  u0 = solver.GetInput(0);
  EXPECT_LT(u0.maxCoeff(), -10);
  double u_lower = u0(0);

  EXPECT_FLOAT_EQ(u_higher, -u_lower);
}


TEST(RiccatiSolver, ControlPolicy) {
  int nhorizon = 11;
  RiccatiSolver solver(nhorizon);
  solver.SetDynamics(dynamics_Adata, dynamics_Bdata, dynamics_fdata, dynamics_xe,
                     dynamics_ue);
  solver.SetCost(cost_Qdata, cost_Rdata, cost_Qfdata);
  StateVector x0;
  StateVector xg;
  InputVector u0;
  x0 << 0, 0, 1.0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  xg << 0, 0, 1.0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  solver.SetInitialState(x0.data());
  solver.SetGoalState(xg.data());

  double t = 0.0;
  u0 = solver.ControlPolicy(x0, t);
  InputVector u_trim = InputVector::Constant(kHoverInput);
  EXPECT_LT((u0 - u_trim).norm(), 1e-5);

  x0[2] = 0.5;
  solver.ControlPolicy(x0, t);
  EXPECT_GT((u0 - u_trim).minCoeff(), 10);
}

}  // namespace rexquad
