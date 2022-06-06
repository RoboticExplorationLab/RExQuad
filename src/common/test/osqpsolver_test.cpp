#include "common/osqpsolver.hpp"

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "osqp/osqp.h"
#include "common/problem_data.h"
#include "common/workspace.h"
#include "common/control.hpp"

namespace rexquad {
// TEST(OSQPSolverTests, RawSolve) {
//   osqp_solve(&workspace);
//   // Print status
//   printf("Status:                %s\n", (&workspace)->info->status);
//   printf("Number of iterations:  %d\n", (int)((&workspace)->info->iter));
//   printf("Objective value:       %.4e\n", (&workspace)->info->obj_val);
//   printf("Primal residual:       %.4e\n", (&workspace)->info->pri_res);
//   printf("Dual residual:         %.4e\n", (&workspace)->info->dua_res);
//   c_float* x = workspace.solution->x;
//   printf("x0 = [ ");
//   for (int i = 0; i < 12; ++i) {
//     printf("%0.3g ", x[i]);
//   }
//   printf("]\n");

//   int nstates_total = 12 * 11;
//   printf("u0 = [ ");
//   for (int i = 0; i < 4; ++i) {
//     printf("%0.4g ", x[i + nstates_total]);
//   }
//   printf("]\n");
// }

TEST(OSQPSolverTests, Initialization) {
  // Set up a double integrator problem
  int nhorizon = 3;

  // Initialize OSQPSolver
  OSQPSolver solver(nstates, ninputs, nhorizon);
  MPCProblem& prob = solver.GetProblem();
  prob.SetDynamics(dynamics_Adata, dynamics_Bdata, dynamics_fdata);
  prob.SetCostTerminal(cost_Qfdata, cost_qfdata);
  prob.SetCostState(cost_Qdata, cost_qdata);
  prob.SetCostInput(cost_Rdata, cost_rdata);
  prob.SetCostConstant(0.0);

  solver.Initialize(&workspace);
  c_float x0[12];
  solver.GetState(x0, 0);
}

TEST(OSQPSolverTests, Solve) {
  OSQPSolver solver(nstates, ninputs, nhorizon);
  MPCProblem& prob = solver.GetProblem();
  prob.SetDynamics(dynamics_Adata, dynamics_Bdata, dynamics_fdata);
  prob.SetCostTerminal(cost_Qfdata, cost_qfdata);
  prob.SetCostState(cost_Qdata, cost_qdata);
  prob.SetCostInput(cost_Rdata, cost_rdata);
  prob.SetCostConstant(0.0);
  solver.Initialize(&workspace);

  // Set initial state
  ErrorVector dx0 = ErrorVector::Zero();
  solver.SetInitialState(dx0.data());

  // Solve
  solver.Solve();
  ErrorVector x0;
  InputVector u0;
  solver.GetState(x0.data(), 0);
  solver.GetInput(u0.data(), 0);
  InputVector u_expected = InputVector::Zero();;
  // u_expected << 12.02562626, 12.02562626, 12.02562626, 12.02562626;
  // u_expected << 55.43705474, 55.43705474, 55.43705474, 55.43705474;
  EXPECT_LT(x0.norm(), 1e-5);
  EXPECT_LT((u0-u_expected).norm(), 1e-5);
  EXPECT_STREQ(workspace.info->status, "solved");

  // Solve again
  solver.Solve();
  solver.GetInput(u0.data(), 0);
  EXPECT_LT((u0-u_expected).norm(), 1e-5);
}

TEST(OSQPSolverTests, ChangeInitialState) {
  OSQPSolver solver(nstates, ninputs, nhorizon);
  MPCProblem& prob = solver.GetProblem();
  prob.SetDynamics(dynamics_Adata, dynamics_Bdata, dynamics_fdata);
  prob.SetCostTerminal(cost_Qfdata, cost_qfdata);
  prob.SetCostState(cost_Qdata, cost_qdata);
  prob.SetCostInput(cost_Rdata, cost_rdata);
  prob.SetCostConstant(0.0);
  solver.Initialize(&workspace);

  // Vectors
  int nvars = solver.NumPrimals();
  int ncons = solver.NumDuals();
  ErrorVector dxe;
  InputVector due;
  ErrorVector dxg;
  Eigen::VectorXd l = Eigen::VectorXd::Zero(ncons);
  Eigen::VectorXd u = Eigen::VectorXd::Zero(ncons);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(nvars);
  Eigen::Map<Eigen::Vector<c_float, Eigen::Dynamic>> xsol(solver.GetSolution(), nvars);

  // Set initial state
  ErrorVector dx0 = ErrorVector::Zero();
  solver.SetInitialState(dx0.data());

  xsol.setZero();
  solver.Solve();
  InputVector u0;
  solver.GetInput(u0.data(), 0);
  EXPECT_LT(u0.norm(), 1e-6);

  prob.GetGoalState(dx0.data());
  prob.GetEquilibriumPoint(dxe.data(), due.data());
  prob.GetGoalState(dxg.data());
  solver.GetBounds(l.data(), u.data());
  solver.GetLinCost(q.data());
  fmt::print("First Solve\n");
  fmt::print("  xe = [{}]\n", dxe.transpose());
  fmt::print("  ue = [{}]\n", due.transpose());
  fmt::print("  xg = [{}]\n", dxg.transpose());
  fmt::print("  x0 = [{}]\n", dx0.transpose());
  fmt::print("  u0 = [{}]\n", u0.transpose());
  fmt::print("  ||q|| = {}\n", q.norm());
  fmt::print("  ||l|| = {}\n", l.norm());
  fmt::print("  ||u|| = {}\n", u.norm());
  fmt::print("  ||z|| = {}\n", xsol.norm());

  printf("Status:                %s\n", (&workspace)->info->status);
  printf("Number of iterations:  %d\n", (int)((&workspace)->info->iter));
  printf("Objective value:       %.4e\n", (&workspace)->info->obj_val);
  printf("Primal residual:       %.4e\n", (&workspace)->info->pri_res);
  printf("Dual residual:         %.4e\n", (&workspace)->info->dua_res);

  // Set initial state to lower
  dx0[2] = +0.5;
  solver.SetInitialState(dx0.data());
  xsol.setZero();
  solver.Solve();
  solver.GetInput(u0.data(), 0);
  fmt::print("u0 = [{}]\n", u0.transpose());
  // EXPECT_GT(u0.maxCoeff(), 10);

  // // Set initial state to higher
  // dx0[2] = +0.5;
  // solver.SetInitialState(dx0.data());
  // solver.Solve();
  // solver.GetInput(u0.data(), 0);
  // // fmt::print("u0 = [{}]\n", u0.transpose());
  // EXPECT_LT(u0.maxCoeff(), -10);

  // Set initial state back
  dx0[2] = 0.0;
  solver.SetInitialState(dx0.data());
  xsol.setZero();
  solver.Solve();
  solver.GetInput(u0.data(), 0);

  prob.GetGoalState(dx0.data());
  prob.GetEquilibriumPoint(dxe.data(), due.data());
  prob.GetGoalState(dxg.data());
  solver.GetBounds(l.data(), u.data());
  solver.GetLinCost(q.data());
  fmt::print("Second Solve\n");
  fmt::print("  xe = [{}]\n", dxe.transpose());
  fmt::print("  ue = [{}]\n", due.transpose());
  fmt::print("  xg = [{}]\n", dxg.transpose());
  fmt::print("  x0 = [{}]\n", dx0.transpose());
  fmt::print("  u0 = [{}]\n", u0.transpose());
  fmt::print("  ||q|| = {}\n", q.norm());
  fmt::print("  ||l|| = {}\n", l.norm());
  fmt::print("  ||u|| = {}\n", u.norm());
  fmt::print("  ||z|| = {}\n", xsol.norm());

  printf("Status:                %s\n", (&workspace)->info->status);
  printf("Number of iterations:  %d\n", (int)((&workspace)->info->iter));
  printf("Objective value:       %.4e\n", (&workspace)->info->obj_val);
  printf("Primal residual:       %.4e\n", (&workspace)->info->pri_res);
  printf("Dual residual:         %.4e\n", (&workspace)->info->dua_res);
}


}
