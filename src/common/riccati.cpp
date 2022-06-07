#include "riccati.hpp"

namespace rexquad {

RiccatiSolver::RiccatiSolver(int nhorizon)
    : nhorizon_(nhorizon),
      Q_(),
      q_(ErrorVector::Zero()),
      R_(),
      r_(InputVector::Zero()),
      Qf_(),
      qf_(ErrorVector::Zero()),
      A_(StateMatrix::Zero()),
      B_(InputMatrix::Zero()),
      f_(ErrorVector::Zero()),
      xe_(StateVector::Zero()),
      ue_(InputVector::Zero()),
      x0_(StateVector::Zero()),
      xf_(StateVector::Zero()),
      dx0_(ErrorVector::Zero()),
      dxf_(ErrorVector::Zero()) {
  Q_.setIdentity();
  R_.setIdentity();
  Qf_.setIdentity();
  xe_[3] = 1.0;
  x0_[3] = 1.0;
  xf_[3] = 1.0;
  for (int i = 0; i < nhorizon; ++i) {
    P_.emplace_back(StateMatrix::Zero());
    p_.emplace_back(ErrorVector::Zero());
    X_.emplace_back(ErrorVector::Zero());
    Y_.emplace_back(ErrorVector::Zero());
    if (i < nhorizon - 1) {
      K_.emplace_back(FeedbackGain::Zero());
      d_.emplace_back(InputVector::Zero());
      U_.emplace_back(InputVector::Zero());
    }
  }
}

void RiccatiSolver::SetDynamics(const StateMatrix& A, const InputMatrix& B,
                                const ErrorVector& f) {
  A_ = A;
  B_ = B;
  f_ = f;
}

void RiccatiSolver::SetCost(const StatePenalty& Q, const InputPenalty& R,
                            const StatePenalty& Qf) {
  Q_ = Q;
  R_ = R;
  Qf_ = Qf;
  this->SetGoalState(xf_);
}

void RiccatiSolver::SetGoalState(const StateVector xf) {
  xf_ = xf;
  ErrorState(dxf_, xf_, xe_);
  q_ = -(Q_ * dxf_);
  qf_ = -(Qf_ * dxf_);
}

void RiccatiSolver::SetInitialState(const StateVector x0) {
  x0_ = x0;
  ErrorState(dx0_, x0_, xe_);
}

void RiccatiSolver::Solve() {
  this->BackwardPass();
  this->ForwardPass();
}

void RiccatiSolver::BackwardPass() {
  int N = nhorizon_-1;
  P_[N] = Qf_;
  p_[N] = qf_;
  ErrorVector Qx;
  InputVector Qu;
  StateMatrix Qxx;
  InputMatrixSquare Quu;
  Eigen::Matrix<mpc_float, kNumInputs, kNumErrStates> Qux;
  for (int k = N-1; k >= 0; --k) {
    const StateMatrix& Pn = P_[k+1];
    const ErrorVector& pn = p_[k+1];

    // Action-value expansion
    Qx = q_ + A_.transpose() * pn;
    Qu = r_ + B_.transpose() * pn;
    Qxx = Q_;
    Qxx += A_.transpose() * Pn * A_;
    Quu = R_;
    Quu += B_.transpose() * Pn * B_;
    Qux = B_.transpose() * Pn * A_;

    // Compute Feedback gains
    Eigen::LLT<InputMatrixSquare> cholQuu(Quu);
    K_[k] = cholQuu.solve(Qux);
    d_[k] = cholQuu.solve(Qu);

    // Compute next Cost-to-go expansion
    P_[k] = Qxx;
    P_[k] += K_[k].transpose() * Quu * K_[k];
    P_[k] += K_[k].transpose() * Qux;
    P_[k] += Qux.transpose() * K_[k];

    p_[k] = Qx;
    p_[k] += K_[k].transpose() * Quu * d_[k];
    p_[k] += K_[k].transpose() * Qu;
    p_[k] += Qux.transpose() * d_[k];
  }
}

}  // namespace rexquad