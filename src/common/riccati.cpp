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

void RiccatiSolver::SetDynamics(const mpc_float* Adata, const mpc_float* Bdata,
                                const mpc_float* fdata, const mpc_float* xe,
                                const mpc_float* ue) {
  for (int i = 0; i < A_.size(); ++i) {
    A_(i) = Adata[i];
  }
  for (int i = 0; i < B_.size(); ++i) {
    B_(i) = Bdata[i];
  }
  for (int i = 0; i < f_.size(); ++i) {
    f_(i) = fdata[i];
  }
  for (int i = 0; i < xe_.size(); ++i) {
    xe_(i) = xe[i];
  }
  for (int i = 0; i < ue_.size(); ++i) {
    ue_(i) = ue[i];
  }
}

void RiccatiSolver::SetCost(const mpc_float* Qdata, const mpc_float* Rdata,
                            const mpc_float* Qfdata) {
  for (int i = 0; i < Q_.rows(); ++i) {
    Q_.diagonal()(i) = Qdata[i];
  }
  for (int i = 0; i < Qf_.rows(); ++i) {
    Qf_.diagonal()(i) = Qfdata[i];
  }
  for (int i = 0; i < R_.rows(); ++i) {
    R_.diagonal()(i) = Rdata[i];
  }
}

void RiccatiSolver::SetGoalState(const mpc_float* xf) {
  for (int i = 0; i < xf_.size(); ++i) {
    xf_(i) = xf[i];
  }
  ErrorState(dxf_, xf_, xe_);
  q_ = -(Q_ * dxf_);
  qf_ = -(Qf_ * dxf_);
}

void RiccatiSolver::SetInitialState(const mpc_float* x0) {
  for (int i = 0; i < x0_.size(); ++i) {
    x0_(i) = x0[i];
  }
  ErrorState(dx0_, x0_, xe_);
}

void RiccatiSolver::Solve() {
  this->BackwardPass();
  this->ForwardPass();
}

void RiccatiSolver::BackwardPass() {
  int N = nhorizon_ - 1;
  P_[N] = Qf_;
  p_[N] = qf_;
  ErrorVector Qx;
  InputVector Qu;
  StateMatrix Qxx;
  InputMatrixSquare Quu;
  Eigen::Matrix<mpc_float, kNumInputs, kNumErrStates> Qux;
  for (int k = N - 1; k >= 0; --k) {
    const StateMatrix& Pn = P_[k + 1];
    const ErrorVector& pn = p_[k + 1];

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
    K_[k] *= -1;
    d_[k] *= -1;

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

void RiccatiSolver::ForwardPass() {
  int N = nhorizon_ - 1;
  X_[0] = dx0_;
  for (int k = 0; k < N - 1; ++k) {
    Y_[k] = P_[k] * X_[k] + p_[k];
    U_[k] = K_[k] * X_[k] + d_[k];
    X_[k + 1] = A_ * X_[k] + B_ * U_[k] + f_;
  }
  Y_[N] = P_[N] * X_[N] + p_[N];
}

const ErrorVector& RiccatiSolver::GetState(int k) const {
  return X_[k];
}

const InputVector& RiccatiSolver::GetInput(int k) const {
  return U_[k];
}

}  // namespace rexquad