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

}  // namespace rexquad