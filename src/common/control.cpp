#include "control.hpp"
#include "quaternion.hpp"

namespace rexquad {

void ErrorState(ErrorVector& e, const StateVector& x, const StateVector& xeq) {
  // Position error
  for (int i = 0; i < 3; ++i) {
    e[i] = x[i] - xeq[i];
  }

  // Quaternion error
  float phi[3];
  rexquad::Quaternion q(x[3], x[4], x[5], x[6]);
  rexquad::Quaternion qeq(xeq[3], xeq[4], xeq[5], xeq[6]);
  rexquad::QuaternionError(phi, q, qeq);
  for (int i = 0; i < 3; ++i) {
    e[3 + i] = phi[i];
  }

  // Velocity error
  for (int i = 0; i < 6; ++i) {
    e[6 + i] = x[7 + i] - xeq[7 + i];
  }
}

}  // namespace rexquad