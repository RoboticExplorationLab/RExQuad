#include "quaternion.hpp"

#include <cmath>

namespace rexquad {

Quaternion::Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

Quaternion Quaternion::Compose(const Quaternion& q2) const {
  return QuaternionComposition(*this, q2);
}

Quaternion Quaternion::Invert() const { return Quaternion(w, -x, -y, -z); }

Quaternion QuaternionComposition(const Quaternion& q1, const Quaternion& q2) {
  return Quaternion(q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
                    q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
                    q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
                    q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w);
}

void QuaternionError(float* err, const Quaternion& q1, const Quaternion& q2) {
  // Compute the error quaternion
  Quaternion qerr = q2.Invert().Compose(q1);

  // Cayley map
  err[0] = qerr.x / qerr.w;
  err[1] = qerr.y / qerr.w;
  err[2] = qerr.z / qerr.w;
}

Quaternion CayleyMap(float x, float y, float z) {
  float norm = x * x + y * y + z * z;
  float m = 1 / std::sqrt(1 + norm);
  return Quaternion(m, m * x, m * y, m * z);
}

}  // namespace rexquad