#pragma once

namespace rexquad {

class Quaternion {

public:
  Quaternion(float w, float x, float y, float z);
  Quaternion Compose(const Quaternion& q2) const;
  Quaternion Invert() const;
  float w;
  float x;
  float y;
  float z;
};

Quaternion QuaternionComposition(const Quaternion& q1, const Quaternion& q2);

void QuaternionError(float* err, const Quaternion& q1, const Quaternion& q2);

Quaternion CayleyMap(float x, float y, float z);


}  // namespace quad