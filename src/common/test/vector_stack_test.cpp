extern "C" {
#include "common/vector_stack.h"
#include <slap/slap.h>
}

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

#define VEC_LENGTH 2
char buf[REXQUAD_QUEUE_SIZE * sizeof(double) * VEC_LENGTH];

namespace rexquad {

bool VectorsAreEqual(const double* a, const double* b) {
  bool are_equal = true;
  for (int i = 0; i < VEC_LENGTH; ++i) {
    are_equal &= a[i] == b[i];
  }
  return are_equal;
}

TEST(VectorQueueTest, SizeAfterCreation) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate(buf, VEC_LENGTH);
  EXPECT_EQ(rexquad_VectorQueueSize(&q), 0);
}

TEST(VectorQueueTest, SizeAfterPush) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate(buf, VEC_LENGTH);
  double v0[2] = {1,2};
  rexquad_VectorQueuePush(&q, v0);
  EXPECT_EQ(rexquad_VectorQueueSize(&q), 1);
}

TEST(VectorQueueTest, GetAfterOnePush) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate(buf, VEC_LENGTH);
  double v0[2] = {1,2};
  rexquad_VectorQueuePush(&q, v0);
  const double* v = rexquad_VectorQueueGet(&q, 0);
  EXPECT_TRUE(VectorsAreEqual(v, v0));
}

TEST(VectorQueueTest, PushConst) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate(buf, VEC_LENGTH);
  const double v0[2] = {1,2};
  rexquad_VectorQueuePush(&q, v0);
  EXPECT_EQ(rexquad_VectorQueueSize(&q), 1);
}

TEST(VectorQueueTest, PushThenPop) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate(buf, VEC_LENGTH);
  const double v0[2] = {1,2};
  rexquad_VectorQueuePush(&q, v0);
  rexquad_VectorQueuePop(&q);
  EXPECT_EQ(rexquad_VectorQueueSize(&q), 0);
}

TEST(VectorQueueTest, GetAfterCreate) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate(buf, VEC_LENGTH);
  const double* v = rexquad_VectorQueueGet(&q, 0);
  EXPECT_EQ(v, (double*)NULL);
}

TEST(VectorQueueTest, SizeAfterPushTwice) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate(buf, VEC_LENGTH);
  const double v0[2] = {1,2};
  const double v1[2] = {3,4};
  rexquad_VectorQueuePush(&q, v0);
  rexquad_VectorQueuePush(&q, v1);
  EXPECT_EQ(rexquad_VectorQueueSize(&q), 2);
}

TEST(VectorQueueTest, SizeAfterPushTwiceThenPop) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate(buf, VEC_LENGTH);
  const double v0[2] = {1,2};
  const double v1[2] = {3,4};
  rexquad_VectorQueuePush(&q, v0);
  rexquad_VectorQueuePush(&q, v1);
  rexquad_VectorQueuePop(&q);
  EXPECT_EQ(rexquad_VectorQueueSize(&q), 1);
}

TEST(VectorQueueTest, GetAfterPushTwiceThenPop) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate(buf, VEC_LENGTH);
  const double v0[2] = {1,2};
  const double v1[2] = {3,4};
  rexquad_VectorQueuePush(&q, v0);
  rexquad_VectorQueuePush(&q, v1);
  rexquad_VectorQueuePop(&q);
  const double* v = rexquad_VectorQueueGet(&q, 0);
  EXPECT_TRUE(VectorsAreEqual(v, v1));
}

TEST(VectorQueueTest, GetAfterPushThree) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate(buf, VEC_LENGTH);
  const double v0[2] = {1,2};
  const double v1[2] = {3,4};
  const double v2[2] = {5,6};
  rexquad_VectorQueuePush(&q, v0);
  rexquad_VectorQueuePush(&q, v1);
  rexquad_VectorQueuePush(&q, v2);
  EXPECT_TRUE(VectorsAreEqual(rexquad_VectorQueueGet(&q, 0), v2));
  EXPECT_TRUE(VectorsAreEqual(rexquad_VectorQueueGet(&q, 1), v1));
  EXPECT_TRUE(VectorsAreEqual(rexquad_VectorQueueGet(&q, 2), v0));
}

TEST(VectorQueueTest, GetIndexLongerThanLength) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate(buf, VEC_LENGTH);
  const double v0[2] = {1,2};
  const double v1[2] = {3,4};
  const double v2[2] = {5,6};
  rexquad_VectorQueuePush(&q, v0);
  rexquad_VectorQueuePush(&q, v1);
  rexquad_VectorQueuePush(&q, v2);
  EXPECT_EQ(rexquad_VectorQueueGet(&q, 3), (double*)NULL);
}

TEST(VectorQueueTest, LengthAfterEndWrapsBack) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate(buf, VEC_LENGTH);
  const double v0[2] = {1,2};
  const double v1[2] = {3,4};
  const double v2[2] = {5,6};
  rexquad_VectorQueuePush(&q, v0);
  rexquad_VectorQueuePush(&q, v1);
  rexquad_VectorQueuePush(&q, v2);

  for (int i = 0; i < REXQUAD_QUEUE_SIZE; ++i) {
    const double* v = rexquad_VectorQueueGet(&q, 2);
    rexquad_VectorQueuePop(&q);
    rexquad_VectorQueuePush(&q, v);
    EXPECT_EQ(rexquad_VectorQueueSize(&q), 3);
  }
}

}
