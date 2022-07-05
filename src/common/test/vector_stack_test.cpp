extern "C" {
#include "common/vector_stack.h"
#include <slap/slap.h>
}

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

namespace rexquad {

TEST(VectorQueueTest, SizeAfterCreation) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate();
  EXPECT_EQ(rexquad_VectorQueueSize(&q), 0);
}

TEST(VectorQueueTest, SizeAfterPush) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate();
  double v0[2] = {1,2};
  rexquad_VectorQueuePush(&q, v0);
  EXPECT_EQ(rexquad_VectorQueueSize(&q), 1);
}

TEST(VectorQueueTest, GetAfterOnePush) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate();
  double v0[2] = {1,2};
  rexquad_VectorQueuePush(&q, v0);
  const double* v = rexquad_VectorQueueGet(&q, 0);
  EXPECT_EQ(v, v0);
}

TEST(VectorQueueTest, PushConst) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate();
  const double v0[2] = {1,2};
  rexquad_VectorQueuePush(&q, v0);
  EXPECT_EQ(rexquad_VectorQueueSize(&q), 1);
}

TEST(VectorQueueTest, PushThenPop) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate();
  const double v0[2] = {1,2};
  rexquad_VectorQueuePush(&q, v0);
  rexquad_VectorQueuePop(&q);
  EXPECT_EQ(rexquad_VectorQueueSize(&q), 0);
}

TEST(VectorQueueTest, GetAfterCreate) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate();
  const double* v = rexquad_VectorQueueGet(&q, 0);
  EXPECT_EQ(v, (double*)NULL);
}

TEST(VectorQueueTest, SizeAfterPushTwice) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate();
  const double v0[2] = {1,2};
  const double v1[2] = {3,4};
  rexquad_VectorQueuePush(&q, v0);
  rexquad_VectorQueuePush(&q, v1);
  EXPECT_EQ(rexquad_VectorQueueSize(&q), 2);
}

TEST(VectorQueueTest, SizeAfterPushTwiceThenPop) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate();
  const double v0[2] = {1,2};
  const double v1[2] = {3,4};
  rexquad_VectorQueuePush(&q, v0);
  rexquad_VectorQueuePush(&q, v1);
  rexquad_VectorQueuePop(&q);
  EXPECT_EQ(rexquad_VectorQueueSize(&q), 1);
}

TEST(VectorQueueTest, GetAfterPushTwiceThenPop) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate();
  const double v0[2] = {1,2};
  const double v1[2] = {3,4};
  rexquad_VectorQueuePush(&q, v0);
  rexquad_VectorQueuePush(&q, v1);
  rexquad_VectorQueuePop(&q);
  const double* v = rexquad_VectorQueueGet(&q, 0);
  EXPECT_EQ(v, v1);
}

TEST(VectorQueueTest, GetAfterPushThree) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate();
  const double v0[2] = {1,2};
  const double v1[2] = {3,4};
  const double v2[2] = {5,6};
  rexquad_VectorQueuePush(&q, v0);
  rexquad_VectorQueuePush(&q, v1);
  rexquad_VectorQueuePush(&q, v2);
  EXPECT_EQ(rexquad_VectorQueueGet(&q, 0), v2);
  EXPECT_EQ(rexquad_VectorQueueGet(&q, 1), v1);
  EXPECT_EQ(rexquad_VectorQueueGet(&q, 2), v0);
}

TEST(VectorQueueTest, GetIndexLongerThanLength) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate();
  const double v0[2] = {1,2};
  const double v1[2] = {3,4};
  const double v2[2] = {5,6};
  rexquad_VectorQueuePush(&q, v0);
  rexquad_VectorQueuePush(&q, v1);
  rexquad_VectorQueuePush(&q, v2);
  EXPECT_EQ(rexquad_VectorQueueGet(&q, 3), (double*)NULL);
}

TEST(VectorQueueTest, LengthAfterEndWrapsBack) {
  rexquad_VectorQueue q = rexquad_VectorQueueCreate();
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
