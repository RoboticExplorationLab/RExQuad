#include <gtest/gtest.h>
#include <inttypes.h>

#include "common/pose.hpp"
#include "fmt/core.h"


namespace rexquad {

TEST(PoseTests, Serialization) {
    fmt::print("Size of PoseMsg: {}\n", sizeof(PoseMsg));
    PoseMsg pose = {1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0};
    char buf[sizeof(PoseMsg)];
    PoseToBytes(buf, pose);
    PoseMsg pose2;
    bool is_msg_id_correct = PoseFromBytes(pose2, buf);
    fmt::print("Msg ID: {}\n", static_cast<int8_t>(buf[0]));
    EXPECT_FLOAT_EQ(pose.x, pose2.x);
    EXPECT_FLOAT_EQ(pose.y, pose2.y);
    EXPECT_FLOAT_EQ(pose.z, pose2.z);
    EXPECT_FLOAT_EQ(pose.qw, pose2.qw);
    EXPECT_FLOAT_EQ(pose.qx, pose2.qx);
    EXPECT_FLOAT_EQ(pose.qy, pose2.qy);
    EXPECT_FLOAT_EQ(pose.qz, pose2.qz);
    EXPECT_TRUE(is_msg_id_correct);
}

}