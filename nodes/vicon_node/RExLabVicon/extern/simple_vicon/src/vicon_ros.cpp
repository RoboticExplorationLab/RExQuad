#include <simple_vicon/ViconDriverROS.h>

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "vicon_ros");
  ros::NodeHandle n("~");

  ViconDriverROS vicon;

  if (!vicon.initialize(n)) {
    ROS_ERROR("Failed to initialize ViconDriverROS");
    return 1;
  }

  ros::spin();

  vicon.finalize();
}
