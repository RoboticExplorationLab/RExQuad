#include <simple_vicon/ViconDriverROS.h>

#include <geometry_msgs/PoseStamped.h>
#include <simple_vicon/Subject.h>

bool ViconDriverROS::initialize(const ros::NodeHandle& n) {
  nh_ = n;

  vicon_driver_params_t params;
  if (!ros::param::get("vicon/server_ip", params.server_ip)) {
    ROS_ERROR("[ViconDriverROS] Could not get parameter vicon/server_ip");
    return false;
  }

  std::string stream_mode;
  ros::param::param<std::string>("vicon/stream_mode", stream_mode, "ServerPush");
  ros::param::param("vicon/publish_subject", publish_subject_, false);
  ros::param::param("vicon/publish_pose", publish_pose_, true);
  ros::param::param("vicon/publish_tf", publish_tf_, true);

  if (!publish_subject_ && !publish_pose_ && !publish_tf_) {
    ROS_WARN("No publishers enabled... exiting.");
    return false;
  }

  if (stream_mode == "ClientPull") params.stream_mode = ViconSDK::StreamMode::ClientPull;
  else if (stream_mode == "ClientPullPreFetch") params.stream_mode = ViconSDK::StreamMode::ClientPullPreFetch;
  else if (stream_mode == "ServerPush") params.stream_mode = ViconSDK::StreamMode::ServerPush;
  else {
    ROS_ERROR("[ViconDriverROS] Invalid Vicon stream mode %s. Choose from ClientPull, ClientPullPreFetch, or ServerPush", stream_mode.c_str());
    return false;
  }

  if (!driver_.init(params, std::bind(&ViconDriverROS::viconCallback, this, std::placeholders::_1))) {
    return false;
  }

  vicon_thread_ = std::thread(&ViconDriver::run_loop, &driver_);

  return true;
}

void ViconDriverROS::finalize() {
  driver_.stop();
  vicon_thread_.join();
}

void ViconDriverROS::viconCallback(vicon_result_t vicon_result) {
  for (const auto& vicon_pose : vicon_result.data) {
    auto it = std::find_if(pubs_.begin(), pubs_.end(), [&vicon_pose] (const auto &p) { return p.first == vicon_pose.subject; });

    vicon_publishers_t *pubs;

    if (it == pubs_.end()) {
      ROS_INFO("[ViconDriverROS] Adding new publisher for %s", vicon_pose.subject.c_str());

      vicon_publishers_t new_pubs;
      if (publish_subject_) {
        new_pubs.subject_pub = nh_.advertise<simple_vicon::Subject>(vicon_pose.subject + "_subject", 1);
      }

      if (publish_pose_) {
        new_pubs.pose_pub = nh_.advertise<geometry_msgs::PoseStamped>(vicon_pose.subject, 1);
      }

      pubs_.emplace_back(vicon_pose.subject, new_pubs);
      pubs = &pubs_.back().second;
    }
    else {
      pubs = &it->second;
    }

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time(vicon_result.time);
    pose.header.frame_id = vicon_pose.subject;

    pose.pose.position.x = vicon_pose.pos[0];
    pose.pose.position.y = vicon_pose.pos[1];
    pose.pose.position.z = vicon_pose.pos[2];
    pose.pose.orientation.w = vicon_pose.quat[0];
    pose.pose.orientation.x = vicon_pose.quat[1];
    pose.pose.orientation.y = vicon_pose.quat[2];
    pose.pose.orientation.z = vicon_pose.quat[3];

    if (publish_subject_) {
      simple_vicon::Subject sub;
      sub.header = pose.header;
      sub.pose = pose.pose;
      sub.occluded = vicon_pose.occluded;

      pubs->subject_pub.publish(sub);
    }
    if (publish_pose_ && !vicon_pose.occluded) {
      pubs->pose_pub.publish(pose);
    }
    if (publish_tf_ && !vicon_pose.occluded) {
      tf::Transform transform;
      transform.setOrigin(
          tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
      transform.setRotation(
          tf::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                         pose.pose.orientation.z, pose.pose.orientation.w));
      br_.sendTransform(tf::StampedTransform(transform, pose.header.stamp, "world", vicon_pose.subject));
    }
  }
}
