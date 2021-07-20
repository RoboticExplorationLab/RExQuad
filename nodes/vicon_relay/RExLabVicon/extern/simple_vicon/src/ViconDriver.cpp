#include <simple_vicon/ViconDriver.h>

#include <iostream>

bool ViconDriver::init(const vicon_driver_params_t& params, callback_type callback) {
  callback_ = callback;

  client_.Connect(params.server_ip);
  if (!client_.IsConnected().Connected) {
    std::cerr << "Failed to connect to Vicon server at " << params.server_ip << std::endl;
    return false;
  }

  client_.SetStreamMode(params.stream_mode);
  client_.SetAxisMapping(ViconSDK::Direction::Forward,
                         ViconSDK::Direction::Left,
                         ViconSDK::Direction::Up);

  return true;
}

void ViconDriver::run_loop() {
  client_.EnableSegmentData();
  int wait_for_buffer_counter = 0;
  bool ready_for_init = false;
  running_.store(true);
  while (running_.load()) {
    auto result = client_.GetFrame().Result;

    if (result != ViconSDK::Result::Success) {
      std::cerr << "ViconSDK GetFrame did not return success: " << result << std::endl;
      continue;
    }

    double framerate = client_.GetFrameRate().FrameRateHz;
    // Wait for flushing the buffer in ServerPush mode
    if (!ready_for_init)
    {
      if (++wait_for_buffer_counter > framerate)
      {
        ready_for_init = true;
      }
      else
      {
        continue;
      }
    }
    vicon_result_t res;
    // We assume that the client and the server clocks are time-synced using NTP
    // Proposed strategy: subtract latency from timecode
    ViconSDK::Output_GetTimecode timecode = client_.GetTimecode();
    if (timecode.Result != ViconSDK::Result::Success) {
      std::cerr << "ViconSDK GetTimeCode did not return success: " << result << std::endl;
      continue;
    }

    // Convert Timecode to seconds
    double timecode_time = timecode.Hours * 3600.0 + timecode.Minutes * 60.0 + timecode.Seconds + (timecode.Frames * timecode.SubFramesPerFrame + timecode.SubFrame) / framerate;

    // Is this the first time we're seeing data? If so, initialise timecode_offset_
    // TODO: Determine average round trip network delay in a separate slower thread and save in network_lag_estimate_?
    // TODO: Also add jump logic here since timecode rotates every 24 hours...
    if (timecode_offset_ == 0.0)
    {
      auto time_now_sys = std::chrono::high_resolution_clock::now().time_since_epoch();
      double time_now = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>>(time_now_sys).count();
      timecode_offset_ = time_now - network_lag_estimate_ - timecode_time;
      printf("Timecode offset :: %.8f Time:: %.8f Timecode %.8f lag :: %f\n", timecode_offset_, time_now, timecode_time, network_lag_estimate_);
    }
    // Move the timecode to the client (our) clock, and subtract the vicon system estimated latency from it.
    res.time = timecode_time + timecode_offset_ - client_.GetLatencyTotal().Total;
    int n_subjects = client_.GetSubjectCount().SubjectCount;
    for (int i = 0; i < n_subjects; i++) {
      std::string subject_name = client_.GetSubjectName(i).SubjectName;
      std::string segment_name = client_.GetSegmentName(subject_name, 0).SegmentName;
      auto trans = client_.GetSegmentGlobalTranslation(subject_name, segment_name);
      auto rot = client_.GetSegmentGlobalRotationQuaternion(subject_name, segment_name);

      if (trans.Result != ViconSDK::Result::Success) {
        std::cerr << "Translation get failed for " << subject_name << "::" << segment_name << std::endl;
      }
      if (rot.Result != ViconSDK::Result::Success) {
        std::cerr << "Rotation get failed for " << subject_name << "::" << segment_name << std::endl;
      }

      res.data.emplace_back(
          subject_name,
          trans.Occluded or rot.Occluded,
          // Translations are in mm
          std::array<double, 3>{
              trans.Translation[0] * 0.001,
              trans.Translation[1] * 0.001,
              trans.Translation[2] * 0.001},
          // Quaternion is returned in W last format; we store it W first.
          std::array<double, 4>{
              rot.Rotation[3],
              rot.Rotation[0],
              rot.Rotation[1],
              rot.Rotation[2]});
    }

    if (res.data.size()) {
      callback_(res);
    }
  }
}

void ViconDriver::stop() {
  running_.store(false);
}
