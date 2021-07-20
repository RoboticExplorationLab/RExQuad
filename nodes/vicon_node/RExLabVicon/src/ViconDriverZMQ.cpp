#include <ViconDriverZMQ.h>

bool ViconDriverZMQ::initialize(char *sub_ip, char *rigid_body, char *pub_ip, char *pub_port) {

	vicon_server_name_ = std::string(sub_ip);
	rigid_body_name_ = std::string(rigid_body);
	zmq_socket_name_ = std::string("tcp://") + std::string(pub_ip) + std::string(":") + std::string(pub_port);
	// std::cout << "Vicon Server " << vicon_server_name_ << "\t" << rigid_body_name_ << std::endl;
	// std::cout << "ZMQ Socket " << zmq_socket_name_ << std::endl;

	// Setup ZMQ channel
	context_ = zmq::context_t(1);
	socket_ = zmq::socket_t(context_, ZMQ_PUB);
	socket_.bind(zmq_socket_name_);

	// Setup VICON
	vicon_ = messaging::VICON();

	vicon_.SerializeToString(&msg_str_);
	msg_ = zmq::message_t(msg_str_.size());
	socket_.send(msg_, zmq::send_flags::none);
	msg_.rebuild(msg_str_.size());

	vicon_driver_params_t params;
	params.server_ip = vicon_server_name_;
	params.stream_mode = ViconSDK::StreamMode::ServerPush;
	if (!driver_.init(params, std::bind(&ViconDriverZMQ::vicon_callback, this, std::placeholders::_1))) {
		printf("failed");
		return false;
	}

	vicon_thread_ = std::thread(&ViconDriver::run_loop, &driver_);
	vicon_thread_.join();
	return true;
}

void ViconDriverZMQ::finalize() {
	driver_.stop();
	vicon_thread_.join();
	socket_.close();
}

void ViconDriverZMQ::vicon_callback(vicon_result_t res) {
	for (const auto& vicon_pose : res.data) {
		if (! (vicon_pose.subject).compare(rigid_body_name_)) {
			vicon_.set_pos_x(vicon_pose.pos[0]);
			vicon_.set_pos_y(vicon_pose.pos[1]);
			vicon_.set_pos_z(vicon_pose.pos[2]);

			vicon_.set_quat_w(vicon_pose.quat[0]);
			vicon_.set_quat_x(vicon_pose.quat[1]);
			vicon_.set_quat_y(vicon_pose.quat[2]);
			vicon_.set_quat_z(vicon_pose.quat[3]);
		}
	}
	vicon_.set_time(res.time);
	vicon_.SerializeToString(&msg_str_);

	int sz = msg_str_.length();

	msg_.rebuild(sz);

	memcpy(msg_.data(), msg_str_.c_str(), sz);

	socket_.send(msg_, zmq::send_flags::none);
}
