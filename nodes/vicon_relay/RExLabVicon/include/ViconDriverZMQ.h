#include <simple_vicon/ViconDriver.h>

#include <stdio.h>
#include <thread>
#include <utility>
#include <vector>
#include <zmq.hpp>
#include "vicon_msg.pb.h"
#include <unistd.h>
#include <string.h>


class ViconDriverZMQ {
	public:
        bool initialize(char *sub_ip, char *rigid_body, char *pub_ip, char *pub_port);
        void finalize();
        void vicon_callback(vicon_result_t res);

        zmq::context_t context_;
        zmq::socket_t socket_; 

    private: 
        void viconCallback(vicon_result_t vicon_result);
        ViconDriver driver_;
        std::thread vicon_thread_;
        messaging::VICON vicon_;

        std::string zmq_socket_name_;
        std::string vicon_server_name_;
        std::string rigid_body_name_;

        std::string msg_str_;
        zmq::message_t msg_;


};