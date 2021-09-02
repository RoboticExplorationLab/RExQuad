#include <zmq.hpp>
#include <stdio.h>
#include "vicon_msg.pb.h"
#include <vector>
#include <unistd.h>
#include <thread>
#include <csignal>
#include <simple_vicon/ViconDriver.h>
#include <ViconDriverZMQ.h>
using namespace std;

ViconDriverZMQ viconZMQ;

void signal_handler(int signal) {
	viconZMQ.finalize();
}

int main(int argc, char *argv[]) {

	if (argc != 5) {
		std::cerr << "You need to supply four arguments to this program!\n"
					 "(rigid body name, publisher ip, subscriber ip, subscriber port #)"
				  << std::endl;
		return -1;
	}

	char *sub_ip = argv[1];
	char *rigid_body = argv[2];
	char *pub_ip = argv[3];
	char *pub_port = argv[4];
	// char *pub_port = argv[4];

	if (!viconZMQ.initialize(sub_ip, rigid_body, pub_ip, pub_port))
	{
		std::cerr << "Failed to init driver" << std::endl;
	}
	std::signal(SIGINT, signal_handler);
	std::signal(SIGTERM, signal_handler);
	std::signal(SIGHUP, signal_handler);
}
