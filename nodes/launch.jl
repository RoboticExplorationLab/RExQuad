using TOML

setup_dict = TOML.tryparsefile("$(@__DIR__)/setup.toml")

# %%
# Launch IMU publisher

include("$(@__DIR__)/imu/imu_publisher.jl")
using .ImuPublisher

zmq_imu_ip = setup_dict["zmq"]["server"]
zmq_imu_port = setup_dict["zmq"]["imu_port"]
imu_serial_port = setup_dict["imu_arduino"]["serial_port"]
imu_baud_rate = setup_dict["imu_arduino"]["baud_rate"]

imu_pub() = imu_publisher(zmq_imu_ip, zmq_imu_port, 
                          imu_serial_port, imu_baud_rate; 
                          debug=true)
imu_thread = Task(imu_pub)
schedule(imu_thread)

# %%
# Launch Vicon publisher
cd("$(@__DIR__)/vicon/build")
vicon_ip = setup_dict["vicon"]["server"]
vicon_rgb_name = setup_dict["vicon"]["subject"]
zmqs_vicon_ip = setup_dict["zmq"]["server"]
zmqs_vicon_port = setup_dict["zmq"]["vicon_port"]
@async run(`./example $vicon_ip $vicon_rgb_name $zmqs_vicon_ip $zmqs_vicon_port`)

# Wait then launch the logger
sleep(2)
include("$(@__DIR__)/logging/imu_vicon_logger.jl")
using .ImuViconLogger

zmqs_vicon_ip = setup_dict["zmq"]["server"]
zmqs_vicon_port = setup_dict["zmq"]["vicon_port"]
zmqs_imu_ip = setup_dict["zmq"]["server"]
zmqs_imu_port = setup_dict["zmq"]["imu_port"]
imu_vicon_logger(zmqs_vicon_ip, zmqs_vicon_port, 
                 zmqs_imu_ip, zmqs_imu_port; debug=false)

while(true)
    sleep(.00001)
end