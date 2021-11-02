# This node is run of the Jetson, acts as the ZMQ publisher for the IMU and Vicon
# data coming through the telemetry radio and the Arduino.
module SerialRelayStart
import Mercury as Hg
using TOML

# Launch IMU publisher
function main(; rate::Float64 = 150.0, debug::Bool = false)
    setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

    imu_serial_port = setup_dict["serial"]["jetson"]["imu_arduino"]["serial_port"]
    imu_baud_rate = setup_dict["serial"]["jetson"]["imu_arduino"]["baud_rate"]

    imu_serial_ipaddr = setup_dict["zmq"]["jetson"]["imu_vicon_relay"]["in"]["server"]
    imu_serial_port = setup_dict["zmq"]["jetson"]["imu_vicon_relay"]["in"]["port"]
    sub_endpoint = tcpstring(imu_serial_ipaddr, imu_serial_port)
    imu_serial_ipaddr = setup_dict["zmq"]["jetson"]["imu_vicon_relay"]["out"]["server"]
    imu_serial_port = setup_dict["zmq"]["jetson"]["imu_vicon_relay"]["out"]["port"]
    pub_endpoint = tcpstring(imu_serial_ipaddr, imu_serial_port)

    # tcpstring(imu_serial_ipaddr, imu_serial_port)

    # imu_serial_port = "/dev/tty.usbmodem14201"
    # imu_baud_rate = 115200

    # imu_ip = setup_dict["zmq"]["jetson"]["imu"]["server"]
    # imu_port = setup_dict["zmq"]["jetson"]["imu"]["port"]
    # imu_port = "5566"

    # vicon_ip = setup_dict["zmq"]["jetson"]["vicon"]["server"]
    # vicon_port = setup_dict["zmq"]["jetson"]["vicon"]["port"]
    # vicon_port = "5544"

    imu_vicon_serial_relay =
        Hg.launch_relay(port_name, baudrate, sub_endpoint, pub_endpoint)
    # motors_serial_relay = Hg.launch_relay(port_name, baudrate, sub_endpoint, pub_endpoint )

    return
end
end

# %% For Testing
# @warn "Testing code is uncommented"
# import Mercury as Hg
# filter_node = ImuViconPublisher.main(; debug=true);

# # %%
# filter_node_task = Threads.@spawn Hg.launch(filter_node)

# # %%
# Hg.closeall(filter_node)
