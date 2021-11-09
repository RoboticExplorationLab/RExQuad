# This node is run of the Jetson, acts as the ZMQ publisher for the IMU and Vicon
# data coming through the telemetry radio and the Arduino.
module SerialRelayStart
    import Mercury as Hg
    using TOML

    # Launch IMU publisher
    function main()::Hg.SerialZmqRelay
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        imu_serial_device = setup_dict["serial"]["jetson"]["imu_arduino"]["serial_port"]
        # imu_serial_device = "/dev/tty.usbmodem14101"
        imu_baud_rate = setup_dict["serial"]["jetson"]["imu_arduino"]["baud_rate"]

        imu_serial_ipaddr = setup_dict["zmq"]["jetson"]["imu_vicon_relay"]["in"]["server"]
        imu_serial_port = setup_dict["zmq"]["jetson"]["imu_vicon_relay"]["in"]["port"]
        imu_sub_endpoint = Hg.tcpstring(imu_serial_ipaddr, imu_serial_port)

        imu_serial_ipaddr = setup_dict["zmq"]["jetson"]["imu_vicon_relay"]["out"]["server"]
        imu_serial_port = setup_dict["zmq"]["jetson"]["imu_vicon_relay"]["out"]["port"]
        imu_pub_endpoint = Hg.tcpstring(imu_serial_ipaddr, imu_serial_port)

        imu_vicon_serial_relay = Hg.launch_relay(imu_serial_device,
                                                 imu_baud_rate,
                                                 imu_sub_endpoint,
                                                 imu_pub_endpoint,
                                                 )

        # motors_serial_relay = Hg.launch_relay(port_name, imu_baud_rate, sub_endpoint, pub_endpoint )

        return imu_vicon_serial_relay
    end
end

# # %% For Testing
# @warn "Testing code is uncommented"
# import Mercury as Hg
# imu_vicon_serial_relay = SerialRelayStart.main();

# # %%
# close(imu_vicon_serial_relay)

# # %%
# Hg.closeall(filter_node)
