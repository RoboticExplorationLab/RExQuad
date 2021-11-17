module SerialRelayStart
    import Mercury as Hg
    using TOML

    # Launch IMU publisher
    function main()::Hg.SerialZmqRelay
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        # motors_serial_device = setup_dict["serial"]["jetson"]["motors_arduino"]["serial_port"]
        motors_serial_device = "/dev/tty.usbmodem14201"
        motors_baud_rate = setup_dict["serial"]["jetson"]["motors_arduino"]["baud_rate"]

        motors_serial_ipaddr = setup_dict["zmq"]["jetson"]["motors_relay"]["in"]["server"]
        motors_serial_port = setup_dict["zmq"]["jetson"]["motors_relay"]["in"]["port"]
        motors_sub_endpoint = Hg.tcpstring(motors_serial_ipaddr, motors_serial_port)

        motors_serial_ipaddr = setup_dict["zmq"]["jetson"]["motors_relay"]["out"]["server"]
        motors_serial_port = setup_dict["zmq"]["jetson"]["motors_relay"]["out"]["port"]
        motors_pub_endpoint = Hg.tcpstring(motors_serial_ipaddr, motors_serial_port)

        motors_serial_relay = Hg.launch_relay(motors_serial_device,
                                              motors_baud_rate,
                                              motors_sub_endpoint,
                                              motors_pub_endpoint,
                                              )
        return motors_serial_relay
    end
end