#
module JetsonMaster
    using ..RExQuad

    import Mercury as Hg
    using ZMQ
    using Printf
    using TOML

    struct MOTORS_C
        front_left::Cfloat
        front_right::Cfloat
        back_right::Cfloat
        back_left::Cfloat
    end

    mutable struct JetsonMasterNode <: Hg.Node
        nodeio::Hg.NodeIO
        ground_info::RExQuad.GROUND_INFO
        state::RExQuad.FILTERED_STATE
        motors::RExQuad.MOTORS
        quad_info::RExQuad.QUAD_INFO
        motor_c_buf::Vector{UInt8}
        motors_relay::Hg.SerialZmqRelay
        debug::Bool
    end

    function JetsonMasterNode(rate::Float64, debug::Bool)
        jetsonMasterNodeIO = Hg.NodeIO(ZMQ.Context(1); rate = rate)

        ground_info = RExQuad.zero_GROUND_INFO()
        state = RExQuad.zero_FILTERED_STATE()
        motors = RExQuad.zero_MOTORS()
        quad_info = QUAD_INFO(state = state, input = motors, time = 0.0)

        motor_c = MOTORS_C(RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE)
        motor_c_buf = reinterpret(UInt8, [motor_c])
        motors_relay = run(`true`)

        debug=debug

        return JetsonMasterNode(
            jetsonMasterNodeIO,
            ground_info,
            state,
            motors,
            quad_info,
            motor_c_buf,
            motors_relay,
            debug
        )
    end

    function Hg.setupIO!(node::JetsonMasterNode, nodeio::Hg.NodeIO)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        ground_info_ip = setup_dict["zmq"]["ground"]["ground_info"]["server"]
        ground_info_port = setup_dict["zmq"]["ground"]["ground_info"]["port"]
        ground_info_sub = Hg.ZmqSubscriber(nodeio.ctx, ground_info_ip, ground_info_port; name="GROUND_INFO_SUB")
        Hg.add_subscriber!(nodeio, node.ground_info, ground_info_sub)

        filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]
        state_sub = Hg.ZmqSubscriber(nodeio.ctx, filtered_state_ip, filtered_state_port; name="FILTERED_STATE_SUB")
        Hg.add_subscriber!(nodeio, node.state, state_sub)

        motors_ip = setup_dict["zmq"]["jetson"]["motors"]["server"]
        motors_port = setup_dict["zmq"]["jetson"]["motors"]["port"]
        motors_sub = Hg.ZmqSubscriber(nodeio.ctx, motors_ip, motors_port; name="MOTORS_SUB")
        Hg.add_subscriber!(nodeio, node.motors, motors_sub)

        quad_info_ip = setup_dict["zmq"]["jetson"]["quad_info"]["server"]
        quad_info_port = setup_dict["zmq"]["jetson"]["quad_info"]["port"]
        quad_info_pub = Hg.ZmqPublisher(nodeio.ctx, quad_info_ip, quad_info_port; name="QUAD_INFO_PUB")
        Hg.add_publisher!(nodeio, node.quad_info, quad_info_pub)

        motors_serial_device = setup_dict["serial"]["jetson"]["motors_arduino"]["serial_port"]
        motors_baud_rate = setup_dict["serial"]["jetson"]["motors_arduino"]["baud_rate"]

        motors_serial_in_ipaddr = setup_dict["zmq"]["jetson"]["motors_relay"]["in"]["server"]
        motors_serial_in_port = setup_dict["zmq"]["jetson"]["motors_relay"]["in"]["port"]
        motors_sub_endpoint = Hg.tcpstring(motors_serial_in_ipaddr, motors_serial_in_port)

        motors_serial_out_ipaddr = setup_dict["zmq"]["jetson"]["motors_relay"]["out"]["server"]
        motors_serial_out_port = setup_dict["zmq"]["jetson"]["motors_relay"]["out"]["port"]
        motors_pub_endpoint = Hg.tcpstring(motors_serial_out_ipaddr, motors_serial_out_port)

        ##### Startup Motor Adafruit Feather Relay #####
        node.motors_relay = Hg.launch_relay(motors_serial_device,
                                            motors_baud_rate,
                                            motors_sub_endpoint,
                                            motors_pub_endpoint,
                                            )
        ##### Link node to relay #####
        motors_serial_pub = Hg.ZmqPublisher(nodeio.ctx, motors_serial_in_ipaddr, motors_serial_in_port; name="MOTORS_SERIAL_PUB")
        Hg.add_publisher!(nodeio, node.motor_c_buf, motors_serial_pub)
    end

    function Hg.compute(node::JetsonMasterNode)
        jetsonMasterNodeIO = Hg.getIO(node)
        Hg.check_relay_running(node.motors_relay)

        motors_serial_pub = Hg.getpublisher(node, "MOTORS_SERIAL_PUB").pub
        quad_info_pub = Hg.getpublisher(node, "QUAD_INFO_PUB").pub

        state_sub = Hg.getsubscriber(node, "FILTERED_STATE_SUB")
        motors_sub = Hg.getsubscriber(node, "MOTORS_SUB")

        # Update the quadinfo state attribute
        Hg.on_new(state_sub) do state
            node.quad_info.state = state
        end

        # Update the send new velocity command to motors
        Hg.on_new(motors_sub) do motors
            node.quad_info.input = motors

            motors_c = MOTORS_C(motors.front_left, motors.front_right, motors.back_right, motors.back_left)
            # ⚠️CRITICAL⚠️ must use .= opperator here to makesure we are writting to same piece of memory
            node.motor_c_buf .= reinterpret(UInt8, [motor_c])
            Hg.publish(motors_serial_pub, node.motor_c_buf)
        end

        # Print debug info and publish quad_info message
        if node.debug
            @printf(
                "Position: \t[%1.3f, %1.3f, %1.3f]\n",
                node.quad_info.state.pos_x,
                node.quad_info.state.pos_y,
                node.quad_info.state.pos_z
            )
            @printf(
                "Orientation: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
                node.quad_info.state.quat_w,
                node.quad_info.state.quat_x,
                node.quad_info.state.quat_y,
                node.quad_info.state.quat_z
            )
        end
        node.quad_info.time = time()
        Hg.publish(quad_info_pub, node.quad_info)
    end

    function Hg.finishup(node::JetsonMasterNode)
        motors_serial_pub = Hg.getpublisher(node, "MOTORS_SERIAL_PUB").pub

        for i in 1:100
            motor_c = MOTORS_C(RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE,
                               RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE)
            node.motor_c_buf .= reinterpret(UInt8, [motor_c])
            Hg.publish(motors_serial_pub, node.motor_c_buf)
            sleep(0.005)
        end

        close(node.motors_relay)
    end
end
