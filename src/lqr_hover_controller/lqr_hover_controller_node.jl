# This node is run of the Jetson, it is a simple LQR controller to
# stabilize the quadrotor around a hover
module LQRcontroller
    using ..RExQuad

    import Mercury as Hg
    using ZMQ
    using StaticArrays
    using TOML

    const lqr_K = generate_LQR_hover_gains(
        Qd = ones(12),
        Rd = fill(0.1, 4);
        save_to_file = false,
    )

    # function wrench2pwm(force_torque::SVector{8})

    # end

    mutable struct LQRcontrollerNode <: Hg.Node
        # Required by Abstract Node type
        nodeio::Hg.NodeIO

        # Specific to GroundLinkNode
        # ProtoBuf Messages
        state::RExQuad.FILTERED_STATE
        motor_command::RExQuad.MOTORS

        start_time::Float64
        # Random
        debug::Bool

        function LQRcontrollerNode(
            state_sub_ip::String,
            state_sub_port::String,
            motor_pub_ip::String,
            motor_pub_port::String,
            rate::Float64,
            debug::Bool,
        )
            # Adding the Ground Vicon Subscriber to the Node
            lqrIO = Hg.NodeIO(ZMQ.Context(1); rate = rate)

            state = RExQuad.zero_FILTERED_STATE()
            state_sub = Hg.ZmqSubscriber(lqrIO.ctx, state_sub_ip, state_sub_port;
                                         name="FILTERED_STATE_SUB")
            Hg.add_subscriber!(lqrIO, state, state_sub)

            motors = RExQuad.zero_MOTORS()
            motor_pub = Hg.ZmqPublisher(lqrIO.ctx, motor_pub_ip, motor_pub_port;
                                        name="MOTOR_PUB")
            Hg.add_publisher!(lqrIO, motors, motor_pub)

            start_time = time()
            debug = debug

            return new(
                motorCommandNodeIO,
                motor_command,
                motor_command_buf,
                start_time,
                debug
            )
        end
    end

    function Hg.compute(node::LQRcontrollerNode)
        lqrIO = Hg.getIO(node)
        # This vector should be constant size but you can't reinterpret SArray types

        state_sub = Hg.getsubscriber(node, "FILTERED_STATE_SUB")
        Hg.on_new(state_sub) do state
            state_vec = SA[state.pos_x, state.pos_y, state.pos_z,
                           state.quat_w, state.quat_x, state.quat_y, state.quat_z,
                           state.vel_x, state.vel_y, state.vel_z,
                           state.ang_x, state.ang_y, state.ang_z]
            display(size(lqr_K))
            display(size(state_vec))
            # display(size(lqr_K * state_vec))
        end


        # Publish on all topics in NodeIO
        # Hg.publish.(motorCommandNodeIO.pubs)
    end

    # Launch IMU publisher
    function main(; debug = false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        serial_port = setup_dict["serial"]["jetson"]["motors_arduino"]["serial_port"]
        baud_rate = setup_dict["serial"]["jetson"]["motors_arduino"]["baud_rate"]

        serial_port = "/dev/ttyACM0"
        baud_rate = 115200

        node = MotorCommandNode(serial_port, baud_rate, debug)
        return node
    end
end
