# This node is run of the Jetson, it is a simple LQR controller to
# stabilize the quadrotor around a hover
module LQRcontroller
    using ..RExQuad

    import Mercury as Hg
    using ZMQ
    using StaticArrays
    using JSON
    using Rotations
    using TOML

    struct MOTORS_C
        front_left::Cfloat
        front_right::Cfloat
        back_right::Cfloat
        back_left::Cfloat
    end

    const num_input = 4
    const num_err_state = 12

    lqr_K = hcat([Vector{Float64}(vec) for vec in JSON.parsefile("src/data/lqr_gain.json")]...)
    lqr_K = SMatrix{num_input, num_err_state, Float64}(lqr_K)

    mutable struct LQRcontrollerNode <: Hg.Node
        # Required by Abstract Node type
        nodeio::Hg.NodeIO

        # ProtoBuf Messages
        state::RExQuad.FILTERED_STATE

        # Specific to GroundLinkNode
        motor_c_buf::Vector{UInt8}
        motor_c::MOTORS_C
        # motor_command::RExQuad.MOTORS

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

            motor_c_buf = reinterpret(UInt8, [MOTORS_C(RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE)])
            motor_c = MOTORS_C(RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE)
            # motor_pub = Hg.ZmqPublisher(lqrIO.ctx, motor_pub_ip, motor_pub_port;
            #                             name="MOTOR_PUB")
            # Hg.add_publisher!(lqrIO, motors, motor_pub)

            debug = debug

            return new(
                lqrIO,
                state,
                motor_c_buf,
                motor_c,
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
        end


        # Publish on all topics in NodeIO
        # Hg.publish.(motorCommandNodeIO.pubs)
    end

    # Launch IMU publisher
    function main(;rate=100.0, debug = false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]

        motor_serial_ipaddr = setup_dict["zmq"]["jetson"]["imu_vicon_relay"]["out"]["server"]
        motor_serial_port = setup_dict["zmq"]["jetson"]["imu_vicon_relay"]["out"]["port"]

        node = LQRcontrollerNode(filtered_state_ip, filtered_state_port,
                                 motor_serial_ipaddr, motor_serial_port,
                                 rate, debug)
        return node
    end
end
