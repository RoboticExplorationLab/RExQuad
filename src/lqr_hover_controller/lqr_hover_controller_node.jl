# This node is run of the Jetson, it is a simple LQR controller to
# stabilize the quadrotor around a hover
module LQRcontroller
    using ..RExQuad

    import Mercury as Hg
    using ZMQ
    using StaticArrays
    using JSON
    using Rotations
    using Printf
    using TOML

    struct MOTORS_C
        front_left::Cfloat
        front_right::Cfloat
        back_right::Cfloat
        back_left::Cfloat
    end

    include("generate_lqr_gains.jl")

    const num_input = 4
    const num_err_state = 12
    lqr_K = hcat([Vector{Float64}(vec) for vec in JSON.parsefile("src/data/lqr_gain.json")]...)
    lqr_K = SMatrix{num_input, num_err_state, Float64}(lqr_K)
    u_hover = Vector{Float64}(JSON.parsefile("src/data/u_equilibrium.json"))

    mutable struct LQRcontrollerNode <: Hg.Node
        nodeio::Hg.NodeIO
        state::RExQuad.FILTERED_STATE         # ProtoBuf Messages
        motors::RExQuad.MOTORS
        start_time::Float64
        end_time::Float64
        cnt::Int64
        debug::Bool
    end

    function LQRcontrollerNode(rate::Float64, debug::Bool, )
        # Adding the Ground Vicon Subscriber to the Node
        lqrIO = Hg.NodeIO(ZMQ.Context(1); rate = rate)
        state = RExQuad.zero_FILTERED_STATE()
        motors = RExQuad.zero_MOTORS()

        start_time = time()
        end_time = time()
        cnt = 0

        debug = debug

        return LQRcontrollerNode(lqrIO, state, motors, start_time, end_time, cnt, debug )
    end

    function Hg.setupIO!(node::LQRcontrollerNode, nodeio::Hg.NodeIO)
        ##### Create State/Motor Publisher #####
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]
        state_sub = Hg.ZmqSubscriber(nodeio.ctx, filtered_state_ip, filtered_state_port; name="FILTERED_STATE_SUB")
        Hg.add_subscriber!(nodeio, node.state, state_sub)

        motors_ip = setup_dict["zmq"]["jetson"]["motors"]["server"]
        motors_port = setup_dict["zmq"]["jetson"]["motors"]["port"]
        motors_pub = Hg.ZmqPublisher(nodeio.ctx, motors_ip, motors_port; name="MOTORS_PUB")
        Hg.add_publisher!(nodeio, node.motors, motors_pub)
    end

    function Hg.compute(node::LQRcontrollerNode)
        lqrIO = Hg.getIO(node)

        motors_pub = Hg.getpublisher(node, "MOTOR_PUB").pub
        state_sub = Hg.getsubscriber(node, "FILTERED_STATE_SUB")

        Hg.on_new(state_sub) do state
            state_vec = SA[state.pos_x, state.pos_y, state.pos_z,
                           state.quat_w, state.quat_x, state.quat_y, state.quat_z,
                           state.vel_x, state.vel_y, state.vel_z,
                           state.ang_x, state.ang_y, state.ang_z]
            state_err = compute_err_state(state_vec)

            inputs = -lqr_K * state_err + u_hover
            inputs = clamp.(inputs, RExQuad.MIN_THROTLE, RExQuad.MAX_THROTLE)

            node.motors.front_left = inputs[1]
            node.motors.front_right = inputs[2]
            node.motors.back_right = inputs[3]
            node.motors.back_left = inputs[4]

            if node.debug
                node.cnt += 1
                if node.cnt % floor(lqrIO.opts.rate) == 0
                    node.end_time = time()

                    @info "Control rate: $(floor(lqrIO.opts.rate)/ (node.end_time - node.start_time))"
                    @printf("State Error: \t[%1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f]\n",
                            state_err[1:7]...)
                    @printf("Motor PWM Commands: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
                            node.motors.front_left, node.motors.front_right,
                            node.motors.back_right, node.motors.back_left)

                    node.start_time = time()
                end
            end

            Hg.publish(motors_pub, nod.motors)
        end
    end
end
