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

    include("$(@__DIR__)/serial_relay_start.jl")
    import .SerialRelayStart

    include("generate_lqr_gains.jl")

    const num_input = 4
    const num_err_state = 12
    lqr_K = hcat([Vector{Float64}(vec) for vec in JSON.parsefile("src/data/lqr_gain.json")]...)
    lqr_K = SMatrix{num_input, num_err_state, Float64}(lqr_K)
    u_hover = Vector{Float64}(JSON.parsefile("src/data/u_equilibrium.json"))

    mutable struct LQRcontrollerNode <: Hg.Node
        nodeio::Hg.NodeIO
        state::RExQuad.FILTERED_STATE         # ProtoBuf Messages
        motor_c_buf::Vector{UInt8}
        motors_relay::Hg.SerialZmqRelay
        start_time::Float64
        end_time::Float64
        cnt::Int64
        debug::Bool
    end

    function LQRcontrollerNode(rate::Float64, debug::Bool, )
            # Adding the Ground Vicon Subscriber to the Node
            lqrIO = Hg.NodeIO(ZMQ.Context(1); rate = rate)
            state = RExQuad.zero_FILTERED_STATE()

            motor_c = MOTORS_C(RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE)
            motor_c_buf = reinterpret(UInt8, [motor_c])
            motors_relay = run(`true`)

            start_time = time()
            end_time = time()
            cnt = 0
            debug = debug

            return LQRcontrollerNode(lqrIO, state, motor_c_buf, motors_relay, start_time, end_time, cnt, debug )
        end

    function Hg.setupIO!(node::LQRcontrollerNode, nodeio::Hg.NodeIO)
        ##### Create State/Motor Publisher #####
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]

        motor_serial_ipaddr = setup_dict["zmq"]["jetson"]["motors_relay"]["in"]["server"]
        motor_serial_port = setup_dict["zmq"]["jetson"]["motors_relay"]["in"]["port"]
        motor_sub_endpoint = Hg.tcpstring(motor_serial_ipaddr, motor_serial_port)

        state_sub = Hg.ZmqSubscriber(nodeio.ctx, filtered_state_ip, filtered_state_port;
                                     name="FILTERED_STATE_SUB")
        Hg.add_subscriber!(nodeio, node.state, state_sub)
        motor_pub = Hg.ZmqPublisher(nodeio.ctx, motor_serial_ipaddr, motor_serial_port;
                                    name="MOTOR_PUB")
        Hg.add_publisher!(nodeio, node.motor_c_buf, motor_pub)

        ##### Startup Motor Adafruit Feather Relay #####
        motor_serial_device = setup_dict["serial"]["jetson"]["motors_arduino"]["serial_port"]
        # motor_serial_device = "/dev/tty.usbmodem14201"
        motor_baud_rate = setup_dict["serial"]["jetson"]["motors_arduino"]["baud_rate"]

        motor_serial_ipaddr = setup_dict["zmq"]["jetson"]["motors_relay"]["out"]["server"]
        motor_serial_port = setup_dict["zmq"]["jetson"]["motors_relay"]["out"]["port"]
        motor_pub_endpoint = Hg.tcpstring(motor_serial_ipaddr, motor_serial_port)

        node.motors_relay = Hg.launch_relay(motor_serial_device,
                                            motor_baud_rate,
                                            motor_sub_endpoint,
                                            motor_pub_endpoint,
                                            )
        return nothing
    end

    function Hg.compute(node::LQRcontrollerNode)
        lqrIO = Hg.getIO(node)
        # This vector should be constant size but you can't reinterpret SArray types

        motor_pub = Hg.getpublisher(node, "MOTOR_PUB").pub
        state_sub = Hg.getsubscriber(node, "FILTERED_STATE_SUB")

        Hg.on_new(state_sub) do state
            state_vec = SA[state.pos_x, state.pos_y, state.pos_z,
                           state.quat_w, state.quat_x, state.quat_y, state.quat_z,
                           state.vel_x, state.vel_y, state.vel_z,
                           state.ang_x, state.ang_y, state.ang_z]
            state_err = compute_err_state(state_vec)

            inputs = -lqr_K * state_err + u_hover
            inputs = clamp.(inputs, RExQuad.MIN_THROTLE, RExQuad.MAX_THROTLE)

            motor_c = MOTORS_C(inputs[1], inputs[2], inputs[3], inputs[4])
            node.motor_c_buf = reinterpret(UInt8, [motor_c])
            Hg.publish.(motor_pub, node.motor_c_buf)

            if node.debug
                node.cnt += 1
                if node.cnt % floor(lqrIO.opts.rate) == 0
                    node.end_time = time()

                    @info "Control rate: $(floor(lqrIO.opts.rate)/ (node.end_time - node.start_time))"

                    @printf("State Error: \t[%1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f]\n",
                            state_err[1:7]...)
                    @printf("Motor PWM Commands: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
                            motor_c.front_left, motor_c.front_right, motor_c.back_right, motor_c.back_left)

                    node.start_time = time()
                end
            end
        end
    end

    # Launch IMU publisher
    function main(; rate = 100.0, debug = false)
        node = LQRcontrollerNode(rate, debug)
        Hg.setupIO!(node, Hg.getIO(node))

        return node
    end
end
