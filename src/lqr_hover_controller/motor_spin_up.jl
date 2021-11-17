# This node is run of the Jetson, it is a simple LQR controller to
# stabilize the quadrotor around a hover
module MotorSpinUp
    using ..RExQuad

    import Mercury as Hg
    using ZMQ
    using StaticArrays
    using JSON
    using Rotations
    using TOML

    include("$(@__DIR__)/serial_relay_start.jl")
    import .SerialRelayStart


    struct MOTORS_C
        front_left::Cfloat
        front_right::Cfloat
        back_right::Cfloat
        back_left::Cfloat
    end

    mutable struct MotorSpinNode <: Hg.Node
        # Required by Abstract Node type
        nodeio::Hg.NodeIO

        # Specific to GroundLinkNode
        motor_c_buf::Vector{UInt8}
        motors_relay::Hg.SerialZmqRelay

        # Random
        debug::Bool

        function MotorSpinNode(
            motor_pub_ip::String,
            motor_pub_port::String,
            rate::Float64,
            debug::Bool,
        )
            # Adding the Ground Vicon Subscriber to the Node
            motorIO = Hg.NodeIO(ZMQ.Context(1); rate = rate)

            motor_c_buf = reinterpret(UInt8, [MOTORS_C(RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE)])
            motor_pub = Hg.ZmqPublisher(motorIO.ctx, motor_pub_ip, motor_pub_port;
                                        name="MOTOR_PUB")
            Hg.add_publisher!(motorIO, motor_c_buf, motor_pub)
            motors_relay = SerialRelayStart.main()

            debug = debug

            return new(
                motorIO,
                motor_c_buf,
                motors_relay,
                debug
            )
        end
    end

    function Hg.compute(node::MotorSpinNode)
        motorIO = Hg.getIO(node)
        Hg.check_relay_running(node.motors_relay)

        motor_c = MOTORS_C(RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE)
        node.motor_c_buf = reinterpret(UInt8, [motor_c])

        Hg.publish.(motorIO.pubs)
    end

    function Hg.finishup(node::MotorSpinNode)
        close(node.motors_relay)
    end

    # Launch IMU publisher
    function main(;rate=10.0, debug = false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        motor_serial_ipaddr = setup_dict["zmq"]["jetson"]["motors_relay"]["in"]["server"]
        motor_serial_port = setup_dict["zmq"]["jetson"]["motors_relay"]["in"]["port"]

        node = MotorSpinNode(motor_serial_ipaddr, motor_serial_port,
                             rate, debug)
        return node
    end
end
