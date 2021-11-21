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
    end

    function MotorSpinNode( rate::Float64, debug::Bool, )
        # Adding the Ground Vicon Subscriber to the Node
        motorIO = Hg.NodeIO(ZMQ.Context(1); rate = rate)
        motor_c_buf = reinterpret(UInt8, sizeof(MOTORS_C))
        motors_relay = run(`true`)
        debug = debug

        return MotorSpinNode( motorIO, motor_c_buf, motors_relay, debug, )
    end

    function Hg.setupIO!(node::MotorSpinNode, nodeio::Hg.NodeIO)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        motor_serial_ipaddr = setup_dict["zmq"]["jetson"]["motors_relay"]["in"]["server"]
        motor_serial_port = setup_dict["zmq"]["jetson"]["motors_relay"]["in"]["port"]
        motors_sub_endpoint = Hg.tcpstring(motors_serial_ipaddr, motors_serial_port)

        motor_pub = Hg.ZmqPublisher(nodeio.ctx, motor_serial_ipaddr, motor_serial_port;
                                    name="MOTOR_PUB")
        Hg.add_publisher!(nodeio, node.motor_c_buf, motor_pub)

        motors_serial_device = setup_dict["serial"]["jetson"]["motors_arduino"]["serial_port"]
        # motors_serial_device = "/dev/tty.usbmodem14201"
        motors_baud_rate = setup_dict["serial"]["jetson"]["motors_arduino"]["baud_rate"]

        motors_serial_ipaddr = setup_dict["zmq"]["jetson"]["motors_relay"]["out"]["server"]
        motors_serial_port = setup_dict["zmq"]["jetson"]["motors_relay"]["out"]["port"]
        motors_pub_endpoint = Hg.tcpstring(motors_serial_ipaddr, motors_serial_port)

        node.motors_relay = Hg.launch_relay(motors_serial_device,
                                            motors_baud_rate,
                                            motors_sub_endpoint,
                                            motors_pub_endpoint,
                                            )
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
end
