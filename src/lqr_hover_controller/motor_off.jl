# This node is run of the Jetson, it is a simple LQR controller to
# stabilize the quadrotor around a hover
module MotorSpinUp
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

    mutable struct MotorSpinNode <: Hg.Node
        nodeio::Hg.NodeIO
        motor_c_buf::Vector{UInt8}
        motors_relay::Hg.SerialZmqRelay
        last_time::Float64
        start_time::Float64
        debug::Bool
    end

    function MotorSpinNode( rate::Float64, debug::Bool, )
        # Adding the Ground Vicon Subscriber to the Node
        motorIO = Hg.NodeIO(ZMQ.Context(1); rate = rate)
        motor_c_buf = reinterpret(UInt8, [MOTORS_C(0.,0.,0.,0.)])
        motors_relay = run(`true`)

        last_time = time()
        start_time = time()
        debug = debug

        return MotorSpinNode( motorIO, motor_c_buf, motors_relay, last_time, start_time, debug, )
    end

    function Hg.setupIO!(node::MotorSpinNode, nodeio::Hg.NodeIO)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        motor_serial_ipaddr = setup_dict["zmq"]["jetson"]["motors_relay"]["in"]["server"]
        motor_serial_port = setup_dict["zmq"]["jetson"]["motors_relay"]["in"]["port"]
        motors_sub_endpoint = Hg.tcpstring(motor_serial_ipaddr, motor_serial_port)

        motor_pub = Hg.ZmqPublisher(nodeio.ctx, motor_serial_ipaddr, motor_serial_port;
                                    name="MOTOR_PUB")
        Hg.add_publisher!(nodeio, node.motor_c_buf, motor_pub)

        motors_serial_device = setup_dict["serial"]["jetson"]["motors_arduino"]["serial_port"]
        # motors_serial_device = "/dev/tty.usbmodem14101"
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

    function Hg.startup(node::MotorSpinNode)
        node.last_time = time()
        node.start_time = time()
    end

    function Hg.compute(node::MotorSpinNode)
        motorIO = Hg.getIO(node)
        Hg.check_relay_running(node.motors_relay)

        ramp_len = 2.5 # 5 second ramp up 5 second ramp down
        max_throt = (RExQuad.MAX_THROTLE+RExQuad.MIN_THROTLE)/2

        function f(t)
            if 0 < t < ramp_len
                return RExQuad.MIN_THROTLE + (max_throt-RExQuad.MIN_THROTLE)/ramp_len * t
            elseif ramp_len < t < 2*ramp_len
                return max_throt - (max_throt-RExQuad.MIN_THROTLE)/ramp_len * (t-ramp_len)
            else
                return RExQuad.MIN_THROTLE
            end
        end

        # # Motors ramp
        # throt = f(time() - node.start_time)
        # motor_c = MOTORS_C(throt, throt, throt, throt)

        # Motors off
        motor_c = MOTORS_C(RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE, RExQuad.MIN_THROTLE)

        # # Motors on
        # motor_c = MOTORS_C(RExQuad.MIN_THROTLE+50, RExQuad.MIN_THROTLE+50, RExQuad.MIN_THROTLE+50, RExQuad.MIN_THROTLE+50)

        node.motor_c_buf .= reinterpret(UInt8, [motor_c])

        if node.debug
            @printf(
                "Motor Commands: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
                motor_c.front_left,
                motor_c.front_right,
                motor_c.back_right,
                motor_c.back_left
            )
        end

        node.last_time = time()

        Hg.publish.(motorIO.pubs)
    end

    function Hg.finishup(node::MotorSpinNode)
        close(node.motors_relay)
    end
end
