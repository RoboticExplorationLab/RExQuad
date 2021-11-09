# This node is run of the Jetson, it is a simple LQR controller to
# stabilize the quadrotor around a hover
module LQRcontroller
    using ..RExQuad

    import Mercury as Hg
    using ZMQ
    using StaticArrays
    using TOML

    const MIN_THROTLE = 1148.0f0
    const MAX_THROTLE = 1832.0f0

    struct MOTORS_C
        front_left::Cfloat
        front_right::Cfloat
        back_right::Cfloat
        back_left::Cfloat

        time::Cdouble
    end


    mutable struct MotorCommandNode <: Hg.Node
        # Required by Abstract Node type
        nodeio::Hg.NodeIO

        # Specific to GroundLinkNode
        # ProtoBuf Messages
        state::RExQuad.FILTERED_STATE
        motor_command::RExQuad.MOTORS

        start_time::Float64
        # Random
        debug::Bool

        function MotorCommandNode(teensy_port::String, teensy_baud::Int, debug::Bool)
            # Adding the Ground Vicon Subscriber to the Node
            motorCommandNodeIO = Hg.NodeIO(ZMQ.Context(1); rate = (10.0))

            # Adding the Quad Info Subscriber to the Node
            motor_command =
                [MOTORS_C(MIN_THROTLE, MIN_THROTLE, MIN_THROTLE, MIN_THROTLE, time())]

            motor_command_buf = @MVector zeros(UInt8, sizeof(MOTORS_C))
            motor_pub = Hg.SerialPublisher(teensy_port, teensy_baud)
            Hg.add_publisher!(motorCommandNodeIO, motor_command_buf, motor_pub)

            start_time = time()

            return new(motorCommandNodeIO, motor_command, motor_command_buf, start_time, debug)
        end
    end

    function Hg.compute(node::MotorCommandNode)
        motorCommandNodeIO = Hg.getIO(node)
        # This vector should be constant size but you can't reinterpret SArray types
        @assert length(node.motor_command) == 1
        last_command = node.motor_command[1]

        if node.debug
            println(last_command)
        end
        if abs(time() - node.start_time) < 5
            node.motor_command[1] = MOTORS_C(
                last_command.front_left + 5,
                last_command.front_right + 5,
                last_command.back_right + 5,
                last_command.back_left + 5,
                time(),
            )
            node.motor_command_buf .= reinterpret(UInt8, node.motor_command)
        else
            node.motor_command[1] =
                MOTORS_C(MIN_THROTLE, MIN_THROTLE, MIN_THROTLE, MIN_THROTLE, time())
            node.motor_command_buf .= reinterpret(UInt8, node.motor_command)
        end

        # Publish on all topics in NodeIO
        Hg.publish.(motorCommandNodeIO.pubs)
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
