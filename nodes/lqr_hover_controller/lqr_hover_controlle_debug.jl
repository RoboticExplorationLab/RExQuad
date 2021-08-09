# This node is run of the Jetson, it is a simple LQR controller to
# stabilize the quadrotor around a hover

module LqrHoverController
    using TOML
    using ZMQ
    using ProtoBuf

    include("$(@__DIR__)/../utils/PubSubBuilder.jl")
    using .PubSubBuilder

    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/motors_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")


    function motor_commander(motor_pub_ip::String, motor_pub_port::String;
                             freq::Int64=100, debug::Bool=false)
        rate = 1/freq
        ctx = Context(1)

        # Setup Filtered state publisher
        motors = MOTORS(front_left=0., front_right=0., back_right=0., back_left=0.,
                        time=0.)
        motors_pub = create_pub(ctx, motor_pub_ip, motor_pub_port)
        iob = IOBuffer()

        state_time = time()

        try
            while true
                motors.front_left, motors.front_right, motors.back_right, motors.back_left = rand(4)

                publish(motors_pub, motors, iob)

                sleep(rate)
            end
        catch e
            close(motors_pub)
            close(ctx)

            if e isa InterruptException
                println("Process terminated by you")
            else
                rethrow(e)
            end
        end
    end

    # Launch IMU publisher
    function main()
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        motors_state_ip = setup_dict["zmq"]["jetson"]["motors"]["server"]
        motors_state_port = setup_dict["zmq"]["jetson"]["motors"]["port"]

        fs_pub() = motor_commander(motors_state_ip, motors_state_port;
                                   freq=100, debug=false)
        fs_thread = Task(fs_pub)
        schedule(fs_thread)

        return fs_thread
    end
end