# This node is run of the Jetson, it is a simple LQR controller to
# stabilize the quadrotor around a hover

module LqrHoverControllerDebug
    using TOML
    using ZMQ
    using ProtoBuf
    using SerialCOBS

    const MAX_THROTLE = 1832
    const MIN_THROTLE = 1148

    include("$(@__DIR__)/../utils/PubSubBuilder.jl")
    using .PubSubBuilder

    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/motors_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")


    function motor_commander(motor_pub_ip::String, motor_pub_port::String,
                             serial_port::String, baud_rate::Int;                     
                             freq::Int64=100, debug::Bool=false)
        rate = 1/freq
        ctx = Context(1)
        ard = Arduino(serial_port, baud_rate);

        # Setup Filtered state publisher
        motors = MOTORS(front_left=0., front_right=0., back_right=0., back_left=0.,
                        time=0.)
        motors_pub = create_pub(ctx, motor_pub_ip, motor_pub_port)
        iob = IOBuffer()
        pb = PipeBuffer()

        len = 100
        ramp = [MIN_THROTLE:(MAX_THROTLE-MIN_THROTLE)/len:MAX_THROTLE;]
        ramp = convert.(Int32, floor.([ramp; reverse(ramp)]))

        try
            open(ard) do sp

                # while true
                for throt in ramp
                    println(throt)

                    motors.front_left = throt
                    motors.front_right = MIN_THROTLE
                    motors.back_right = MIN_THROTLE
                    motors.back_left = MIN_THROTLE
                    motors.time = time()

                    msg_size = writeproto(pb, motors);
                    message(ard, take!(pb))

                    publish(motors_pub, motors, iob)

                    sleep(rate)
                    GC.gc(false)
                end
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

        serial_port = setup_dict["serial"]["jetson"]["motors_arduino"]["serial_port"]
        baud_rate = setup_dict["serial"]["jetson"]["motors_arduino"]["baud_rate"]

        motors_state_ip = setup_dict["zmq"]["jetson"]["motors"]["server"]
        motors_state_port = setup_dict["zmq"]["jetson"]["motors"]["port"]

        fs_pub() = motor_commander(motors_state_ip, motors_state_port,
                                   serial_port, baud_rate;
                                   freq=50, debug=false)
        fs_thread = Task(fs_pub)
        schedule(fs_thread)

        return fs_thread
    end
end