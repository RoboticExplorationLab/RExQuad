# This node is run of the Jetson, it is a simple LQR controller to
# stabilize the quadrotor around a hover

module LqrHoverControllerDebug
    using TOML
    using ZMQ
    using ProtoBuf
    using SerialCOBS

    MAX_THROTLE = 1900
    THROTLE_OFF = 1500
    MIN_THROTLE = 1100

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
        motors = MOTORS(front_left=MIN_THROTLE, front_right=MIN_THROTLE, 
                        back_right=MIN_THROTLE, back_left=MIN_THROTLE,
                        time=time())
        motors_pub = create_pub(ctx, motor_pub_ip, motor_pub_port)
        iob = IOBuffer()
        pb = PipeBuffer()

        try
            open(ard) do sp 
                # motors.front_left = THROTLE_OFF
                # motors.front_right = THROTLE_OFF
                # motors.back_right = THROTLE_OFF
                # motors.back_left = THROTLE_OFF

                # msg_size = writeproto(pb, motors);
                # message(ard, take!(pb))
                # sleep(2)

                # motors.front_left = MAX_THROTLE
                # motors.front_right = MAX_THROTLE
                # motors.back_right = MAX_THROTLE
                # motors.back_left = MAX_THROTLE

                # msg_size = writeproto(pb, motors);
                # message(ard, take!(pb))
                # sleep(7)

                # motors.front_left = THROTLE_OFF
                # motors.front_right = THROTLE_OFF
                # motors.back_right = THROTLE_OFF
                # motors.back_left = THROTLE_OFF

                # msg_size = writeproto(pb, motors);
                # message(ard, take!(pb))
                # sleep(2)

                while true
                    # motors.front_left = Float64((1600 - THROTLE_OFF) * (sin(time() / 2) + 1) + THROTLE_OFF)

                    # println(motors.front_left) 
                    # motors.front_right = (1600 - MIN_THROTLE) * (sin(time() / 2 + π/2) + 1) + MIN_THROTLE 
                    # motors.back_right = (1600 - MIN_THROTLE) * (sin(time() / 2 + π) + 1) + MIN_THROTLE 
                    # motors.back_left = (1600 - MIN_THROTLE) * (sin(time() / 2 + 3π/2) + 1) + MIN_THROTLE 

                    motors.front_left = 1500 #1650
                    motors.front_right = 1500
                    motors.back_right = 1500
                    motors.back_left = 1500

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
                                   freq=20, debug=false)
        fs_thread = Task(fs_pub)
        schedule(fs_thread)

        return fs_thread
    end
end