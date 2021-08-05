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


    function motor_commander(filtered_state_sub_ip::String, filtered_state_sub_port::String,
                             motor_pub_ip::String, motor_pub_port::String;
                             freq::Int64=200, debug::Bool=false)
        ctx = Context(1)

        # Initalize Subscriber threads
        state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                               quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                               vel_x=0., vel_y=0., vel_z=0.,
                               ang_x=0., ang_y=0., ang_z=0.)
        state_sub() = subscriber_thread(ctx, state, filtered_state_sub_ip, filtered_state_sub_port)
        # Setup and Schedule Subscriber Tasks
        state_thread = Task(state_sub)
        schedule(state_thread)

        # Setup Filtered state publisher
        motors = MOTORS(front_left=0., front_right=0., back_right=0., back_left=0.,
                        time=0.)
        motors_pub = create_pub(ctx, motor_pub_ip, motor_pub_port)
        iob = PipeBuffer()

        state_time = time()

        try
            while true
                # Prediction
                if state.time > state_time
                    # TODO: Run controller here
                    # controller()



                    writeproto(iob, motors)
                    ZMQ.send(motors_pub, take!(iob))

                    state_time = state.time
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

        zmq_filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        zmq_filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]
        zmq_motors_state_ip = setup_dict["zmq"]["jetson"]["motors"]["server"]
        zmq_motors_state_port = setup_dict["zmq"]["jetson"]["motors"]["port"]

        fs_pub() = motor_commander(zmq_filtered_state_ip, zmq_filtered_state_port,
                                   zmq_motors_state_ip, zmq_motors_state_port;
                                   freq=200, debug=false)
        fs_thread = Task(fs_pub)
        schedule(fs_thread)

        return fs_thread
    end
end