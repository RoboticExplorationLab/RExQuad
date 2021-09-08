#
module JetsonLink
    using TOML
    using ZMQ
    using ProtoBuf

    include("$(@__DIR__)/../utils/PubSubBuilder.jl")
    using .PubSubBuilder

    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/motors_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/quad_info_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/ground_info_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")


    function quad_link(state_sub_ip::String, state_sub_port::String,
                       motors_sub_ip::String, motors_sub_port::String,
                       vicon_sub_ip::String, vicon_sub_port::String,
                       quad_info_pub_ip::String, quad_info_pub_port::String,
                       ground_info_sub_ip::String, ground_info_sub_port::String;
                       freq::Int64=20, debug::Bool=false)
        rate = 1 / freq
        ctx = Context(1)

        ground_info = GROUND_INFO(deadman=true, time=0.)
        ground_info_sub() = subscriber_thread(ctx, ground_info, ground_info_sub_ip, ground_info_sub_port)
        # Setup and Schedule Subscriber Tasks
        ground_info_thread = Task(ground_info_sub)
        schedule(ground_info_thread)
        ground_info_time = ground_info.time
        first_loop = true

        state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                               quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                               vel_x=0., vel_y=0., vel_z=0.,
                               ang_x=0., ang_y=0., ang_z=0.,
                               time=0.)
        state_sub() = subscriber_thread(ctx, state, state_sub_ip, state_sub_port)
        # Setup and Schedule Subscriber Tasks
        state_thread = Task(state_sub)
        schedule(state_thread)
        state_time = state.time

        motors = MOTORS(front_left=0., front_right=0., back_right=0., back_left=0.,
                        time=0.)
        motors_sub() = subscriber_thread(ctx, motors, motors_sub_ip, motors_sub_port)
        # Setup and Schedule Subscriber Tasks
        motors_thread = Task(motors_sub)
        schedule(motors_thread)
        motors_time = motors.time

        vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                      quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                      time=0.)
        vicon_sub() = subscriber_thread(ctx, vicon, vicon_sub_ip, vicon_sub_port)
        # Setup and Schedule Subscriber Tasks
        vicon_thread = Task(vicon_sub)
        schedule(vicon_thread)
        vicon_time = vicon.time

        quad_info = QUAD_INFO(state=state, input=motors, measurement=vicon, time=time())
        quad_pub = create_pub(ctx, quad_info_pub_ip, quad_info_pub_port)

        # Setup inial times
        iob = IOBuffer()

        try
            if (debug)
                println("Waiting for consitstent Heartbeat")
            end

            # Wait until we've heard 100
            # cnt = 0
            # while cnt < 500
            #     while ground_info_time == ground_info.time
            #         sleep(0.1)
            #     end
            #     ground_info_time = ground_info.time
            #     cnt += 1
            # end

            if (debug)
                println("Consistent Heartbeat established")
            end

            cnt = 0
            last_time = time()

            while true
                if (debug)
                    println("Time since last heartbeat: ", abs(ground_info_time - ground_info.time))
                end

                # if abs(time() - ground_info.time) > 5.0
                #     # If haven't heard from ground in more than a second kill
                #     error("\nDeadman switched off!!\n")
                # end
                ground_info_time = ground_info.time

                pub = true
                # pub = false

                if state.time > state_time
                    state_time = state.time
                    pub = true
                end
                if motors.time > motors_time
                    motors_time = motors.time
                    pub = true
                end
                if vicon.time > vicon_time
                    vicon_time = vicon.time
                    pub = true
                end

                if pub
                    quad_info.time = time()
                    if (debug)
                        println("Published QuadInfo message to ground station")

                        if cnt % 100 == 0
                            loop_run_rate = 100 / (time() - last_time)
                            println("jetson_link Frequency (Hz): ", loop_run_rate)
                            last_time = time()
                        end
                        cnt += 1
                    end

                    publish(quad_pub, quad_info)
                end

                sleep(rate)
                GC.gc(false)
            end
        catch e
            close(quad_pub)
            close(ctx)

            if e isa InterruptException
                println("Shutting down Jetson Link")
            else
                rethrow(e)
            end
        end
    end

    # Launch IMU publisher
    function main(; debug=false)::Task
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]
        motors_ip = setup_dict["zmq"]["jetson"]["motors"]["server"]
        motors_port = setup_dict["zmq"]["jetson"]["motors"]["port"]
        vicon_ip = setup_dict["zmq"]["jetson"]["vicon"]["server"]
        vicon_port = setup_dict["zmq"]["jetson"]["vicon"]["port"]
        quad_info_ip = setup_dict["zmq"]["jetson"]["quad_info"]["server"]
        quad_info_port = setup_dict["zmq"]["jetson"]["quad_info"]["port"]
        ground_info_ip = setup_dict["zmq"]["ground"]["ground_info"]["server"]
        ground_info_port = setup_dict["zmq"]["ground"]["ground_info"]["port"]

        # Launch the relay to send the Vicon data through the telemetry radio
        link_pub() = quad_link(filtered_state_ip, filtered_state_port,
                               motors_ip, motors_port,
                               vicon_ip, vicon_port,
                               quad_info_ip, quad_info_port,
                               ground_info_ip, ground_info_port;
                               freq=50, debug=debug)
        return Threads.@spawn link_pub()
    end
end