#
module JetsonLinkDebug
    using TOML
    using ZMQ
    using ProtoBuf

    include("$(@__DIR__)/../utils/PubSubBuilder.jl")
    using .PubSubBuilder

    msgs = "$(@__DIR__)/../../msgs"
    include("$(msgs)/filtered_state_msg_pb.jl")
    include("$(msgs)/motors_msg_pb.jl")
    include("$(msgs)/vicon_msg_pb.jl")
    include("$(msgs)/quad_info_msg_pb.jl")
    include("$(msgs)/ground_info_msg_pb.jl")
    include("$(msgs)/messaging.jl")


    function quad_link(quad_info_pub_ip::String, quad_info_pub_port::String,
                       ground_info_sub_ip::String, ground_info_sub_port::String;
                       freq::Int64=20, debug::Bool=false)
        rate = 1 / freq

        ctx = Context(1)

        state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                               quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                               vel_x=0., vel_y=0., vel_z=0.,
                               ang_x=0., ang_y=0., ang_z=0.,
                               time=0.)
        motors = MOTORS(front_left=0., front_right=0., back_right=0., back_left=0.,
                        time=0.)
        vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                      quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                      time=0.)

        quad_info = QUAD_INFO(state=state, input=motors, measurement=vicon, time=time())
        quad_pub = create_pub(ctx, quad_info_pub_ip, quad_info_pub_port)
        iob = IOBuffer()

        ground_info = GROUND_INFO(deadman=true, time=0.)
        ground_info_sub() = subscriber_thread(ctx, ground_info, ground_info_sub_ip, ground_info_sub_port)

        # Setup and Schedule Subscriber Tasks
        ground_info_thread = Task(ground_info_sub)
        schedule(ground_info_thread)
        ground_info_time = 0

        try
            while true
                if (debug) println("Published QuadInfo message to ground station") end

                # If haven't heard from ground in more than a second kill
                if abs(ground_info.time - ground_info_time) > 1.0 
                    return
                end
                ground_info_time = ground_info.time

                publish(quad_pub, quad_info)

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

        quad_info_ip = setup_dict["zmq"]["jetson"]["quad_info"]["server"]
        quad_info_port = setup_dict["zmq"]["jetson"]["quad_info"]["port"]
        ground_info_sub_ip = setup_dict
        ground_info_sub_port = setup_dict

        # Launch the relay to send the Vicon data through the telemetry radio
        link_pub() = quad_link(quad_info_ip, quad_info_port,
                               ground_info_sub_ip, ground_info_sub_port;
                               freq=20, debug=debug)
        return Threads.@spawn link_pub()
    end
end