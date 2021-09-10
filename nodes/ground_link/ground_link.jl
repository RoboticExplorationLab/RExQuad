# Node for communicating with the Jetson (run on the ground station)
module GroundLink
    using TOML
    using ZMQ
    using ProtoBuf
    using Printf
    using StaticArrays

    include("$(@__DIR__)/visualizer.jl")

    include("$(@__DIR__)/../utils/PubSubBuilder.jl")
    using .PubSubBuilder

    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/motors_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/quad_info_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/ground_info_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")


    function quad_link(
                       vicon_ground_sub_ip::String, vicon_ground_sub_port::String,
                       vicon_jetson_sub_ip::String, vicon_jetson_sub_port::String;
                       freq::Int64=200, debug::Bool=false)
        rate = 1 / freq
        ctx = Context(1)

        ground_vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                            quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                            time=0.)
        ground_vicon_sub() = subscriber_thread(ctx, ground_vicon, vicon_ground_sub_ip, vicon_ground_sub_port)
        # Setup and Schedule Subscriber Tasks
        ground_vicon_thread = Task(ground_vicon_sub)
        schedule(ground_vicon_thread)

        jetson_vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                            quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                            time=0.)
        jetson_vicon_sub() = subscriber_thread(ctx, jetson_vicon, vicon_jetson_sub_ip, vicon_jetson_sub_port)

        # Setup and Schedule Subscriber Tasks
        jetson_vicon_thread = Task(jetson_vicon_sub)
        schedule(jetson_vicon_thread)

        # Create a thread for the 3D visualizer (showing the state estimate)
        vis = QuadVisualizer()
        open(vis)
        add_copy!(vis)

        try
            while true
                TrajOptPlots.visualize!(vis,
                                        SA[ground_vicon.pos_x,
                                           ground_vicon.pos_y,
                                           ground_vicon.pos_z,
                                           ground_vicon.quat_w,
                                           ground_vicon.quat_x,
                                           ground_vicon.quat_y,
                                           ground_vicon.quat_z,
                                           0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0],
                                        SA[jetson_vicon.pos_x,
                                           jetson_vicon.pos_y,
                                           jetson_vicon.pos_z,
                                           jetson_vicon.quat_w,
                                           jetson_vicon.quat_x,
                                           jetson_vicon.quat_y,
                                           jetson_vicon.quat_z,
                                           0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0],)

                if (debug)
                    @printf("Jetson Vicon:\n")
                    @printf("\tPosition: \t[%1.3f, %1.3f, %1.3f]\n",
                            jetson_vicon.pos_x, jetson_vicon.pos_y, jetson_vicon.pos_z)
                    @printf("\tQuaternion: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
                            jetson_vicon.quat_w, jetson_vicon.quat_x, jetson_vicon.quat_y, jetson_vicon.quat_z)

                    @printf("Ground Vicon:\n")
                    @printf("\tPosition: \t[%1.3f, %1.3f, %1.3f]\n",
                            ground_vicon.pos_x, ground_vicon.pos_y, ground_vicon.pos_z)
                    @printf("\tQuaternion: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
                            ground_vicon.quat_w, ground_vicon.quat_x, ground_vicon.quat_y, ground_vicon.quat_z)
                end

                sleep(rate)
                GC.gc(false)
            end
        catch e
            close(ctx)

            if e isa InterruptException
                println("Process terminated by you")
            else
                rethrow(e)
            end
        end
    end

    # Launch IMU publisher
    function main(; debug=false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        ground_vicon_ip = setup_dict["zmq"]["ground"]["vicon"]["server"]
        ground_vicon_port = setup_dict["zmq"]["ground"]["vicon"]["port"]
        jetson_vicon_ip = setup_dict["zmq"]["jetson"]["vicon"]["server"]
        jetson_vicon_port = setup_dict["zmq"]["jetson"]["vicon"]["port"]

        # Launch the relay to send the Vicon data through the telemetry radio
        link_sub() = quad_link(ground_vicon_ip, ground_vicon_port,
                               jetson_vicon_ip, jetson_vicon_port;
                               debug=debug)
        return Threads.@spawn link_sub()
    end
end