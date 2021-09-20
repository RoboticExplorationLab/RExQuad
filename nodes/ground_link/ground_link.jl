# Node for communicating with the Jetson (run on the ground station)
module GroundLink
    import Mercury as Hg
    using TOML
    using ZMQ
    using ProtoBuf
    using Printf
    using StaticArrays

    include("$(@__DIR__)/visualizer.jl")

    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/motors_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/quad_info_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/ground_info_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")





    function quad_link(vicon_ground_sub_ip::String, vicon_ground_sub_port::String,
                       quad_info_sub_ip::String, quad_info_sub_port::String,
                       ground_info_pub_ip::String, ground_info_pub_port::String;
                       freq::Int64=200, debug::Bool=false)
        ctx = Context(1)
        ground_vicon_sub = Hg.ZmqSubscriber(ctx, vicon_ground_sub_ip, vicon_ground_sub_port)
        quad_info_sub = Hg.ZmqSubscriber(ctx, quad_info_sub_ip, quad_info_sub_port)
        ground_info_pub = Hg.ZmqPublisher(ctx, ground_info_pub_ip, ground_info_pub_port)

        lrl = Hg.LoopRateLimiter(freq)

        filtered_state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                                        quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                                        vel_x=0., vel_y=0., vel_z=0.,
                                        ang_x=0., ang_y=0., ang_z=0.,
                                        time=0.)
        motors = MOTORS(front_left=0., front_right=0., back_right=0., back_left=0.,
                        time=0.)
        jetson_vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                             quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                             time=0.)
        quad_info = QUAD_INFO(state=filtered_state, input=motors,
                              measurement=jetson_vicon, time=0.)
        ground_vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                             quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                             time=0.)

        vicon_sub_task = @task Hg.Subscribers.subscribe(ground_vicon_sub, ground_vicon)
        schedule(vicon_sub_task)
        quad_info_sub_task = @task Hg.Subscribers.subscribe(quad_info_sub, quad_info)
        schedule(quad_info_sub_task)

        ground_info = GROUND_INFO(deadman=true, time=time())

        # Create a thread for the 3D visualizer (showing the state estimate)
        vis = QuadVisualizer()
        open(vis); add_copy!(vis)

        try
            Hg.@rate while true
                TrajOptPlots.visualize!(vis,
                    SA[ground_vicon.pos_x, ground_vicon.pos_y, ground_vicon.pos_z,
                        ground_vicon.quat_w, ground_vicon.quat_x, ground_vicon.quat_y, ground_vicon.quat_z,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    SA[jetson_vicon.pos_x, jetson_vicon.pos_y, jetson_vicon.pos_z,
                        jetson_vicon.quat_w,jetson_vicon.quat_x, jetson_vicon.quat_y, jetson_vicon.quat_z,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0],)

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

                GC.gc(false)
            end lrl
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

        vicon_ground_ip = setup_dict["zmq"]["ground"]["vicon"]["server"]
        vicon_ground_port = setup_dict["zmq"]["ground"]["vicon"]["port"]

        quad_info_ip = setup_dict["zmq"]["jetson"]["quad_info"]["server"]
        quad_info_port = setup_dict["zmq"]["jetson"]["quad_info"]["port"]

        ground_info_ip = setup_dict["zmq"]["ground"]["ground_info"]["server"]
        ground_info_port = setup_dict["zmq"]["ground"]["ground_info"]["port"]

        # Launch the relay to send the Vicon data through the telemetry radio
        link_sub() = quad_link(vicon_ground_ip, vicon_ground_port,
                               quad_info_ip, quad_info_port,
                               ground_info_ip, ground_info_port;
                               debug=debug)
        return Threads.@spawn link_sub()
    end
end

# %%
import Mercury as Hg

GroundLink.main()