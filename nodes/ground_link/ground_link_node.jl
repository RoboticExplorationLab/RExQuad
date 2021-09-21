# Node for communicating with the Jetson (run on the ground station)
module GroundLink
    using Revise

    import Mercury as Hg
    using ZMQ
    using Printf
    using StaticArrays
    using TOML

    include("$(@__DIR__)/visualizer.jl")

    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/motors_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/quad_info_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/ground_info_msg_pb.jl")


    mutable struct GroundLinkNode <: Hg.Node
        # Required by Abstract Node type
        nodeio::Hg.NodeIO
        rate::Float64
        should_finish::Bool

        # Specific to GroundLinkNode
        # ProtoBuf Messages
        ground_vicon::VICON
        filtered_state::FILTERED_STATE
        motors::MOTORS
        jetson_vicon::VICON
        quad_info::QUAD_INFO
        ground_info::GROUND_INFO

        # Random
        vis::QuadVisualizer
        debug::Bool

        function GroundLinkNode(vicon_ground_sub_ip::String, vicon_ground_sub_port::String,
                                quad_info_sub_ip::String, quad_info_sub_port::String,
                                ground_info_pub_ip::String, ground_info_pub_port::String,
                                rate::Float64, debug::Bool)
            # Adding the Ground Vicon Subscriber to the Node
            groundLinkNodeIO = Hg.NodeIO(Context(1))
            rate = rate
            should_finish = false

            ground_vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                                quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                                time=0.)
            ground_vicon_sub = Hg.ZmqSubscriber(groundLinkNodeIO.ctx, vicon_ground_sub_ip, vicon_ground_sub_port)
            Hg.add_subscriber!(groundLinkNodeIO, ground_vicon, ground_vicon_sub)

            # Adding the Quad Info Subscriber to the Node
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
            quad_info_sub = Hg.ZmqSubscriber(groundLinkNodeIO.ctx, quad_info_sub_ip, quad_info_sub_port)
            Hg.add_subscriber!(groundLinkNodeIO, quad_info, quad_info_sub)

            # Adding the Ground Info Publisher to the Node
            ground_info = GROUND_INFO(deadman=true, time=time())
            ground_info_pub = Hg.ZmqPublisher(groundLinkNodeIO.ctx, ground_info_pub_ip, ground_info_pub_port)
            Hg.add_publisher!(groundLinkNodeIO, ground_info, ground_info_pub)

            vis = QuadVisualizer()
            debug = debug

            return new(groundLinkNodeIO, rate, should_finish,
                       ground_vicon, filtered_state, motors, jetson_vicon, quad_info, ground_info,
                       vis, debug)
        end
    end

    function Hg.startup(node::GroundLinkNode)
        open(node.vis); add_copy!(node.vis)
    end

    function Hg.compute(node::GroundLinkNode)
        nodeio = Hg.getIO(node)

        Hg.publish.(nodeio.pubs)
    end

    # Launch IMU publisher
    function main(; rate=100.0, debug=false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        vicon_ground_ip = setup_dict["zmq"]["ground"]["vicon"]["server"]
        vicon_ground_port = setup_dict["zmq"]["ground"]["vicon"]["port"]

        quad_info_ip = setup_dict["zmq"]["jetson"]["quad_info"]["server"]
        quad_info_port = setup_dict["zmq"]["jetson"]["quad_info"]["port"]

        ground_info_ip = setup_dict["zmq"]["ground"]["ground_info"]["server"]
        ground_info_port = setup_dict["zmq"]["ground"]["ground_info"]["port"]

        node = GroundLinkNode(vicon_ground_ip, vicon_ground_port,
                              quad_info_ip, quad_info_port,
                              ground_info_ip, ground_info_port,
                              rate, debug)

        return node
    end
end

# # %%
# import Mercury as Hg

# node = GroundLink.main();

# # %%
# Hg.launch(node)

# # %%
# if all([isopen(submsg.sub) for submsg in node.nodeio.subs])
#     Hg.launch(node)
# else
#     Hg.closeall(node)
# end

# # %%
# Hg.closeall(node)
