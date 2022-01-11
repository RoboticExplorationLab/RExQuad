# Node for communicating with the Jetson (run on the ground station)
module GroundMaster
    using ..RExQuad

    import Mercury as Hg
    using ZMQ
    using StaticArrays
    using Printf
    using TOML

    include("$(@__DIR__)/visualizer.jl")


    struct SerializedVICON_C
        msgid::UInt8
        is_occluded::Bool
        position_scale::UInt16

        position_x::Float32
        position_y::Float32
        position_z::Float32

        quaternion_w::Float32
        quaternion_x::Float32
        quaternion_y::Float32
        quaternion_z::Float32

        time_us::UInt32
    end

    mutable struct GroundMasterNode <: Hg.Node
        # Required by Abstract Node type
        nodeio::Hg.NodeIO
        serialized_vicon_buf::Vector{UInt8}
        quad_info::QUAD_INFO # ProtoBuf Messages
        ground_info::GROUND_INFO # ProtoBuf Messages
        vis::QuadVisualizer
        debug::Bool
    end

    function GroundLinkNode(rate::Float64, debug::Bool)
        # Adding the Ground Vicon Subscriber to the Node
        groundLinkNodeIO = Hg.NodeIO(Context(1); rate = rate)
        serialized_vicon_buf = zeros(UInt8, sizeof(SerializedVICON_C))
        quad_info = RExQuad.zero_QUAD_INFO()
        ground_info = RExQuad.zero_GROUND_INFO()
        vis = QuadVisualizer()
        add_copy!(vis)

        return GroundLinkNode(
            groundLinkNodeIO,
            serialized_vicon_buf,
            quad_info,
            ground_info,
            vis,
            debug,
        )
    end

    function Hg.setupIO!(node::GroundLinkNode, nodeio::Hg.NodeIO)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        ground_vicon_ip = setup_dict["zmq"]["ground"]["vicon"]["server"]
        ground_vicon_port = setup_dict["zmq"]["ground"]["vicon"]["port"]
        ground_vicon_sub = Hg.ZmqSubscriber(nodeio.ctx, ground_vicon_ip, ground_vicon_port; name="GROUND_VICON_SUB")
        Hg.add_subscriber!(nodeio, node.serialized_vicon_buf, ground_vicon_sub)

        quad_info_ip = setup_dict["zmq"]["jetson"]["quad_info"]["server"]
        quad_info_port = setup_dict["zmq"]["jetson"]["quad_info"]["port"]
        quad_info_sub = Hg.ZmqSubscriber(nodeio.ctx, quad_info_ip, quad_info_port; name = "QUAD_INFO_SUB")
        Hg.add_subscriber!(nodeio, node.quad_info, quad_info_sub)

        ground_info_ip = setup_dict["zmq"]["ground"]["ground_info"]["server"]
        ground_info_ip = "192.168.3.152"
        ground_info_port = setup_dict["zmq"]["ground"]["ground_info"]["port"]
        ground_info_pub = Hg.ZmqPublisher(nodeio.ctx, ground_info_ip, ground_info_port)
        Hg.add_publisher!(nodeio, node.ground_info, ground_info_pub)

        open(vis)
    end

    function Hg.compute(node::GroundMasterNode)
        groundMasterNodeIO = Hg.getIO(node)
        quad_info_sub = Hg.getsubscriber(node, "QUAD_INFO_SUB")
        ground_vicon_sub = Hg.getsubscriber(node, "GROUND_VICON_SUB")

        Hg.on_new(ground_vicon_sub) do serialized_vicon_buf
            serialized_vicon = reinterpret(SerializedVICON_C, serialized_vicon_buf)[1]

            TrajOptPlots.visualize!(
                node.vis,
                SA[
                    serialized_vicon.position_x,
                    serialized_vicon.position_y,
                    serialized_vicon.position_z,
                    serialized_vicon.quaternion_w,
                    serialized_vicon.quaternion_x,
                    serialized_vicon.quaternion_y,
                    serialized_vicon.quaternion_z,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            )
        end

        Hg.on_new(quad_info_sub) do quad_info
            visualize_copy!(
                node.vis,
                SA[
                    quad_info.state.pos_x,
                    quad_info.state.pos_y,
                    quad_info.state.pos_z,
                    quad_info.state.quat_w,
                    quad_info.state.quat_x,
                    quad_info.state.quat_y,
                    quad_info.state.quat_z,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            )
            if node.debug
                @printf(
                    "Position: \t[%1.3f, %1.3f, %1.3f]\n",
                    quad_info.state.pos_x,
                    quad_info.state.pos_y,
                    quad_info.state.pos_z
                )
                @printf(
                    "Orientation: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
                    quad_info.state.quat_w,
                    quad_info.state.quat_x,
                    quad_info.state.quat_y,
                    quad_info.state.quat_z
                )
            end
        end

        Hg.publish.(groundMasterNodeIO.pubs)
    end
end
