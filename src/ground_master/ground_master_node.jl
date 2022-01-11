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

        # Serialized Vicon message buffer
        serialized_vicon_buf::Vector{UInt8}

        # Specific to GroundMasterNode
        # ProtoBuf Messages
        quad_info::QUAD_INFO
        ground_info::GROUND_INFO

        # Random
        vis::QuadVisualizer
        debug::Bool

        function GroundMasterNode(
            vicon_ground_sub_ip::String,
            vicon_ground_sub_port::String,
            quad_info_sub_ip::String,
            quad_info_sub_port::String,
            ground_info_pub_ip::String,
            ground_info_pub_port::String,
            rate::Float64,
            debug::Bool,
        )
            # Adding the Ground Vicon Subscriber to the Node
            groundMasterNodeIO = Hg.NodeIO(Context(1); rate = rate)

            serialized_vicon_buf = zeros(UInt8, sizeof(SerializedVICON_C))
            ground_vicon_sub = Hg.ZmqSubscriber(
                groundMasterNodeIO.ctx,
                vicon_ground_sub_ip,
                vicon_ground_sub_port;
                name="GROUND_VICON_SUB"
            )
            Hg.add_subscriber!(groundMasterNodeIO, serialized_vicon_buf, ground_vicon_sub)

            # Adding the Quad Info Subscriber to the Node
            quad_info = RExQuad.zero_QUAD_INFO()
            quad_info_sub = Hg.ZmqSubscriber(
                groundMasterNodeIO.ctx,
                quad_info_sub_ip,
                quad_info_sub_port;
                name = "QUAD_INFO_SUB",
            )
            Hg.add_subscriber!(groundMasterNodeIO, quad_info, quad_info_sub)

            # Adding the Ground Info Publisher to the Node
            ground_info = RExQuad.zero_GROUND_INFO()
            ground_info_pub = Hg.ZmqPublisher(
                groundMasterNodeIO.ctx,
                ground_info_pub_ip,
                ground_info_pub_port
            )
            Hg.add_publisher!(groundMasterNodeIO, ground_info, ground_info_pub)

            vis = QuadVisualizer()
            open(vis)
            add_copy!(vis)

            return new(
                groundMasterNodeIO,
                serialized_vicon_buf,
                quad_info,
                ground_info,
                vis,
                debug,
            )
        end
    end

    function Hg.setupIO!(node::GroundMasterNode)

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

    # Launch IMU publisher
    function main(; rate = 100.0, debug = false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        vicon_ground_ip = setup_dict["zmq"]["ground"]["vicon"]["server"]
        vicon_ground_port = setup_dict["zmq"]["ground"]["vicon"]["port"]

        quad_info_ip = setup_dict["zmq"]["jetson"]["quad_info"]["server"]
        quad_info_port = setup_dict["zmq"]["jetson"]["quad_info"]["port"]

        ground_info_ip = setup_dict["zmq"]["ground"]["ground_info"]["server"]
        ground_info_ip = "192.168.3.152"
        ground_info_port = setup_dict["zmq"]["ground"]["ground_info"]["port"]

        node = GroundMasterNode(
            vicon_ground_ip,
            vicon_ground_port,
            quad_info_ip,
            quad_info_port,
            ground_info_ip,
            ground_info_port,
            rate,
            debug,
        )
        return node
    end
end
