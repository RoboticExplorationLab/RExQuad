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

struct SerializedVICONcpp
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

mutable struct GroundLinkNode <: Hg.Node
    # Required by Abstract Node type
    nodeio::Hg.NodeIO

    # Task listening for SerializedVicon messages
    ground_vicon_submsg::Hg.SubscribedVICON
    ground_vicon_sub_task::Task

    # Specific to GroundLinkNode
    # ProtoBuf Messages
    filtered_state::FILTERED_STATE
    motors::MOTORS
    jetson_vicon::VICON
    quad_info::QUAD_INFO
    ground_info::GROUND_INFO

    # Random
    vis::QuadVisualizer
    debug::Bool

    big_bang_time::Float32
    recieved_time::Float64
    last_jetson_vicon::Hg.SerializedVICONcpp

    function GroundLinkNode(
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
        groundLinkNodeIO = Hg.NodeIO(Context(1); rate = rate)

        ground_vicon_sub = Hg.ZmqSubscriber(
            groundLinkNodeIO.ctx,
            vicon_ground_sub_ip,
            vicon_ground_sub_port,
        )
        ground_vicon_submsg = Hg.SubscribedVICON(ground_vicon_sub)
        ground_vicon_sub_task = @async Hg.subscribe(ground_vicon_submsg)

        # Adding the Quad Info Subscriber to the Node
        filtered_state = FILTERED_STATE(
            pos_x = 0.0,
            pos_y = 0.0,
            pos_z = 0.0,
            quat_w = 0.0,
            quat_x = 0.0,
            quat_y = 0.0,
            quat_z = 0.0,
            vel_x = 0.0,
            vel_y = 0.0,
            vel_z = 0.0,
            ang_x = 0.0,
            ang_y = 0.0,
            ang_z = 0.0,
            time = 0.0,
        )
        motors = MOTORS(
            front_left = 0.0,
            front_right = 0.0,
            back_right = 0.0,
            back_left = 0.0,
            time = 0.0,
        )
        jetson_vicon = VICON(
            pos_x = 0.0,
            pos_y = 0.0,
            pos_z = 0.0,
            quat_w = 0.0,
            quat_x = 0.0,
            quat_y = 0.0,
            quat_z = 0.0,
            time = 0.0,
        )
        quad_info = QUAD_INFO(
            state = filtered_state,
            input = motors,
            measurement = jetson_vicon,
            time = 0.0,
        )
        quad_info_sub = Hg.ZmqSubscriber(
            groundLinkNodeIO.ctx,
            quad_info_sub_ip,
            quad_info_sub_port;
            name = "QUAD_INFO_SUB",
        )
        Hg.add_subscriber!(groundLinkNodeIO, quad_info, quad_info_sub)

        # Adding the Ground Info Publisher to the Node
        ground_info = GROUND_INFO(deadman = true, time = time())
        ground_info_pub =
            Hg.ZmqPublisher(groundLinkNodeIO.ctx, ground_info_pub_ip, ground_info_pub_port)
        Hg.add_publisher!(groundLinkNodeIO, ground_info, ground_info_pub)

        vis = QuadVisualizer()
        open(vis)
        add_copy!(vis)

        big_bang_time = time()
        recieved_time = time()
        last_jetson_vicon = zero(Hg.SerializedVICONcpp)

        return new(
            groundLinkNodeIO,
            ground_vicon_submsg,
            ground_vicon_sub_task,
            filtered_state,
            motors,
            jetson_vicon,
            quad_info,
            ground_info,
            vis,
            debug,
            big_bang_time,
            recieved_time,
            last_jetson_vicon,
        )
    end
end

function Hg.compute(node::GroundLinkNode)
    groundLinkNodeIO = Hg.getIO(node)

    quad_info_sub = Hg.getsubscriber(node, "QUAD_INFO_SUB")

    Hg.on_new(node.ground_vicon_submsg) do serialized_vicon
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

        if sqrt(
            sum(
                SA[
                    serialized_vicon.position_x,
                    serialized_vicon.position_y,
                    serialized_vicon.position_z,
                ] - SA[
                    node.last_jetson_vicon.position_x,
                    node.last_jetson_vicon.position_y,
                    node.last_jetson_vicon.position_z,
                ],
            ) .^ 2,
        ) > 0.01
            node.big_bang_time = serialized_vicon.time_us
            node.recieved_time = time()
            @info "recorded a big bang!"

        end

        node.last_jetson_vicon = serialized_vicon

        # node.last_jetson_vicon.position_x = serialized_vicon.position_x
        # node.last_jetson_vicon.position_y = serialized_vicon.position_y
        # node.last_jetson_vicon.position_z = serialized_vicon.position_z
        # node.last_jetson_vicon.quaternion_w = serialized_vicon.position_w
        # node.last_jetson_vicon.quaternion_x = serialized_vicon.position_x
        # node.last_jetson_vicon.quaternion_y = serialized_vicon.position_y
        # node.last_jetson_vicon.quaternion_z = serialized_vicon.position_z
    end

    Hg.on_new(quad_info_sub) do quad_info
        visualize_copy!(
            node.vis,
            SA[
                quad_info.measurement.pos_x,
                quad_info.measurement.pos_y,
                quad_info.measurement.pos_z,
                quad_info.measurement.quat_w,
                quad_info.measurement.quat_x,
                quad_info.measurement.quat_y,
                quad_info.measurement.quat_z,
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
                "Vicon pos: \t[%1.3f, %1.3f, %1.3f]\n",
                quad_info.measurement.pos_x,
                quad_info.measurement.pos_y,
                quad_info.measurement.pos_z
            )
            @printf(
                "Vicon ori: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
                quad_info.measurement.quat_w,
                quad_info.measurement.quat_x,
                quad_info.measurement.quat_y,
                quad_info.measurement.quat_z
            )
        end
    end

    Hg.publish.(groundLinkNodeIO.pubs)

    return nothing
end

# Launch IMU publisher
function main(; rate = 100.0, debug = false)
    setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

    vicon_ground_ip = setup_dict["zmq"]["ground"]["vicon"]["server"]
    vicon_ground_port = setup_dict["zmq"]["ground"]["vicon"]["port"]

    quad_info_ip = setup_dict["zmq"]["jetson"]["quad_info"]["server"]
    quad_info_port = setup_dict["zmq"]["jetson"]["quad_info"]["port"]

    ground_info_ip = setup_dict["zmq"]["ground"]["ground_info"]["server"]
    ground_info_port = setup_dict["zmq"]["ground"]["ground_info"]["port"]

    node = GroundLinkNode(
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

# %%
import Mercury as Hg

node = GroundLink.main(; rate = 200.0, debug = false);

# %%
node_task = @async Hg.launch(node)

# %%
Hg.stopnode(node)
