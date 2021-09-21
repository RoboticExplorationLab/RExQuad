#
module JetsonLink
    import Mercury as Hg
    using ZMQ
    using TOML

    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/motors_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/quad_info_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/ground_info_msg_pb.jl")


    mutable struct JetsonLinkNode <: Hg.Node
        # Required by Abstract Node type
        nodeio::Hg.NodeIO
        rate::Float64
        should_finish::Bool

        # Specific to GroundLinkNode
        # ProtoBuf Messages
        ground_info::GROUND_INFO
        state::FILTERED_STATE
        motors::MOTORS
        vicon::VICON
        quad_info::QUAD_INFO

        # Random
        debug::Bool

        function JetsonLinkNode(
                                ground_info_sub_ip::String, ground_info_sub_port::String,
                                state_sub_ip::String, state_sub_port::String,
                                motors_sub_ip::String, motors_sub_port::String,
                                vicon_sub_ip::String, vicon_sub_port::String,
                                quad_info_pub_ip::String, quad_info_pub_port::String,
                                freq::Float64, debug::Bool)
            # Adding the Ground Vicon Subscriber to the Node
            jetsonLinkNodeIO = Hg.NodeIO(ZMQ.Context(1))
            should_finish = false

            ground_info = GROUND_INFO(deadman=true, time=0.)
            ground_info_sub = Hg.ZmqSubscriber(jetsonLinkNodeIO.ctx, ground_info_sub_ip, ground_info_sub_port)
            Hg.add_subscriber!(jetsonLinkNodeIO, ground_info, ground_info_sub)

            state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                                   quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                                   vel_x=0., vel_y=0., vel_z=0.,
                                   ang_x=0., ang_y=0., ang_z=0.,
                                   time=0.)
            state_sub = Hg.ZmqSubscriber(jetsonLinkNodeIO.ctx, state_sub_ip, state_sub_port)
            Hg.add_subscriber!(jetsonLinkNodeIO, state, state_sub)

            motors = MOTORS(front_left=0., front_right=0., back_right=0., back_left=0.,
                            time=0.)
            motors_sub = Hg.ZmqSubscriber(jetsonLinkNodeIO.ctx, motors_sub_ip, motors_sub_port)
            Hg.add_subscriber!(jetsonLinkNodeIO, motors, motors_sub)

            vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                          quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                          time=0.)
            vicon_sub = Hg.ZmqSubscriber(jetsonLinkNodeIO.ctx, vicon_sub_ip, vicon_sub_port)
            Hg.add_subscriber!(jetsonLinkNodeIO, vicon, vicon_sub)

            # Adding the Quad Info Subscriber to the Node
            quad_info = QUAD_INFO(state=state, input=motors, measurement=vicon, time=0.)
            quad_info_sub = Hg.ZmqPublisher(jetsonLinkNodeIO.ctx, quad_info_pub_ip, quad_info_pub_port)
            Hg.add_publisher!(jetsonLinkNodeIO, quad_info, quad_info_sub)

            return new(jetsonLinkNodeIO, rate, should_finish,
                       ground_info, state, motors, vicon, quad_info,
                       debug)
        end
    end

    function Hg.compute(node::JetsonLinkNode)
        jetsonLinkNodeIO = Hg.getIO(node)

        node.quad_info.time = time()

        # Publish on all topics in NodeIO
        Hg.publish.(jetsonLinkNodeIO.pubs)
    end

    # Launch IMU publisher
    function main(; rate=100.0, debug=false)::Task
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        ground_info_ip = setup_dict["zmq"]["ground"]["ground_info"]["server"]
        ground_info_port = setup_dict["zmq"]["ground"]["ground_info"]["port"]

        filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]

        motors_ip = setup_dict["zmq"]["jetson"]["motors"]["server"]
        motors_port = setup_dict["zmq"]["jetson"]["motors"]["port"]

        vicon_ip = setup_dict["zmq"]["jetson"]["vicon"]["server"]
        vicon_port = setup_dict["zmq"]["jetson"]["vicon"]["port"]

        quad_info_ip = setup_dict["zmq"]["jetson"]["quad_info"]["server"]
        quad_info_port = setup_dict["zmq"]["jetson"]["quad_info"]["port"]

        node = JetsonLinkNode(ground_info_ip, ground_info_port,
                              filtered_state_ip, filtered_state_port,
                              motors_ip, motors_port,
                              vicon_ip, vicon_port,
                              quad_info_ip, quad_info_port,
                              rate, debug)
        return node
    end
end

# %%
import Mercury as Hg
filter_node = JetsonLink.main(; rate=100.0, debug=true);
