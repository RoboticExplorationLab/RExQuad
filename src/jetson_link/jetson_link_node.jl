#
module JetsonLink
    using ..RExQuad

    import Mercury as Hg
    using ZMQ
    using Printf
    using TOML

    mutable struct JetsonLinkNode <: Hg.Node
        # Required by Abstract Node type
        nodeio::Hg.NodeIO
        rate::Float64
        should_finish::Bool

        # Specific to GroundLinkNode
        # ProtoBuf Messages
        ground_info::GROUND_INFO
        state::FILTERED_STATE
        # motors::MOTORS
        quad_info::QUAD_INFO

        # Random
        debug::Bool

        function JetsonLinkNode(
            ground_info_sub_ip::String,
            ground_info_sub_port::String,
            state_sub_ip::String,
            state_sub_port::String,
            motors_sub_ip::String,
            motors_sub_port::String,
            quad_info_pub_ip::String,
            quad_info_pub_port::String,
            rate::Float64,
            debug::Bool,
        )
            # Adding the Ground Vicon Subscriber to the Node
            jetsonLinkNodeIO = Hg.NodeIO(ZMQ.Context(1); rate = rate)
            should_finish = false

            ground_info = GROUND_INFO(deadman = true,
                                    time = 0.0)
            ground_info_sub = Hg.ZmqSubscriber(jetsonLinkNodeIO.ctx, ground_info_sub_ip, ground_info_sub_port;
                                            name="GROUND_INFO_SUB")
            Hg.add_subscriber!(jetsonLinkNodeIO, ground_info, ground_info_sub)

            state = FILTERED_STATE(
                pos_x = 0.0, pos_y = 0.0, pos_z = 0.0,
                quat_w = 0.0, quat_x = 0.0, quat_y = 0.0, quat_z = 0.0,
                vel_x = 0.0, vel_y = 0.0, vel_z = 0.0,
                ang_x = 0.0, ang_y = 0.0, ang_z = 0.0,
                time = 0.0,
            )
            state_sub = Hg.ZmqSubscriber(jetsonLinkNodeIO.ctx, state_sub_ip, state_sub_port;
                                        name="FILTERED_STATE_SUB")
            Hg.add_subscriber!(jetsonLinkNodeIO, state, state_sub)

            # motors = MOTORS(
            #     front_left = 0.0,
            #     front_right = 0.0,
            #     back_right = 0.0,
            #     back_left = 0.0,
            #     time = 0.0,
            # )
            # motors_sub = Hg.ZmqSubscriber(jetsonLinkNodeIO.ctx, motors_sub_ip, motors_sub_port;
            #                               name="MOTORS_SUB")
            # Hg.add_subscriber!(jetsonLinkNodeIO, motors, motors_sub)

            # Adding the Quad Info Subscriber to the Node
            quad_info =
                QUAD_INFO(state = state,
                        #   input = motors,
                        time = 0.0)
            quad_info_sub = Hg.ZmqPublisher(jetsonLinkNodeIO.ctx, quad_info_pub_ip, quad_info_pub_port;
                                            name="QUAD_INFO_PUB")
            Hg.add_publisher!(jetsonLinkNodeIO, quad_info, quad_info_sub)

            return new(
                jetsonLinkNodeIO,
                rate,
                should_finish,
                ground_info,
                state,
                # motors,
                quad_info,
                debug,
            )
        end
    end

    function Hg.compute(node::JetsonLinkNode)
        jetsonLinkNodeIO = Hg.getIO(node)

        state_sub = Hg.getsubscriber(node, "FILTERED_STATE_SUB")
        Hg.on_new(state_sub) do state
            node.quad_info.state = state
        end
        # state_sub = Hg.getsubscriber(node, "FILTERED_STATE_SUB")
        # Hg.on_new(state_sub) do state
        #     node.quad_info.state = state
        # end
        # state_sub = Hg.getsubscriber(node, "FILTERED_STATE_SUB")
        # Hg.on_new(state_sub) do state
        #     node.quad_info.state = state
        # end


        node.quad_info.time = time()
        if node.debug
            @printf(
                "Position: \t[%1.3f, %1.3f, %1.3f]\n",
                node.quad_info.state.pos_x,
                node.quad_info.state.pos_y,
                node.quad_info.state.pos_z
            )
            @printf(
                "Orientation: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
                node.quad_info.state.quat_w,
                node.quad_info.state.quat_x,
                node.quad_info.state.quat_y,
                node.quad_info.state.quat_z
            )
        end

        # quad_info_pub = Hg.getpublisher(node, "QUAD_INFO_PUB")
        # Publish on all topics in NodeIO
        # Hg.publish(quad_info_pub)

        Hg.publish.(jetsonLinkNodeIO.pubs)
    end

    # Launch IMU publisher
    function main(; rate = 33.0, debug = false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        ground_info_ip = setup_dict["zmq"]["ground"]["ground_info"]["server"]
        ground_info_port = setup_dict["zmq"]["ground"]["ground_info"]["port"]

        filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]

        motors_ip = setup_dict["zmq"]["jetson"]["motors"]["server"]
        motors_port = setup_dict["zmq"]["jetson"]["motors"]["port"]

        quad_info_ip = setup_dict["zmq"]["jetson"]["quad_info"]["server"]
        # quad_info_ip = "192.168.3.117"
        quad_info_port = setup_dict["zmq"]["jetson"]["quad_info"]["port"]

        node = JetsonLinkNode(
            ground_info_ip,
            ground_info_port,
            filtered_state_ip,
            filtered_state_port,
            motors_ip,
            motors_port,
            quad_info_ip,
            quad_info_port,
            rate,
            debug,
        )
        return node
    end
end
