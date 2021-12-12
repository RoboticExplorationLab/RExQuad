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
        # Specific to GroundLinkNode
        # ProtoBuf Messages
        ground_info::GROUND_INFO
        state::FILTERED_STATE
        motors::MOTORS
        quad_info::QUAD_INFO
        # Random
        debug::Bool
    end

    function JetsonLinkNode(rate::Float64, debug::Bool)
        jetsonLinkNodeIO = Hg.NodeIO(ZMQ.Context(1); rate = rate)

        ground_info = RExQuad.zero_GROUND_INFO()
        state = RExQuad.zero_FILTERED_STATE()
        motors = RExQuad.zero_MOTORS()
        quad_info = QUAD_INFO(state = state, input = motors, time = 0.0)

        debug=debug

        return JetsonLinkNode(
            jetsonLinkNodeIO,
            ground_info,
            state,
            motors,
            quad_info,
            debug
        )
    end

    function Hg.setupIO!(node::JetsonLinkNode, nodeio::Hg.NodeIO)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        ground_info_ip = setup_dict["zmq"]["ground"]["ground_info"]["server"]
        ground_info_port = setup_dict["zmq"]["ground"]["ground_info"]["port"]
        ground_info_sub = Hg.ZmqSubscriber(nodeio.ctx, ground_info_ip, ground_info_port; name="GROUND_INFO_SUB")
        Hg.add_subscriber!(nodeio, node.ground_info, ground_info_sub)

        filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]
        state_sub = Hg.ZmqSubscriber(nodeio.ctx, filtered_state_ip, filtered_state_port; name="FILTERED_STATE_SUB")
        Hg.add_subscriber!(nodeio, node.state, state_sub)

        motors_ip = setup_dict["zmq"]["jetson"]["motors"]["server"]
        motors_port = setup_dict["zmq"]["jetson"]["motors"]["port"]
        motors_sub = Hg.ZmqSubscriber(nodeio.ctx, motors_ip, motors_port; name="MOTORS_SUB")
        Hg.add_subscriber!(nodeio, node.state, motors_sub)

        quad_info_ip = setup_dict["zmq"]["jetson"]["quad_info"]["server"]
        quad_info_port = setup_dict["zmq"]["jetson"]["quad_info"]["port"]
        quad_info_sub = Hg.ZmqPublisher(nodeio.ctx, quad_info_ip, quad_info_port; name="QUAD_INFO_PUB")
        Hg.add_publisher!(nodeio, node.quad_info, quad_info_sub)
    end

    function Hg.compute(node::JetsonLinkNode)
        jetsonLinkNodeIO = Hg.getIO(node)

        state_sub = Hg.getsubscriber(node, "FILTERED_STATE_SUB")
        Hg.on_new(state_sub) do state
            node.quad_info.state = state
        end

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

        Hg.publish.(jetsonLinkNodeIO.pubs)
    end
end
