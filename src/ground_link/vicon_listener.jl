# Node for communicating with the Jetson (run on the ground station)
module ViconListener
    using ..RExQuad

    import Mercury as Hg
    using ZMQ
    using StaticArrays
    using Printf
    using TOML

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

    mutable struct ViconListenerNode <: Hg.Node
        # Required by Abstract Node type
        nodeio::Hg.NodeIO
        # Serialized Vicon message buffer
        serialized_vicon_buf::Vector{UInt8}
        # Random
        start_time::Float64
        end_time::Float64
        cnt::Int64
        debug::Bool
    end

    function ViconListenerNode(rate::Float64, debug::Bool, )
        # Adding the Ground Vicon Subscriber to the Node
        viconListenerIO = Hg.NodeIO(Context(1); rate = rate)

        serialized_vicon_buf = zeros(UInt8, sizeof(SerializedVICON_C))

        start_time = time()
        end_time = time()
        cnt = 0
        debug = debug

        return ViconListenerNode(
            viconListenerIO,
            serialized_vicon_buf,
            start_time,
            end_time,
            cnt,
            debug,
        )
    end

    function Hg.setupIO!(node::ViconListenerNode, nodeio::Hg.NodeIO)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")
        vicon_ground_ip = setup_dict["zmq"]["ground"]["vicon"]["server"]
        vicon_ground_port = setup_dict["zmq"]["ground"]["vicon"]["port"]

        ground_vicon_sub = Hg.ZmqSubscriber(nodeio.ctx, vicon_ground_ip, vicon_ground_port;
                                            name="GROUND_VICON_SUB" )
        Hg.add_subscriber!(nodeio, node.serialized_vicon_buf, ground_vicon_sub)
    end

    function Hg.compute(node::ViconListenerNode)
        viconListenerIO = Hg.getIO(node)
        ground_vicon_sub = Hg.getsubscriber(node, "GROUND_VICON_SUB")

        Hg.on_new(ground_vicon_sub) do serialized_vicon_buf
            serialized_vicon_c = reinterpret(SerializedVICON_C, serialized_vicon_buf)[1]

            quat = SA[serialized_vicon_c.quaternion_w, serialized_vicon_c.quaternion_x,
                      serialized_vicon_c.quaternion_y, serialized_vicon_c.quaternion_z]
            # Check for unit quaternion violation
            unit_quat_err = abs(quat' * quat - 1)
            if (unit_quat_err > .01)
                debug && (@warn "Quaternion non-unit: $unit_quat_err $quat")
            end

            if node.debug
                node.cnt += 1
                if node.cnt % floor(viconListenerIO.opts.rate) == 0
                    node.end_time = time()

                    @printf("Quaternion: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
                            quat...)

                    node.start_time = time()
                end
            end
        end
    end
end
