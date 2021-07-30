#
module JetsonLink
    using TOML
    using ZMQ
    using ProtoBuf

    include("$(@__DIR__)/../utils/PubSubBuilder.jl")
    using .PubSubBuilder

    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/motors_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/quad_info_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")

    function quad_link(filtered_state_sub_ip::String, filtered_state_sub_port::String,
                       quad_info_pub_ip::String, quad_info_pub_port::String;
                       freq::Int64=200, debug::Bool=false)
        ctx = Context(1)

        filtered_state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                                        quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                                        vel_x=0., vel_y=0., vel_z=0.,
                                        ang_x=0., ang_y=0., ang_z=0.)
        filtered_state_sub() = subscriber_thread(ctx, filtered_state,
                                                 filtered_state_sub_ip,
                                                 filtered_state_sub_port)
        # Setup and Schedule Subscriber Tasks
        filtered_state_thread = Task(filtered_state_sub)
        schedule(filtered_state_thread)

        # motors = MOTORS(front_left=0., front_right=0., back_right=0., back_left=0.,
        #                 time=0.)
        # vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
        #               quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
        #               time=time())
        # quad_info = QUAD_INFO(state=filtered_state, motors=motors, vicon=vicon)
        # quad_pub = create_pub(ctx, quad_info_pub_ip, quad_info_pub_port)

        # Setup inial times
        iob = PipeBuffer()
        delay = 1 / freq

        try
            while true
                # writeproto(iob, quad_info)
                # ZMQ.send(quad_pub, take!(iob))
                sleep(delay)
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
    function main(; debug=false)::Task
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        zmq_quad_info_pub_ip = setup_dict["zmq"]["jetson"]["quad_info"]["server"]
        zmq_quad_info_pub_port = setup_dict["zmq"]["jetson"]["quad_info"]["port"]
        zmq_filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        zmq_filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]

        # Launch the relay to send the Vicon data through the telemetry radio
        link_pub() = quad_link(zmq_filtered_state_ip, zmq_filtered_state_port,
                               zmq_quad_info_pub_ip, zmq_quad_info_pub_port;
                               freq=200, debug=debug)
        link_thread = Task(link_pub)
        schedule(link_thread)

        return link_thread
    end
end