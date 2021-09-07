# Node for communicating with the Jetson (run on the ground station)
module GroundLink
    using TOML
    using ZMQ
    using ProtoBuf
    using Printf

    include("$(@__DIR__)/../utils/PubSubBuilder.jl")
    include("$(@__DIR__)/../visualizer.jl")
    using .PubSubBuilder

    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/motors_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/quad_info_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/ground_info_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")


    function printQuadInfo(quad_info::QUAD_INFO)
        run(`clear`);

        println("Quad Info:")

        println("\tFiltered State:")
        @printf("\t\tPosition: [%1.3f, %1.3f, %1.3f]\n",
                quad_info.state.pos_x,
                quad_info.state.pos_y,
                quad_info.state.pos_z)
        @printf("\t\tQuaternion: [%1.3f, %1.3f, %1.3f, %1.3f]\n",
                quad_info.state.quat_w, quad_info.state.quat_x,
                quad_info.state.quat_y, quad_info.state.quat_z)
        @printf("\t\tLinear Vel: [%1.3f, %1.3f, %1.3f]\n",
                quad_info.state.vel_x,
                quad_info.state.vel_y,
                quad_info.state.vel_z)
        @printf("\t\tAngular Vel: [%1.3f, %1.3f, %1.3f]\n",
                quad_info.state.ang_x,
                quad_info.state.ang_y,
                quad_info.state.ang_z)

        println("\tMotor Command:")
        @printf("\t\tCommands: [%1.3f, %1.3f, %1.3f, %1.3f]\n",
                quad_info.input.front_left, quad_info.input.front_right,
                quad_info.input.back_right, quad_info.input.back_left)

        println("\tVicon Measurement:")
        @printf("\t\tPosition: [%1.3f, %1.3f, %1.3f]\n",
                quad_info.measurement.pos_x,
                quad_info.measurement.pos_y,
                quad_info.measurement.pos_z)
        @printf("\t\tQuaternion: [%1.3f, %1.3f, %1.3f, %1.3f]\n",
                quad_info.measurement.quat_w, quad_info.measurement.quat_x,
                quad_info.measurement.quat_y, quad_info.measurement.quat_z)
    end


    function quad_link(quad_info_sub_ip::String, quad_info_sub_port::String,
                       ground_info_sub_ip::String, ground_info_sub_port::String;
                       freq::Int64=200, debug::Bool=false)
        rate = 1 / freq
        ctx = Context(1)

        filtered_state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                                        quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                                        vel_x=0., vel_y=0., vel_z=0.,
                                        ang_x=0., ang_y=0., ang_z=0.,
                                        time=0.)
        motors = MOTORS(front_left=0., front_right=0., back_right=0., back_left=0.,
                        time=0.)
        vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                      quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                      time=0.)
        quad_info = QUAD_INFO(state=filtered_state,
                              input=motors,
                              measurement=vicon,
                              time=0.)
        quad_sub() = subscriber_thread(ctx, quad_info, quad_info_sub_ip, quad_info_sub_port)

        # Setup and Schedule Subscriber Tasks
        quad_thread = Task(quad_sub)
        schedule(quad_thread)

        # Create a thread for the 3D visualizer (showing the state estimate)
        vis = QuadVisualizer()
        open(vis)
        vis_task = @task visualize_filteredstate(vis)
        schedule(vis_task)

        quad_info_time = 0.

        ground_info = GROUND_INFO(deadman=true,
                                  time=time())
        ground_pub = create_pub(ctx, ground_info_sub_ip, ground_info_sub_port)
        iob = IOBuffer()

        try
            while true
                if quad_info.time > quad_info_time
                    # printQuadInfo(quad_info)

                    quad_info_time = quad_info.time
                end

                ground_info.deadman = true
                ground_info.time = time()
                publish(ground_pub, ground_info, iob)

                sleep(rate)
                GC.gc(false)
            end
        catch e
            close(ground_pub)
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

        quad_info_sub_ip = setup_dict["zmq"]["jetson"]["quad_info"]["server"]
        quad_info_sub_port = setup_dict["zmq"]["jetson"]["quad_info"]["port"]
        ground_info_pub_ip = setup_dict["zmq"]["ground"]["ground_info"]["server"]
        ground_info_pub_port = setup_dict["zmq"]["ground"]["ground_info"]["port"]

        # Launch the relay to send the Vicon data through the telemetry radio
        link_sub() = quad_link(quad_info_sub_ip, quad_info_sub_port,
                               ground_info_pub_ip, ground_info_pub_port;
                               debug=debug)
        return Threads.@spawn link_sub()
    end
end