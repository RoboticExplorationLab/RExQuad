# This node is run of the Jetson, acts as the ZMQ publisher for the IMU and Vicon 
# data coming through the telemetry radio and the Arduino.
module FilteredStatePublisher 
    using TOML
    using ZMQ
    using ProtoBuf
    using EKF

    include("$(@__DIR__)/imu_states.jl")

    include("$(@__DIR__)/../../msgs/imu_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")

    function filtered_state_publisher(imu_sub_ip::String, imu_sub_port::String, 
                                      vicon_sub_ip::String, vicon_sub_port::String, 
                                      filtered_state_pub_ip::String, filtered_state_pub_port::String; 
                                      debug::Bool=false)
        ctx = Context(1)

        # Setup Filtered state publisher
        filtered_state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                                        quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                                        vel_x=0., vel_y=0., vel_z=0., 
                                        ang_x=0., ang_y=0., ang_z=0.)
        filtered_state_pub = create_pub(ctx, filtered_state_pub_ip, filtered_state_pub_port)
        iob = PipeBuffer()

        try
            while true       
                v̇, ω = rand(3), rand(3)
                p, q, v, α, β = rand(3), rand(UnitQuaternion), rand(3), rand(3), rand(3)
                filtered_state.pos_x, filtered_state.pos_y, filtered_state.pos_z = p
                filtered_state.quat_w, filtered_state.quat_x, filtered_state.quat_y, filtered_state.quat_z = params(q)
                filtered_state.vel_x, filtered_state.vel_y, filtered_state.vel_z = v
                filtered_state.ang_x, filtered_state.ang_y, filtered_state.ang_z = ω - β

                # println(filtered_state)

                writeproto(iob, filtered_state)
                ZMQ.send(filtered_state_pub, take!(iob))

                sleep(.1)
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
    function main()
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        zmq_jetson_ip = setup_dict["zmq"]["jetson"]["server"]
        zmq_imu_port = setup_dict["zmq"]["jetson"]["imu_port"]
        zmq_vicon_port = setup_dict["zmq"]["jetson"]["vicon_port"]
        zmq_filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state_port"]

        filtered_state_pub() = filtered_state_publisher(zmq_jetson_ip, zmq_imu_port, 
                                                        zmq_jetson_ip, zmq_vicon_port, 
                                                        zmq_jetson_ip, zmq_filtered_state_port; 
                                                        debug=true)
        filtered_state_thread = Task(filtered_state_pub)
        schedule(filtered_state_thread)
    end
end