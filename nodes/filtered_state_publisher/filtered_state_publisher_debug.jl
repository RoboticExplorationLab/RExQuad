# This node is run of the Jetson, acts as the ZMQ publisher for the IMU and Vicon
# data coming through the telemetry radio and the Arduino.
module FilteredStatePublisher
    using TOML
    using ZMQ
    using ProtoBuf
    using EKF
    using BenchmarkTools

    # Import pub/sub utility functions
    include("$(@__DIR__)/../utils/PubSubBuilder.jl")
    using .PubSubBuilder
    # Import Protobuf Messages
    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")
    # EKF helper functions
    include("$(@__DIR__)/imu_states.jl")


    function filtered_state_publisher(imu_sub_ip::String, imu_sub_port::String,
                                      vicon_sub_ip::String, vicon_sub_port::String,
                                      state_pub_ip::String, state_pub_port::String;
                                      freq::Int64=200, debug::Bool=false)
        rate = 1 / freq
        ctx = Context(1)

        # Setup Filtered state publisher
        state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                                        quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                                        vel_x=0., vel_y=0., vel_z=0.,
                                        ang_x=0., ang_y=0., ang_z=0.)
        state_pub = create_pub(ctx, state_pub_ip, state_pub_port)
        iob = IOBuffer()

        try
            while true
                state.pos_x, state.pos_y, state.pos_z = rand(3)
                state.quat_w, state.quat_x, state.quat_y, state.quat_z = params(rand(UnitQuaternion))
                state.vel_x, state.vel_y, state.vel_z = rand(3)
                state.ang_x, state.ang_y, state.ang_z = rand(3)

                publish(state_pub, state, iob)

                sleep(rate)
                GC.gc(false) # TODO: hopefully get rid of this
            end
        catch e
            if e isa InterruptException
                println("Shutting Down Filtered State Publisher")
            else
                rethrow(e)
            end
        finally
            close(state_pub)
            close(ctx)
        end
    end

    # Launch IMU publisher
    function main(; debug=false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        imu_ip = setup_dict["zmq"]["jetson"]["imu"]["server"]
        imu_port = setup_dict["zmq"]["jetson"]["imu"]["port"]
        vicon_ip = setup_dict["zmq"]["jetson"]["vicon"]["server"]
        vicon_port = setup_dict["zmq"]["jetson"]["vicon"]["port"]
        filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]

        fs_pub() = filtered_state_publisher(imu_ip, imu_port,
                                            vicon_ip, vicon_port,
                                            filtered_state_ip, filtered_state_port;
                                            freq=200, debug=true)
        fs_thread = Task(fs_pub)
        schedule(fs_thread)

        return fs_thread
    end
end
