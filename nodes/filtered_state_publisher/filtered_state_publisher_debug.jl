# This node is run of the Jetson, acts as the ZMQ publisher for the IMU and Vicon
# data coming through the telemetry radio and the Arduino.
module FilteredStatePublisherDebug
    import Mercury as Hg
    using ZMQ
    using EKF
    using StaticArrays
    using SparseArrays
    using ForwardDiff: jacobian
    using Rotations: UnitQuaternion, RotationError, CayleyMap, add_error
    using Rotations: rotation_error, params, ∇differential, kinematics, RotZ
    using Printf
    using TOML

    # Import Protobuf Messages
    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")
    # EKF helper functions
    include("$(@__DIR__)/imu_states.jl")


    function filtered_state_publisher(state_pub_ip::String, state_pub_port::String;
                                      freq::Int64=200, debug::Bool=false)
        ctx = Context(1)
        state_pub = Hg.Publisher(ctx, state_pub_ip, state_pub_port)

        lrl = Hg.LoopRateLimiter(freq)

        # Setup Filtered state publisher
        state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                               quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                               vel_x=0., vel_y=0., vel_z=0.,
                               ang_x=0., ang_y=0., ang_z=0.)

        tinit = time()
        try
            Hg.@rate while true
                t = time() - tinit
                state.pos_x, state.pos_y, state.pos_z = [sin(t), cos(t), 2]
                state.quat_w, state.quat_x, state.quat_y, state.quat_z = params(UnitQuaternion(RotZ(90)))
                state.vel_x, state.vel_y, state.vel_z = rand(3)
                state.ang_x, state.ang_y, state.ang_z = rand(3)

                if debug
                    @printf("Filtered position: \t[%1.3f, %1.3f, %1.3f]\n",
                            state.pos_x, state.pos_y, state.pos_z)
                    @printf("Filtered orientation: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
                            state.quat_w, state.quat_x, state.quat_y, state.quat_z)
                end

                Hg.publish(state_pub, state)
            end lrl
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

        fs_pub() = filtered_state_publisher(filtered_state_ip, filtered_state_port;
                                            freq=100, debug=true)
        return Threads.@spawn fs_pub()
    end
end
