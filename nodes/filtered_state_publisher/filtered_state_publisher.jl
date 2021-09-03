# This node is run of the Jetson, acts as the ZMQ publisher for the IMU and Vicon
# data coming through the telemetry radio and the Arduino.

module FilteredStatePublisher
    using TOML
    using ZMQ
    using ProtoBuf
    using EKF
    using LinearAlgebra: I

    include("$(@__DIR__)/../utils/PubSubBuilder.jl")
    using .PubSubBuilder

    include("$(@__DIR__)/imu_states.jl")

    include("$(@__DIR__)/../../msgs/imu_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")


    function filtered_state_publisher(imu_sub_ip::String, imu_sub_port::String,
                                      vicon_sub_ip::String, vicon_sub_port::String,
                                      filtered_state_pub_ip::String, filtered_state_pub_port::String;
                                      freq::Int64=200, debug::Bool=false)
        rate = 1 / freq
        ctx = Context(1)

        # Initalize Subscriber threads
        imu = IMU(acc_x=0., acc_y=0., acc_z=0.,
                  gyr_x=0., gyr_y=0., gyr_z=0.,
                  time=0.)
        imu_sub() = subscriber_thread(ctx, imu, imu_sub_ip, imu_sub_port)

        vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                      quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                      time=0.)
        vicon_sub() = subscriber_thread(ctx, vicon, vicon_sub_ip, vicon_sub_port)

        # Setup and Schedule Subscriber Tasks
        imu_thread = Task(imu_sub)
        schedule(imu_thread)
        vicon_thread = Task(vicon_sub)
        schedule(vicon_thread)

        # Setup Filtered state publisher
        state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                               quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                               vel_x=0., vel_y=0., vel_z=0.,
                               ang_x=0., ang_y=0., ang_z=0.,
                               time=0.)
        state_pub = create_pub(ctx, filtered_state_pub_ip, filtered_state_pub_port)
        iob = IOBuffer()

        # Setup the EKF filter
        est_state = ImuState(rand(3)..., params(ones(UnitQuaternion))..., rand(9)...)
        est_cov = Matrix(2.2 * I(length(ImuError)))
        process_cov = Matrix(2.2 * I(length(ImuError)))
        measure_cov = Matrix(2.2 * I(length(ViconError)))

        ekf = ErrorStateFilter{ImuState, ImuError, ImuInput, Vicon, ViconError}(est_state, est_cov,
                                                                                process_cov, measure_cov)
        vicon_time = 0.
        imu_time = 0.
        filtering = false

        try
            while true
                # Prediction
                if imu.time > imu_time
                    dt = imu.time - imu_time

                    input = ImuInput(imu.acc_x, imu.acc_y, imu.acc_z,
                                     imu.gyr_x, imu.gyr_y, imu.gyr_z)

                    prediction!(ekf, input, dt=dt)

                    imu_time = imu.time
                end

                # Update & Publish
                if vicon.time > vicon_time
                    measurement = Vicon(vicon.pos_x, vicon.pos_y, vicon.pos_z,
                                        vicon.quat_w, vicon.quat_x, vicon.quat_y, vicon.quat_z)

                    update!(ekf, measurement)

                    vicon_time = vicon.time

                    ., ω = getComponents(input)
                    p, q, v, α, β = getComponents(ekf.est_state)
                    state.pos_x, state.pos_y, state.pos_z = p
                    state.quat_w, state.quat_x, state.quat_y, state.quat_z = params(q)
                    state.vel_x, state.vel_y, state.vel_z = v
                    state.ang_x, state.ang_y, state.ang_z = ω - β
                    state.time = time()

                    publish(state_pub, state, iob)
                end

                sleep(rate)
                GC.gc(false)
            end
        catch e
            close(state_pub)
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

        imu_ip = setup_dict["zmq"]["jetson"]["imu"]["server"]
        imu_port = setup_dict["zmq"]["jetson"]["imu"]["port"]
        vicon_ip = setup_dict["zmq"]["jetson"]["vicon"]["server"]
        vicon_port = setup_dict["zmq"]["jetson"]["vicon"]["port"]
        filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]

        fs_pub() = filtered_state_publisher(imu_ip, imu_port,
                                            vicon_ip, vicon_port,
                                            filtered_state_ip, filtered_state_port;
                                            freq=200, debug=debug)
        return Threads.@spawn fs_pub()
    end
end