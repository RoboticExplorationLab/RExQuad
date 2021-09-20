# This node is run of the Jetson, acts as the ZMQ publisher for the IMU and Vicon
# data coming through the telemetry radio and the Arduino.
module FilteredStatePublisher
    using Revise

    import Mercury as Hg
    using ZMQ
    using EKF
    using StaticArrays
    using SparseArrays
    using LinearAlgebra: I
    using ForwardDiff: jacobian
    using Rotations: UnitQuaternion, RotationError, CayleyMap, add_error
    using Rotations: rotation_error, params, ∇differential, kinematics, RotZ
    using Printf
    using TOML

    # EKF helper functions
    include("$(@__DIR__)/imu_states.jl")
    # Import Protobuf Messages
    include("$(@__DIR__)/../../msgs/imu_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")

    mutable struct FilterNode <: Hg.Node
        # Required by Abstract Node type
        nodeio::Hg.NodeIO
        rate::Float64
        should_finish::Bool

        # Specific to GroundLinkNode
        # ProtoBuf Messages
        vicon::VICON
        imu::IMU
        state::FILTERED_STATE

        # EKF type
        ekf::ErrorStateFilter{ImuState, ImuError, ImuInput, Vicon, ViconError}

        # Random
        debug::Bool

        function FilterNode(imu_sub_ip::String, imu_sub_port::String,
                            vicon_sub_ip::String, vicon_sub_port::String,
                            state_pub_ip::String, state_pub_port::String,
                            rate::Int64, debug::Bool)
            # Adding the Ground Vicon Subscriber to the Node
            filterNodeIO = Hg.NodeIO()
            rate = rate
            should_finish = false

            # Initialize the ZMQ Context
            ctx = Context(1)

            # Adding the Quad Info Subscriber to the Node
            imu = IMU(acc_x=0., acc_y=0., acc_z=0.,
                      gyr_x=0., gyr_y=0., gyr_z=0.,
                      time=0.)
            imu_sub = Hg.ZmqSubscriber(ctx, imu_sub_ip, imu_sub_port)
            Hg.add_subscriber!(groundLinkNodeIO, imu, imu_sub)

            vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                          quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                          time=0.)
            vicon_sub = Hg.ZmqSubscriber(ctx, vicon_sub_ip, vicon_sub_port)
            Hg.add_subscriber!(groundLinkNodeIO, vicon, vicon_sub)

            state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                                   quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                                   vel_x=0., vel_y=0., vel_z=0.,
                                   ang_x=0., ang_y=0., ang_z=0.,
                                   time=0.)
            state_pub = Hg.ZmqPublisher(ctx, state_pub_ip, state_pub_port)
            Hg.add_publisher!(groundLinkNodeIO, state, state_pub)

            # Setup the EKF filter
            est_state = ImuState(rand(3)..., params(ones(UnitQuaternion))..., rand(9)...)
            est_cov = Matrix(2.2 * I(length(ImuError)))
            process_cov = Matrix(0.5 * I(length(ImuError)))
            measure_cov = Matrix(0.005 * I(length(ViconError)))

            ekf = ErrorStateFilter{ImuState, ImuError, ImuInput, Vicon, ViconError}(est_state, est_cov,
                                                                                    process_cov, measure_cov)

            debug = debug

            return new(filterNodeIO, rate, should_finish,
                       vicon, imu, state,
                       ekf,
                       debug)
        end
    end


    function Hg.compute(node::GroundLinkNode)
        if node.imu.time > imu_time
            dt = imu.time - imu_time

            input = ImuInput(imu.acc_x, imu.acc_y, imu.acc_z,
                                imu.gyr_x, imu.gyr_y, imu.gyr_z)

            prediction!(ekf, input, dt)

            imu_time = imu.time

            # Update & Publish
            if vicon.time > vicon_time
                vicon_time = vicon.time

                measurement = Vicon(vicon.pos_x, vicon.pos_y, vicon.pos_z,
                                    vicon.quat_w, vicon.quat_x, vicon.quat_y, vicon.quat_z)
                update!(ekf, measurement)

                _, ω = getComponents(input)
                p, q, v, α, β = getComponents(ImuState(ekf.est_state))

                state.pos_x, state.pos_y, state.pos_z = p
                state.quat_w, state.quat_x, state.quat_y, state.quat_z = params(q)
                state.vel_x, state.vel_y, state.vel_z = v
                state.ang_x, state.ang_y, state.ang_z = ω - β
                state.time = time()

                Hg.Publishers.publish(state_pub, state)

                if (debug)
                    @printf("Position: \t[%1.3f, %1.3f, %1.3f]\n",
                            state.pos_x, state.pos_y, state.pos_z)
                    @printf("Orientation: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
                            state.quat_w, state.quat_x, state.quat_y, state.quat_z)
                end
            end
        end

        # TrajOptPlots.visualize!(node.vis,
        #     SA[node.ground_vicon.pos_x, node.ground_vicon.pos_y, node.ground_vicon.pos_z,
        #        node.ground_vicon.quat_w, node.ground_vicon.quat_x, node.ground_vicon.quat_y, node.ground_vicon.quat_z,
        #        0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #     SA[node.jetson_vicon.pos_x, node.jetson_vicon.pos_y, node.jetson_vicon.pos_z,
        #        node.jetson_vicon.quat_w, node.jetson_vicon.quat_x, node.jetson_vicon.quat_y, node.jetson_vicon.quat_z,
        #        0.0, 0.0, 0.0, 0.0, 0.0, 0.0],)

        # if (node.debug)
        #     @printf("Jetson Vicon:\n")
        #     @printf("\tPosition: \t[%1.3f, %1.3f, %1.3f]\n",
        #             node.jetson_vicon.pos_x, node.jetson_vicon.pos_y, node.jetson_vicon.pos_z)
        #     @printf("\tQuaternion: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
        #             node.jetson_vicon.quat_w, node.jetson_vicon.quat_x, node.jetson_vicon.quat_y, node.jetson_vicon.quat_z)

        #     @printf("Ground Vicon:\n")
        #     @printf("\tPosition: \t[%1.3f, %1.3f, %1.3f]\n",
        #             node.ground_vicon.pos_x, node.ground_vicon.pos_y, node.ground_vicon.pos_z)
        #     @printf("\tQuaternion: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
        #             node.ground_vicon.quat_w, node.ground_vicon.quat_x, node.ground_vicon.quat_y, node.ground_vicon.quat_z)
        # end
    end


    function filtered_state_publisher(imu_sub_ip::String, imu_sub_port::String,
                                      vicon_sub_ip::String, vicon_sub_port::String,
                                      state_pub_ip::String, state_pub_port::String;
                                      freq::Int64=200, debug::Bool=false)
        ctx = Context(1)
        imu_sub = Hg.ZmqSubscriber(ctx, imu_sub_ip, imu_sub_port)
        vicon_sub = Hg.ZmqSubscriber(ctx, vicon_sub_ip, vicon_sub_port)
        state_pub = Hg.ZmqPublisher(ctx, state_pub_ip, state_pub_port)

        lrl = Hg.LoopRateLimiter(freq)

        # Setup ProtoBuf messages
        imu = IMU(acc_x=0., acc_y=0., acc_z=0.,
                  gyr_x=0., gyr_y=0., gyr_z=0.,
                  time=0.)
        vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                      quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                      time=0.)
        state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                               quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                               vel_x=0., vel_y=0., vel_z=0.,
                               ang_x=0., ang_y=0., ang_z=0.,
                               time=0.)

        imu_sub_task = @task Hg.Subscribers.subscribe(imu_sub, imu)
        schedule(imu_sub_task)
        vicon_sub_task = @task Hg.Subscribers.subscribe(vicon_sub, vicon)
        schedule(vicon_sub_task)

        # Setup the EKF filter
        est_state = ImuState(rand(3)..., params(ones(UnitQuaternion))..., rand(9)...)
        est_cov = Matrix(2.2 * I(length(ImuError)))
        process_cov = Matrix(0.5 * I(length(ImuError)))
        measure_cov = Matrix(0.005 * I(length(ViconError)))

        ekf = ErrorStateFilter{ImuState, ImuError, ImuInput, Vicon, ViconError}(est_state, est_cov,
                                                                                process_cov, measure_cov)

        try
            Hg.@rate while true
                if imu.time > imu_time
                    dt = imu.time - imu_time

                    input = ImuInput(imu.acc_x, imu.acc_y, imu.acc_z,
                                     imu.gyr_x, imu.gyr_y, imu.gyr_z)

                    prediction!(ekf, input, dt)

                    imu_time = imu.time

                    # Update & Publish
                    if vicon.time > vicon_time
                        vicon_time = vicon.time

                        measurement = Vicon(vicon.pos_x, vicon.pos_y, vicon.pos_z,
                                            vicon.quat_w, vicon.quat_x, vicon.quat_y, vicon.quat_z)
                        update!(ekf, measurement)

                        _, ω = getComponents(input)
                        p, q, v, α, β = getComponents(ImuState(ekf.est_state))

                        state.pos_x, state.pos_y, state.pos_z = p
                        state.quat_w, state.quat_x, state.quat_y, state.quat_z = params(q)
                        state.vel_x, state.vel_y, state.vel_z = v
                        state.ang_x, state.ang_y, state.ang_z = ω - β
                        state.time = time()

                        Hg.Publishers.publish(state_pub, state)

                        if (debug)
                            @printf("Position: \t[%1.3f, %1.3f, %1.3f]\n",
                                    state.pos_x, state.pos_y, state.pos_z)
                            @printf("Orientation: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
                                    state.quat_w, state.quat_x, state.quat_y, state.quat_z)
                        end
                    end
                end

                GC.gc(false)
            end lrl
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

# %%
FilteredStatePublisher.main()