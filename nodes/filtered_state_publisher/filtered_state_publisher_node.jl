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

    mutable struct FilterNode <: Hg.Node
        # Required by Abstract Node type
        nodeio::Hg.NodeIO
        rate::Float64
        should_finish::Bool

        # Specific to GroundLinkNode
        # ProtoBuf Messages
        imu::IMU
        vicon::VICON
        state::FILTERED_STATE

        last_imu_time::Float64

        # EKF type
        ekf::ErrorStateFilter{ImuState, ImuError, ImuInput, Vicon, ViconError}

        # Random
        debug::Bool

        function FilterNode(imu_sub_ip::String, imu_sub_port::String,
                            vicon_sub_ip::String, vicon_sub_port::String,
                            state_pub_ip::String, state_pub_port::String,
                            rate::Float64, debug::Bool)
            # Adding the Ground Vicon Subscriber to the Node
            filterNodeIO = Hg.NodeIO(Context(1))
            rate = rate
            should_finish = false

            # Adding the Quad Info Subscriber to the Node
            imu = IMU(acc_x=0., acc_y=0., acc_z=0.,
                      gyr_x=0., gyr_y=0., gyr_z=0.,
                      time=0.)
            imu_sub = Hg.ZmqSubscriber(filterNodeIO.ctx, imu_sub_ip, imu_sub_port)
            Hg.add_subscriber!(filterNodeIO, imu, imu_sub)

            vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                          quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                          time=0.)
            vicon_sub = Hg.ZmqSubscriber(filterNodeIO.ctx, vicon_sub_ip, vicon_sub_port)
            Hg.add_subscriber!(filterNodeIO, vicon, vicon_sub)

            state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                                   quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                                   vel_x=0., vel_y=0., vel_z=0.,
                                   ang_x=0., ang_y=0., ang_z=0.,
                                   time=0.)
            state_pub = Hg.ZmqPublisher(filterNodeIO.ctx, state_pub_ip, state_pub_port)
            Hg.add_publisher!(filterNodeIO, state, state_pub)

            last_imu_time = imu.time

            # Setup the EKF filter
            est_state = ImuState(rand(3)..., params(ones(UnitQuaternion))..., rand(9)...)
            est_cov = Matrix(2.2 * I(length(ImuError)))
            process_cov = Matrix(0.5 * I(length(ImuError)))
            measure_cov = Matrix(0.005 * I(length(ViconError)))

            ekf = ErrorStateFilter{ImuState, ImuError, ImuInput, Vicon, ViconError}(est_state, est_cov,
                                                                                    process_cov, measure_cov)

            debug = debug

            return new(filterNodeIO, rate, should_finish,
                       imu, vicon, state, last_imu_time,
                       ekf,
                       debug)
        end
    end

    function Hg.compute(node::FilterNode)
        filterNodeIO = Hg.getIO(node)

        # On recieving a new IMU message
        Hg.on_new(filterNodeIO.subs[1]) do imu_msg
            dt = imu_msg.time - node.last_imu_time
            node.last_imu_time = imu_msg.time

            input = ImuInput(imu_msg.acc_x, imu_msg.acc_y, imu_msg.acc_z,
                             imu_msg.gyr_x, imu_msg.gyr_y, imu_msg.gyr_z)
            # Run prediciton step on EKF
            prediction!(node.ekf, input, dt)

            # On recieving a new VICON message
            Hg.on_new(filterNodeIO.subs[2]) do vicon_msg
                # Update EKF
                measurement = Vicon(vicon_msg.pos_x, vicon_msg.pos_y, vicon_msg.pos_z,
                                    vicon_msg.quat_w, vicon_msg.quat_x, vicon_msg.quat_y, vicon_msg.quat_z)
                update!(node.ekf, measurement)

                # Update the filtered state message and publish it
                _, ω = getComponents(input)
                p, q, v, α, β = getComponents(ImuState(node.ekf.est_state))

                node.state.pos_x, node.state.pos_y, node.state.pos_z = p
                node.state.quat_w, node.state.quat_x, node.state.quat_y, node.state.quat_z = params(q)
                node.state.vel_x, node.state.vel_y, node.state.vel_z = v
                node.state.ang_x, node.state.ang_y, node.state.ang_z = ω - β
                node.state.time = time()

                Hg.publish.(filterNodeIO.pubs)

                if (node.debug)
                    @printf("Position: \t[%1.3f, %1.3f, %1.3f]\n",
                            node.state.pos_x, node.state.pos_y, node.state.pos_z)
                    @printf("Orientation: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
                            node.state.quat_w, node.state.quat_x, node.state.quat_y, node.state.quat_z)
                end
            end
        end
    end

    # Launch IMU publisher
    function main(; rate=100.0, debug=false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        imu_ip = setup_dict["zmq"]["jetson"]["imu"]["server"]
        imu_port = setup_dict["zmq"]["jetson"]["imu"]["port"]

        vicon_ip = setup_dict["zmq"]["jetson"]["vicon"]["server"]
        vicon_port = setup_dict["zmq"]["jetson"]["vicon"]["port"]

        filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]

        node = FilterNode(imu_ip, imu_port,
                          vicon_ip, vicon_port,
                          filtered_state_ip, filtered_state_port,
                          rate, debug)

        return node
    end
end

# %%
import Mercury as Hg

filter_node = FilteredStatePublisher.main(; rate=100.0, debug=true);

# %%
filter_node_task = Threads.@spawn Hg.launch(filter_node)

# # %%
# Hg.closeall(filter_node)

# # %%
# Base.throwto(filter_node_task, InterruptException())


