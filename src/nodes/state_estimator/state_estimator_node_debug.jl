# This node is run of the Jetson, acts as the ZMQ publisher for the IMU and Vicon
# data coming through the telemetry radio and the Arduino.
module StateEstimatorDebug
    import Mercury as Hg
    import ZMQ
    using StaticArrays
    import EKF
    import EKF.CommonSystems as ComSys
    using Rotations: UnitQuaternion, params
    using LinearAlgebra: I
    using Printf
    using TOML

    # Import Protobuf Messages
    include("$(@__DIR__)/../../msgs/imu_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")

    struct IMU_VICON_C
        # IMU
        acc_x::Cfloat;
        acc_y::Cfloat;
        acc_z::Cfloat;
        gyr_x::Cfloat;
        gyr_y::Cfloat;
        gyr_z::Cfloat;
        # VICON
        pos_x::Cfloat;
        pos_y::Cfloat;
        pos_z::Cfloat;
        quat_w::Cfloat;
        quat_x::Cfloat;
        quat_y::Cfloat;
        quat_z::Cfloat;

        time::UInt32;
    end

    mutable struct StateEsitmatorNode <: Hg.Node
        # Required by Abstract Node type
        nodeio::Hg.NodeIO

        # Serial messages
        imu_vicon::IMU_VICON_C
        last_imu_time::Float64
        last_vicon_time::UInt32

        # ProtoBuf Messages
        state::FILTERED_STATE

        # EKF type
        input::ComSys.ImuInput
        measurement::ComSys.ViconMeasure
        ekf::EKF.ErrorStateFilter{ComSys.ImuState, ComSys.ImuError, ComSys.ImuInput, ComSys.ViconMeasure, ComSys.ViconError}

        # Random
        debug::Bool

        function StateEsitmatorNode(imu_vicon_sub_ip::String, imu_vicon_sub_port::String,
                                    state_pub_ip::String, state_pub_port::String,
                                    rate::Float64, debug::Bool)
            # Adding the Ground Vicon Subscriber to the Node
            filterNodeIO = Hg.NodeIO(ZMQ.Context(1); rate=rate)

            tmp = zeros(Cfloat, 14)
            tmp[10] = 1.
            imu_vicon = reinterpret(IMU_VICON_C, tmp)[1]

            last_imu_time = time()
            last_vicon_time = zero(UInt32)

            # Adding the Quad Info Subscriber to the Node
            state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                                   quat_w=1., quat_x=0., quat_y=0., quat_z=0.,
                                   vel_x=0., vel_y=0., vel_z=0.,
                                   ang_x=0., ang_y=0., ang_z=0.,
                                   time=0.)
            state_pub = Hg.ZmqPublisher(filterNodeIO.ctx, state_pub_ip, state_pub_port)
            Hg.add_publisher!(filterNodeIO, state, state_pub)

            # Setup the EKF filter
            est_state = ComSys.ImuState(rand(3)..., params(ones(UnitQuaternion))..., rand(9)...)
            est_cov = Matrix(2.2 * I(length(ComSys.ImuError)))
            process_cov = Matrix(0.5 * I(length(ComSys.ImuError)))
            measure_cov = Matrix(0.005 * I(length(ComSys.ViconError)))

            input = ComSys.ImuInput(0.,0.,0.,0.,0.,0.)
            measurement = ComSys.ViconMeasure(0.,0.,0.,1.,0.,0.,0.)
            ekf = EKF.ErrorStateFilter{
                ComSys.ImuState,
                ComSys.ImuError,
                ComSys.ImuInput,
                ComSys.ViconMeasure,
                ComSys.ViconError
            }(
                est_state,
                est_cov,
                process_cov,
                measure_cov
            )

            debug = debug

            return new(filterNodeIO,
                       imu_vicon, last_imu_time, last_vicon_time,
                       state,
                       input, measurement, ekf,
                       debug
                       )
        end
    end

    function Hg.compute(node::StateEsitmatorNode)
        filterIO = Hg.getIO(node)

        dt = time() - node.last_imu_time
        node.last_imu_time = time()

        # If we got a new imu message run predicition step
        imu = SA[node.imu_vicon.acc_x, node.imu_vicon.acc_y, node.imu_vicon.acc_z,
                    node.imu_vicon.gyr_x, node.imu_vicon.gyr_y, node.imu_vicon.gyr_z]
        node.input = ComSys.ImuInput(imu)

        EKF.prediction!(node.ekf, node.input, dt)

        vicon = SA[node.imu_vicon.pos_x, node.imu_vicon.pos_y, node.imu_vicon.pos_z,
                    node.imu_vicon.quat_w, node.imu_vicon.quat_x, node.imu_vicon.quat_y, node.imu_vicon.quat_z]
        node.measurement = ComSys.ViconMeasure(vicon)

        # If we got a new vicon message run update step
        EKF.update!(node.ekf, node.measurement)

        # Update the filters state protobuf message and send
        _, ω = ComSys.getComponents(node.input)
        p, q, v, _, β = ComSys.getComponents(ComSys.ImuState(node.ekf.est_state))

        node.state.pos_x, node.state.pos_y, node.state.pos_z = p
        node.state.quat_w, node.state.quat_x, node.state.quat_y, node.state.quat_z = params(q)
        node.state.vel_x, node.state.vel_y, node.state.vel_z = v
        node.state.ang_x, node.state.ang_y, node.state.ang_z = ω - β
        node.state.time = time()

        if (node.debug)
            @printf("Acceleration: \t[%1.3f, %1.3f, %1.3f]\n",
                    node.input[1:3]...)
            @printf("Vicon Position: \t[%1.3f, %1.3f, %1.3f]\n",
                    node.measurement[1:3]...)
            @printf("Rotational Vel: \t[%1.3f, %1.3f, %1.3f]\n",
                    node.input[4:6]...)
            @printf("Vicon Orientation: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
                    node.measurement[4:7]...)
            @printf("Position: \t[%1.3f, %1.3f, %1.3f]\n",
                    node.state.pos_x, node.state.pos_y, node.state.pos_z)
            @printf("Orientation: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
                    node.state.quat_w, node.state.quat_x, node.state.quat_y, node.state.quat_z)
        end

        Hg.publish.(filterIO.pubs)
    end

    # Launch IMU publisher
    function main(; rate=100.0, debug=false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        imu_vicon_ip = setup_dict["zmq"]["jetson"]["imu"]["server"]
        imu_vicon_port = setup_dict["zmq"]["jetson"]["imu"]["port"]

        filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]

        node = StateEsitmatorNode(imu_vicon_ip, imu_vicon_port,
                                  filtered_state_ip, filtered_state_port,
                                  rate, debug)

        return node
    end
end

# %% For Testing
@warn "Testing code is uncommented"
import Mercury as Hg

filter_node = StateEstimatorDebug.main(; rate=10.0, debug=true);

# %%
filter_node_task = Threads.@spawn Hg.launch(filter_node)

# # %%
# Hg.stopnode(filter_node; timeout = 1.0)