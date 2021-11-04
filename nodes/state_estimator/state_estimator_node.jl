# This node is run of the Jetson, acts as the ZMQ publisher for the IMU and Vicon
# data coming through the telemetry radio and the Arduino.
module StateEstimatorDebug
    import Mercury as Hg
    import ZMQ
    using StaticArrays
    import EKF
    import EKF.CommonSystems as ComSys
    using Rotations: UnitQuaternion, rotation_error, CayleyMap,  params
    using LinearAlgebra: I
    using Printf
    using TOML

    include("$(@__DIR__)/serial_relay_start.jl")
    import .SerialRelayStart

    # Import Protobuf Messages
    include("$(@__DIR__)/../../msgs/imu_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
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
        imu_vicon_last::IMU_VICON_C
        imu_vicon_buf::Vector{UInt8}
        last_imu_time::Float64
        last_vicon_time::UInt32

        # ProtoBuf Messages
        state::FILTERED_STATE

        # Serial Relay
        imu_vicon_serial_relay::Hg.SerialZmqRelay

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

            imu_vicon_last = IMU_VICON_C(zeros(Cfloat, 9)..., 1., zeros(Cfloat, 4)...)
            imu_vicon_buf = reinterpret(UInt8, [IMU_VICON_C(zeros(Cfloat, 9)..., 1., zeros(Cfloat, 4)...)])
            imu_vicon_sub = Hg.ZmqSubscriber(filterNodeIO.ctx,
                                             imu_vicon_sub_ip,
                                             imu_vicon_sub_port;
                                             name="IMU_VICON_SUB")
            Hg.add_subscriber!(filterNodeIO, imu_vicon_buf, imu_vicon_sub)

            last_imu_time = time()
            last_vicon_time = zero(UInt32)

            # Adding the Quad Info Subscriber to the Node
            state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                                   quat_w=1., quat_x=0., quat_y=0., quat_z=0.,
                                   vel_x=0., vel_y=0., vel_z=0.,
                                   ang_x=0., ang_y=0., ang_z=0.,
                                   time=0.)
            state_pub = Hg.ZmqPublisher(filterNodeIO.ctx, state_pub_ip, state_pub_port;
                                        name="FILTERED_STATE_PUB")
            Hg.add_publisher!(filterNodeIO,
                              state,
                              state_pub)

            # Setup Serial Relay
            imu_vicon_serial_relay = SerialRelayStart.main()

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
                       imu_vicon_last, imu_vicon_buf, last_imu_time, last_vicon_time,
                       state,
                       imu_vicon_serial_relay,
                       input, measurement, ekf,
                       debug
                       )
        end
    end

    # Wait until weve heard from the subscriber once
    function Hg.startup(node::StateEsitmatorNode)
        received_first = false
        filterIO = Hg.getIO(node)
        imu_vicon_sub = Hg.getsubscriber(node, "IMU_VICON_SUB")

        while (!received_first)
            Hg.poll_subscribers(node)

            Hg.on_new(imu_vicon_sub) do imu_vicon_buf
                node.imu_vicon_last = reinterpret(IMU_VICON_C, imu_vicon_buf)[1]
                received_first = true
            end
        end
    end

    function Hg.compute(node::StateEsitmatorNode)
        filterIO = Hg.getIO(node)
        # Make sure serial relay is still healthy
        Hg.check_relay_running(node.imu_vicon_serial_relay)
        # Get the serial relay output subscriber
        imu_vicon_sub = Hg.getsubscriber(node, "IMU_VICON_SUB")

        Hg.on_new(imu_vicon_sub) do imu_vicon_buf
            # Convert the buffer of data to IMU_VICON_C type
            imu_vicon = reinterpret(IMU_VICON_C, imu_vicon_buf)[1]

            outlier_check = not_outlier(node.imu_vicon_last, imu_vicon; debug=true)

            if outlier_check
                # Update the time for the dynamics time step
                dt = time() - node.last_imu_time
                node.last_imu_time = time()

                # If we got a new imu message run predicition step
                imu = SA[imu_vicon.acc_x, imu_vicon.acc_y, imu_vicon.acc_z,
                        imu_vicon.gyr_x, imu_vicon.gyr_y, imu_vicon.gyr_z]
                node.input = ComSys.ImuInput(imu)

                EKF.prediction!(node.ekf, node.input, dt)

                vicon = SA[imu_vicon.pos_x, imu_vicon.pos_y, imu_vicon.pos_z,
                        imu_vicon.quat_w, imu_vicon.quat_x, imu_vicon.quat_y, imu_vicon.quat_z]
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

                node.imu_vicon_last = imu_vicon
            end

            if (node.debug)
                @printf("Acceleration: \t[%1.3f, %1.3f, %1.3f]\n",
                        node.input[1:3]...)
                @printf("Rotational Vel: \t[%1.3f, %1.3f, %1.3f]\n",
                        node.input[4:6]...)
                @printf("Vicon Position: \t[%1.3f, %1.3f, %1.3f]\n",
                        node.measurement[1:3]...)
                @printf("Vicon Orientation: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n",
                        node.measurement[4:7]...)
                @printf("Position: \t[%1.3f, %1.3f, %1.3f]\n",
                        node.state.pos_x, node.state.pos_y, node.state.pos_z)
                @printf("Orientation: \t[%1.3f, %1.3f, %1.3f, %1.3f]\n\n",
                        node.state.quat_w, node.state.quat_x, node.state.quat_y, node.state.quat_z)
            end
        end

        Hg.publish.(filterIO.pubs)
    end

    function not_outlier(imu_vicon_last::IMU_VICON_C, imu_vicon::IMU_VICON_C; debug::Bool=false, )::Bool
        acc = SA[imu_vicon.acc_x, imu_vicon.acc_y, imu_vicon.acc_z]
        gyr = SA[imu_vicon.gyr_x, imu_vicon.gyr_y, imu_vicon.gyr_z]
        pos = SA[imu_vicon.pos_x, imu_vicon.pos_y, imu_vicon.pos_z]
        quat = SA[imu_vicon.quat_w, imu_vicon.quat_x,
                  imu_vicon.quat_y, imu_vicon.quat_z]
        pos_last = SA[imu_vicon_last.pos_x, imu_vicon_last.pos_y, imu_vicon_last.pos_z]
        quat_last = SA[imu_vicon_last.quat_w, imu_vicon_last.quat_x,
                       imu_vicon_last.quat_y, imu_vicon_last.quat_z]

        # Check for nan's
        for field in fieldnames(IMU_VICON_C)
            last_field = getfield(imu_vicon_last, field)
            curr_field = getfield(imu_vicon, field)
            (isnan(last_field) || isnan(curr_field)) && return false
        end

        # Check for too large of imu accelerations (||a|| > 5g's)
        acc_mag = sqrt(acc' * acc)
        if (acc_mag > 50)
            debug && (@warn "Outlier accleration magnitude: $acc_mag")
            return false
        end
        # Check for too fast of imu rotation (||ω|| > 1 RPS)
        gyr_mag = sqrt(gyr' * gyr)/(2π)
        if (gyr_mag > 1)
            debug && (@warn "Outlier rotational vel magnitude: $gyr_mag")
            return false
        end
        # Check for a vicon position outside the arena (||p|| > 10m)
        pos_mag = sqrt(pos' * pos)
        if (pos_mag > 10)
            debug && (@warn "Position outside arena: $pos_mag")
            return false
        end

        # Check for big jump in vicon position (||pₖ₋₁ - pₖ|| < 10 cm)
        pos_change = sqrt(sum((pos - pos_last).^2))
        if (pos_change > .1)
            debug && (@warn "Outlier jump in position: $pos_change")
            return false
        end

        # Check for unit quaternion violation
        unit_quat_err = abs(quat' * quat - 1)
        if (unit_quat_err > .001)
            debug && (@warn "Quaternion non-unit: $unit_quat_err")
            return false
        end

        # Check for large jump in orientation ( > π/3)
        rot_err = rotation_error(UnitQuaternion(quat, false),
                                 UnitQuaternion(quat_last, false),
                                 CayleyMap())
        rot_err_mag = sqrt(rot_err' * rot_err)
        if (rot_err_mag > π/3)
            debug && (@warn "Outlier jump in orientation: $rot_err_mag")
            return false
        end

        return true
    end

    # Make sure to kill the serial relay process
    function Hg.finishup(node::StateEsitmatorNode)
        close(node.imu_vicon_serial_relay)
    end

    # Launch IMU publisher
    function main(; rate=100.0, debug=false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        imu_serial_ipaddr = setup_dict["zmq"]["jetson"]["imu_vicon_relay"]["out"]["server"]
        imu_serial_port = setup_dict["zmq"]["jetson"]["imu_vicon_relay"]["out"]["port"]

        filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]

        node = StateEsitmatorNode(imu_serial_ipaddr, imu_serial_port,
                                  filtered_state_ip, filtered_state_port,
                                  rate, debug)

        return node
    end
end

# %% For Testing
@warn "Testing code is uncommented"
import Mercury as Hg

filter_node = StateEstimatorDebug.main(; rate=1.0, debug=true);

# %%
filter_node_task = Threads.@spawn Hg.launch(filter_node)

# # %%
# Hg.stopnode(filter_node; timeout = 1.0)