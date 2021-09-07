struct FilterNode <: Node
    data::NodeData
    imu_msg::IMU
    vicon_msg::VICON
    state_msg::FILTERED_STATE
    est_state::ImuState
    est_cov::Matrix{Float64}
    process_cov::Matrix{Float64}
    measure_cov::Matrix{Float64}
    imu_time::Float64  # seconds
    vicon_time::Float64  # seconds
    ekf::ErrorStateFilter{ImuState, ImuError, ImuInput, Vicon, ViconError}
end

function FilterNode(ctx = ZMQ.Context(1))
    setup = get_node_setup()
    data = NodeData()
    data.frequency = 200  # Hz

    imu_subscriber = PubSubBuilder.Subscriber(ctx, setup["zmq"]["jetson"]["imu"]["server"], 
                                                   setup["zmq"]["jetson"]["imu"]["port"])
    imu_msg = IMU(acc_x=0., acc_y=0., acc_z=0.,
                gyr_x=0., gyr_y=0., gyr_z=0.,
                time=0.)
    add_subscriber!(data, imu_msg, imu_subscriber)


    vicon_subscriber = PubSubBuilder.Subscriber(ctx, setup["zmq"]["jetson"]["vicon"]["server"], 
                                                     setup["zmq"]["jetson"]["vicon"]["port"])
    vicon_msg = VICON(pos_x=0., pos_y=0., pos_z=0.,
                    quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                    time=0.)
    add_subscriber!(data, vicon_msg, vicon_subscriber)
    
    
    state_publisher = PubSubBuilder.Publisher(ctx, setup["zmq"]["jetson"]["filtered_state"]["server"],
                                                   setup["zmq"]["jetson"]["filtered_state"]["port"])
    state_msg = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                            quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                            vel_x=0., vel_y=0., vel_z=0.,
                            ang_x=0., ang_y=0., ang_z=0.,
                            time=0.)
    add_publisher!(data, state_msg, state_publisher)

    # Setup the EKF filter
    est_state = ImuState(rand(3)..., params(ones(UnitQuaternion))..., rand(9)...)
    est_cov = Matrix(2.2 * I(length(ImuError)))
    process_cov = Matrix(2.2 * I(length(ImuError)))
    measure_cov = Matrix(2.2 * I(length(ViconError)))

    ekf = ErrorStateFilter{ImuState, ImuError, ImuInput, Vicon, ViconError}(est_state, est_cov,
                                                                            process_cov, measure_cov)

    imu_time = 0.0
    vicon_time = 0.0

    FilterNode(data, imu_msg, vicon_msg, state_msg, est_state, est_cov, process_cov, 
        measure_cov, imu_time, vicon_time, ekf)
end

function compute(node::FilterNode)
    ekf = node.ekf
    state = node.state_msg

    imu_msg_time = 0.0
    imu_lock = getdata(node).sub_locks[1]
    input = ImuInput{Float64}(0,0,0, 0,0,0)
    lock(imu_lock) do 
        imu_msg_time = node.imu_msg.time
        input.v̇𝑥 = node.imu_msg.acc_x
        input.v̇𝑦 = node.imu_msg.acc_y
        input.v̇𝑧 = node.imu_msg.acc_z
        input.ω𝑥 = node.imu_msg.gyr_x
        input.ω𝑦 = node.imu_msg.gyr_y
        input.ω𝑧 = node.imu_msg.gyr_z
    end

    has_new_imu_message = imu_msg_time > node.imu_time
    if has_new_imu_message
        dt = node.imu_msg.time - node.imu_time
        node.imu_time = imu_msg_time 

        # Propagate the dynamics with the inputs from the IMU 
        prediction!(ekf, input, dt)

        # Handle the vicon measurement
        vicon_msg_time = 0.0
        vicon_lock = getdata(node).sub_locks[1]
        measurement = Vicon{Float64}(0, 0, 0, 0, 0, 0, 0) 
        lock(vicon_lock) do
            vicon_msg_time = node.vicon_msg.time
            measurement.p𝑥 = node.vicon_msg.pos_x
            measurement.p𝑦 = node.vicon_msg.pos_y
            measurement.p𝑧 = node.vicon_msg.pos_z
            measurement.q𝑤 = node.vicon_msg.quat_w
            measurement.q𝑥 = node.vicon_msg.quat_x
            measurement.q𝑦 = node.vicon_msg.quat_y
            measurement.q𝑧 = node.vicon_msg.quat_z
        end
        has_new_vicon_msg = vicon_msg_time > node.vicon_time
        if has_new_vicon_msg
            node.vicon_time = vicon_msg_time

            # Update the EKF with the new measurement
            update!(ekf, measurement)

            # Update the state message and publish
            _, ω = getComponents(input)
            p, q, v, α, β = getComponents(ImuState(ekf.est_state))
            state.pos_x, state.pos_y, state.pos_z = p
            state.quat_w, state.quat_x, state.quat_y, state.quat_z = params(q)
            state.vel_x, state.vel_y, state.vel_z = v
                    state.ang_x, state.ang_y, state.ang_z = ω - β
                    state.time = time()
            state_publisher = node.data.publishers[1].pub
            publish(state_publisher, state)
        end
    end
end