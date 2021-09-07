

abstract type Node end

startup(::Node) = throw(ErrorException("Startup hasn't been implemented on your node yet!"))
compute(::Node) = throw(ErrorException("Compute hasn't been implemented on your node yet!"))
publishers(::Node) = throw(ErrorException("publishers method hasn't been implemented on your node yet!"))
subscribers(::Node) = throw(ErrorException("subscribers method hasn't been implemented on your node yet!"))
frequency(::Node) = 200  # Hz

function run(node::Node)
    rate = 1 / frequency(node)
    startup(node)
    try
        while true
            compute(node)
            sleep(rate)
            GC.gc(false)
        end
    catch e
        for pub in publishers(node)
            close(pub)
        end
        for sub in subscribers(node)
            close(sub)
        end

        rethrow(e)
    end
end

struct FilterNode
    pubs::Vector{Publisher}
    subs::Vector{Subscriber}
    imu_msg::IMU
    vicon_msg::VICON
    state_msg::FILTERED_STATE
    est_state::ImuState
    est_cov::Matrix{Float64}
    process_cov::Matrix{Float64}
    measure_cov::Matrix{Float64}
end
publishers(node::FilterNode) = node.pubs
subscribers(node::FilterNode) = node.subs

function FilterNode(ctx = ZMQ.Context(1))
    setup = get_node_setup()
    imu_subscriber = PubSubBuilder.Subscriber(ctx, setup["zmq"]["jetson"]["imu"]["server"], 
                                                   setup["zmq"]["jetson"]["imu"]["port"])
    imu = IMU(acc_x=0., acc_y=0., acc_z=0.,
                gyr_x=0., gyr_y=0., gyr_z=0.,
                time=0.)

    vicon_subscriber = PubSubBuilder.Subscriber(ctx, setup["zmq"]["jetson"]["vicon"]["server"], 
                                                     setup["zmq"]["jetson"]["vicon"]["port"])
    vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                    quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                    time=0.)
    
    
    state_publisher = PubSubBuilder.Publisher(ctx, setup["zmq"]["jetson"]["filtered_state"]["server"],
                                                   setup["zmq"]["jetson"]["filtered_state"]["port"])
    state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                            quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                            vel_x=0., vel_y=0., vel_z=0.,
                            ang_x=0., ang_y=0., ang_z=0.,
                            time=0.)

    FilterNode([state_publisher], [imu_subscriber, vicon_subscriber])
end

function setup(node::FilterNode)
    PubSubBuilder.publish(node.pubs[1], node.state_msg)
    imu_task = @task PubSubBuilder.subscribe(node.subs[1], node.imu_msg)
    vicon_task = @task PubSubBuilder.subscribe(node.subs[2], node.vicon_msg)
    schedule(imu_task)
    schedule(vicon_task)
end

function compute(node::FilterNode)

end