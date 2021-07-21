# This node is run of the Jetson, acts as the ZMQ publisher for the IMU and Vicon 
# data coming through the telemetry radio and the Arduino.

module FilteredStatePublisher 
    using Pkg
    Pkg.activate(@__DIR__)

    using TOML
    using ZMQ
    using ProtoBuf
    using EKF
    using Dates

    include("$(@__DIR__)/imu_states.jl")

    include("$(@__DIR__)/../../msgs/imu_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")

    function create_sub(ctx::ZMQ.Context, sub_ip, sub_port)
        s = Socket(ctx, SUB)
        ZMQ.subscribe(s)
        ZMQ.connect(s, "tcp://$sub_ip:$sub_port")
        return s
    end

    function create_pub(ctx::ZMQ.Context, pub_ip, pub_port)
        p = Socket(ctx, PUB)
        ZMQ.bind(p, "tcp://$pub_ip:$pub_port")
        return p
    end

    function subscriber_thread(ctx::ZMQ.Context, proto_msg::ProtoBuf.ProtoType, 
                               sub_ip::String, sub_port::String)
        sub = create_sub(ctx, sub_ip, sub_port) 

        try 
            println("waiting..")
            while true 
                bin_data = recv(sub)
                io = seek(convert(IOStream, bin_data), 0)
                data = readproto(io, proto_msg)
                for n in propertynames(proto_msg)
                    if hasproperty(proto_msg,n)
                        setproperty!(proto_msg, n, getproperty(data, n))
                    end
                end
            end 
        catch e 
            close(sub)
            println(stacktrace())
            println(e)
        end 
    end 

    function imu_vicon_publisher(imu_sub_ip::String, imu_sub_port::String, 
                                 vicon_sub_ip::String, vicon_sub_port::String, 
                                 filtered_state_pub_ip::String, filtered_state_pub_port::String, 
                                 debug::Bool=false)
        ctx = Context(1)

        # Initalize Subscriber threads
        imu = IMU(acc_x=0., acc_y=0., acc_z=0.,
                  gyr_x=0., gyr_y=0., gyr_z=0.,
                  time=time())
        imu_sub() = subscriber_thread(ctx, imu, imu_sub_ip, imu_sub_port)
        
        vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                      quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                      time=time())
        vicon_sub() = subscriber_thread(ctx, vicon, vicon_sub_ip, vicon_sub_port)

        # Setup and Schedule Subscriber Tasks
        imu_thread = Task(imu_sub)
        schedule(imu_thread)

        vicon_thread = Task(vicon_sub)
        schedule(vicon_thread)

        # Setup Filtered state publisher
        filtered_state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                                        quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                                        vel_x=0., vel_y=0., vel_z=0., 
                                        ang_x=0., ang_y=0., ang_z=0.)
        filtered_state_pub = create_pub(ctx, filtered_state_pub_ip, filtered_state_pub_port)
        iob = PipeBuffer()

        # Setup the EKF filter
        est_state = zeros(ImuState); est_state.q𝑤 = 1.0
        est_cov = Matrix(2.2 * I(length(ImuErrorState)))
        measurement = zeros(Vicon); vicon.q𝑤= 1.0;
        input = zeros(ImuInput)
        process_cov = Matrix(2.2 * I(length(ImuErrorState)))
        measure_cov = Matrix(2.2 * I(length(ViconErrorMeasurement)))

        ekf = ErrorStateFilter{ImuState, ImuError, ImuInput, Vicon, ViconError}(est_state, est_cov, 
                                                                                process_cov, measure_cov) 
        vicon_time = time()
        imu_time = time()
        filtering = false

        try
            while true                
                # Prediction 
                if imu.time > imu_time 
                    dt = imu.time - imu_time

                    input[1:3] .= [imu.acc_x, imu.acc_y, imu.acc_z]
                    input[4:6] .= [imu.gyr_x, imu.gyr_y, imu.gyr_z]

                    prediction!(ekf, input, dt=dt)

                    imu_time = imu.time
                end  

                # Update 
                if imu.time > imu_time  
                    measurement[1:3] .= [vicon.pos_x, vicon.pos_y, vicon.pos_z]
                    measurement[4:end] .= [vicon.quat_w, vicon.quat_x, vicon.quat_y, vicon.quat_z]

                    update!(ekf, vicon)

                    vicon_time = vicon.time
                    filtering = true
                end  

                # Publishing
                if filtering
                    v̇, ω = getComponents(input)
                    p, q, v, α, β = getComponents(ekf.est_state)
                    filtered_state.pos_x, filtered_state.pos_y, filtered_state.pos_z = p
                    filtered_state.quat_w, filtered_state.quat_x, filtered_state.quat_y, filtered_state.quat_z = params(q)
                    filtered_state.vel_x, filtered_state.vel_y, filtered_state.vel_z = v
                    filtered_state.ang_x, filtered_state.ang_y, filtered_state.ang_z = ω - β

                    writeproto(iob, filtered_state)
                    ZMQ.send(filtered_state_pub, take!(iob))
                end
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
        imu_serial_port = setup_dict["imu_arduino"]["serial_port"]
        imu_baud_rate = setup_dict["imu_arduino"]["baud_rate"]

        imu_pub() = imu_publisher(zmq_jetson_ip, zmq_imu_port, 
                                  zmq_jetson_ip, zmq_vicon_port, 
                                  imu_serial_port, imu_baud_rate; 
                                  debug=true)
        imu_thread = Task(imu_pub)
        schedule(imu_thread)
    end
end