module ImuViconLogger

    export imu_vicon_logger

    using ZMQ
    using ZMQ: propertynames
    
    using ProtoBuf
    using DataFrames
    using CSV
    using Dates

    # Include the IMU and Vicon protocol buffer message types
    include("$(@__DIR__)/../../gen/msgs/message_pb.jl")
    Base.exit_on_sigint(false)

    function create_sub(ctx, sub_ip, sub_port)
        s = Socket(ctx, SUB)
        ZMQ.subscribe(s)
        ZMQ.connect(s, "tcp://$sub_ip:$sub_port")
        return s
    end

    """ imu_subscriber
    subcriber thread that updates a protobuf message in a thread
    """
    function imu_subscriber(ctx, sub_ip, sub_port, proto_msg)
        sub = create_sub(ctx, sub_ip, sub_port)
        try
            println("IMU subscriber waiting...")
            while true
                bin_data = recv(sub)
                io = seek(convert(IOStream, bin_data),0)
                data = readproto(io, IMU())
                for n in propertynames(proto_msg)
                    setproperty!(proto_msg, n, getproperty(data,n))
                end
            end
        catch e
            close(sub)
            println(stacktrace())
            println(e)
        end
    end


    """ vicon_subscriber
    subcriber thread that updates a protobuf message in a thread
    """
    function vicon_subscriber(ctx, sub_ip, sub_port, proto_msg)
        sub = create_sub(ctx, sub_ip, sub_port)
        try
            println("Vicon subscriber waiting...")
            while true
                bin_data = recv(sub)
                io = seek(convert(IOStream, bin_data),0)
                data = readproto(io, VICON())
                for n in propertynames(proto_msg)
                    setproperty!(proto_msg, n, getproperty(data,n))
                end
            end
        catch e
            close(sub)
            println(stacktrace())
            println(e)
        end
    end


    ### Subscribing example with ZMQ and Protobuf
    function imu_vicon_logger(zmqs_vicon_ip, zmqs_vicon_port, 
                              zmqs_imu_ip, zmqs_imu_port; debug=false)
        ctx = Context(1)
        vicon = VICON();
        imu = IMU();
        # Declare tasks
        vicon_sub() = vicon_subscriber(ctx, zmqs_vicon_ip, zmqs_vicon_port, vicon)
        imu_sub() = imu_subscriber(ctx, zmqs_imu_ip, zmqs_imu_port, imu)
        # Setup and run tasks in different threads
        vicon_thread = Task(vicon_sub)
        imu_thread = Task(imu_sub)
        schedule(vicon_thread)
        schedule(imu_thread)
        # Initialize DataFrame
        vicon_df = DataFrame(time=[],
                             pos_x=[], pos_y=[], pos_z=[],
                             quat_w=[], quat_x=[], quat_y=[], quat_z=[])
        imu_df = DataFrame(time=[],
                           acc_x=[], acc_y=[], acc_z=[],
                           gyr_x=[], gyr_y=[], gyr_z=[])

        try
            while true
                if hasproperty(vicon, :time)
                    push!(vicon_df, (vicon.time, vicon.pos_x, vicon.pos_y, vicon.pos_z,
                                     vicon.quat_w, vicon.quat_x, vicon.quat_y, vicon.quat_z))
                    debug && println([vicon.time,
                                      vicon.pos_x, vicon.pos_y, vicon.pos_z,
                                      vicon.quat_w, vicon.quat_x, vicon.quat_y, vicon.quat_z])
                end
                if hasproperty(imu, :time)
                    push!(imu_df, (imu.time, imu.acc_x, imu.acc_y, imu.acc_z,
                                   imu.gyr_x, imu.gyr_y, imu.gyr_z))
                    debug && println([imu.time,
                                      imu.acc_x, imu.acc_y, imu.acc_z,
                                      imu.gyr_x, imu.gyr_y, imu.gyr_z])
                end
                sleep(0.0001)
            end
        catch e
            println("Heard error!")

            close(ctx)
            if e isa InterruptException
                # Save DataFrames
                CSV.write("$(@__DIR__)/../../data/vicon_$(now()).csv", vicon_df)
                CSV.write("$(@__DIR__)/../../data/imu_$(now()).csv", imu_df)

                # Clean up
                println("Process terminated by you")
                Base.throwto(vicon_thread, InterruptException())
                Base.throwto(imu_thread, InterruptException())
            else
                rethrow(e)
            end
        end
    end

end
