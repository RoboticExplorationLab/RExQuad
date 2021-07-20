module ImuPublisher
    using Pkg
    Pkg.activate(@__DIR__)

    using TOML
    using ZMQ
    using ProtoBuf
    using SerialCOBS
    using Dates

    include("$(@__DIR__)/../../msgs/imu_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")

    function create_pub(ctx, pub_ip, pub_port)
        p = Socket(ctx, PUB)
        ZMQ.bind(p, "tcp://$pub_ip:$pub_port")
        return p
    end

    # %%
    function imu_publisher(pub_ip::String, pub_port::String, 
                           serial_port::String, baud_rate::Int; 
                           debug::Bool=false)
        ard = Arduino(serial_port, baud_rate);

        ctx = Context(1)
        pub = create_pub(ctx, pub_ip, pub_port)
        imu = IMU(acc_x=0., acc_y=0., acc_z=0.,
                    gyr_x=0., gyr_y=0., gyr_z=0.)

        imu = VICON(acc_x=0., acc_y=0., acc_z=0.,
                    gyr_x=0., gyr_y=0., gyr_z=0.)
        iob = PipeBuffer()

        try
            open(ard) do sp
                while true
                    if bytesavailable(ard) > 0
                        iob = PipeBuffer(recieve(ard))
                        try
                            readproto(iob, imu)
                        catch 
                        end

                        try
                            readproto(iob, imu)
                        catch 
                        end
                        println(imu)
                
                    end
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

    # %%
    # Launch IMU publisher
    function main()
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        zmq_imu_ip = setup_dict["zmq"]["server"]
        zmq_imu_port = setup_dict["zmq"]["imu_port"]
        imu_serial_port = setup_dict["imu_arduino"]["serial_port"]
        imu_baud_rate = setup_dict["imu_arduino"]["baud_rate"]

        imu_pub() = imu_publisher(zmq_imu_ip, zmq_imu_port, 
                                  imu_serial_port, imu_baud_rate; 
                                  debug=true)
        imu_thread = Task(imu_pub)
        schedule(imu_thread)
    end
end