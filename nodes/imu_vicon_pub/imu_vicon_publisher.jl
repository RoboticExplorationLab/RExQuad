# This node is run of the Jetson, acts as the ZMQ publisher for the IMU and Vicon 
# data coming through the telemetry radio and the Arduino.

module ImuViconPublisher 
    using Pkg
    Pkg.activate(@__DIR__)

    using TOML
    using ZMQ
    using ProtoBuf
    using SerialCOBS
    using Dates

    include("$(@__DIR__)/../../msgs/imu_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")

    function create_pub(ctx, pub_ip, pub_port)
        p = Socket(ctx, PUB)
        ZMQ.bind(p, "tcp://$pub_ip:$pub_port")
        return p
    end

    function imu_vicon_publisher(imu_pub_ip::String, imu_pub_port::String, 
                                 vicon_pub_ip::String, vicon_pub_port::String, 
                                 serial_port::String, baud_rate::Int; 
                                 debug::Bool=false)
        ard = Arduino(serial_port, baud_rate);

        ctx = Context(1)
        imu_pub = create_pub(ctx, imu_pub_ip, imu_pub_port)
        imu = IMU(acc_x=0., acc_y=0., acc_z=0.,
                  gyr_x=0., gyr_y=0., gyr_z=0.,
                  time=time())
        
        vicon_pub = create_pub(ctx, vicon_pub_ip, vicon_pub_port)
        vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                      quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                      time=time())

        try
            open(ard) do sp
                while true
                    if bytesavailable(ard) > 0
                        # Get the data packet from the Arudino
                        iob = PipeBuffer(recieve(ard))

                        try # Check if its a IMU protobuf
                            readproto(iob, imu)
                            imu.time = time()

                            if (debug) println(imu.acc_x, " ", imu.acc_y, " ", imu.acc_z) end

                            writeproto(iob, imu)
                            ZMQ.send(imu_pub, take!(iob))
                        catch # Check if its a VICON protobuf
                            readproto(iob, vicon)
                            
                            if (debug) println(vicon_pub.pos_x, " ", vicon_pub.pos_y, " ", vicon_pub.pos_z) end

                            writeproto(iob, vicon)
                            ZMQ.send(vicon_pub, take!(iob))
                        finally
                            if (debug) println("Heard enroneous message from IMU/VICON Arduino.") end
                        end
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