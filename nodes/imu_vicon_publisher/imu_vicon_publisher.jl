# This node is run of the Jetson, acts as the ZMQ publisher for the IMU and Vicon
# data coming through the telemetry radio and the Arduino.

module ImuViconPublisher
    using TOML
    using ZMQ
    using ProtoBuf
    using SerialCOBS

    include("$(@__DIR__)/../utils/PubSubBuilder.jl")
    using .PubSubBuilder

    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/imu_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")


    function imu_vicon_publisher(serial_port::String, baud_rate::Int,
                                 imu_pub_ip::String, imu_pub_port::String,
                                 vicon_pub_ip::String, vicon_pub_port::String;
                                 freq::Int64=200, debug::Bool=false)
        rate = 1 / freq

        ard = Arduino(serial_port, baud_rate);

        ctx = Context(1)
        imu = IMU(acc_x=0., acc_y=0., acc_z=0.,
                  gyr_x=0., gyr_y=0., gyr_z=0.,
                  time=time())
        imu_pub = create_pub(ctx, imu_pub_ip, imu_pub_port)

        vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                      quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                      time=time())
        vicon_pub = create_pub(ctx, vicon_pub_ip, vicon_pub_port)
        vicon_time = time()

        imu_vicon = IMU_VICON(imu=imu, vicon=vicon)

        try
            open(ard) do sp
                while true
                    if bytesavailable(ard) > 0
                        iob = IOBuffer(recieve(ard))
                        readproto(iob, imu_vicon)

                        if debug
                            println("IMU: ", imu_vicon.imu.acc_x, ", ", imu_vicon.imu.acc_y, ", ", imu_vicon.imu.acc_z)
                            println("VICON: ", imu_vicon.vicon.pos_x, ", ", imu_vicon.vicon.pos_x, ", ", imu_vicon.vicon.pos_x)
                            println()
                        end

                        if imu_vicon.vicon.time > vicon_time
                            vicon_time = imu_vicon.vicon.time
                            publish(vicon_pub, imu_vicon.vicon)
                        end
                        publish(imu_pub, imu_vicon.imu)
                    end

                    sleep(rate)
                    GC.gc(false)
                end
            end
        catch e
            close(imu_pub)
            close(vicon_pub)
            close(ctx)

            if e isa InterruptException
                println("Shutting down IMU Vicon Publisher")
            else
                rethrow(e)
            end
        end
    end

    # Launch IMU publisher
    function main(; debug=false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        imu_serial_port = setup_dict["serial"]["jetson"]["imu_arduino"]["serial_port"]
        imu_baud_rate = setup_dict["serial"]["jetson"]["imu_arduino"]["baud_rate"]
        imu_ip = setup_dict["zmq"]["jetson"]["imu"]["server"]
        imu_port = setup_dict["zmq"]["jetson"]["imu"]["port"]
        vicon_ip = setup_dict["zmq"]["jetson"]["vicon"]["server"]
        vicon_port = setup_dict["zmq"]["jetson"]["vicon"]["port"]

        imu_pub() = imu_vicon_publisher(imu_serial_port, imu_baud_rate,
                                        imu_ip, imu_port,
                                        vicon_ip, vicon_port;
                                        freq=200, debug=debug)
        return Threads.@spawn imu_pub()
    end
end