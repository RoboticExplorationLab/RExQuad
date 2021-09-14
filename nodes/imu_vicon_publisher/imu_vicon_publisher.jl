# This node is run of the Jetson, acts as the ZMQ publisher for the IMU and Vicon
# data coming through the telemetry radio and the Arduino.
module ImuViconPublisher
    import Mercury as Hg
    using ZMQ
    using LibSerialPort
    using StaticArrays
    using Rotations: RotXYZ
    using Printf
    using TOML

    # Import protobuf messages
    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/imu_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")

    function imu_vicon_publisher(port_name::String, baudrate::Int,
                                 imu_pub_ip::String, imu_pub_port::String,
                                 vicon_pub_ip::String, vicon_pub_port::String;
                                 freq::Int64=200, debug::Bool=false)
        sp = LibSerialPort.open(port_name, baudrate); close(sp)
        ard_sub = Hg.SerialSubscriber(sp);

        ctx = ZMQ.Context(1)
        imu_pub = Hg.Publisher(ctx, imu_pub_ip, imu_pub_port)
        vicon_pub = Hg.Publisher(ctx, vicon_pub_ip, vicon_pub_port)

        lrl = Hg.LoopRateLimiter(freq)


        imu = IMU(acc_x=0., acc_y=0., acc_z=0.,
                  gyr_x=0., gyr_y=0., gyr_z=0.,
                  time=time())
        vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                      quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                      time=time())
        imu_vicon = IMU_VICON(imu=imu, vicon=vicon)

        Rot_imu_body = RotXYZ(0, 0, -π/2)

        open(ard_sub)
        try
            cnt = 1
            start_time = time()

            Hg.@rate while true
                if Hg.receive(ard_sub, imu_vicon)
                    acc_vec = SA[imu_vicon.imu.acc_x, imu_vicon.imu.acc_y, imu_vicon.imu.acc_z]
                    gyr_vec = SA[imu_vicon.imu.gyr_x, imu_vicon.imu.gyr_y, imu_vicon.imu.gyr_z]
                    imu_vicon.imu.acc_x, imu_vicon.imu.acc_y, imu_vicon.imu.acc_z = Rot_imu_body * acc_vec
                    imu_vicon.imu.gyr_x, imu_vicon.imu.gyr_y, imu_vicon.imu.gyr_z = Rot_imu_body * gyr_vec

                    if debug
                        @printf("IMU accel: \t[%1.3f, %1.3f, %1.3f]\n",
                                imu_vicon.imu.acc_x, imu_vicon.imu.acc_y, imu_vicon.imu.acc_z)
                        @printf("Vicon pos: \t[%1.3f, %1.3f, %1.3f]\n",
                                imu_vicon.vicon.pos_x, imu_vicon.vicon.pos_x, imu_vicon.vicon.pos_x)

                        num_cnt = 100
                        if cnt % num_cnt == 0
                            end_time = time()
                            println(num_cnt / (end_time - start_time))
                            start_time = time()
                        end
                        cnt += 1
                    end
                end

                Hg.publish(imu_pub, imu_vicon.imu)
                Hg.publish(vicon_pub, imu_vicon.vicon)
            end lrl
        finally
            close(ard_sub)
        end
    end

    # Launch IMU publisher
    function main(; debug=false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        imu_serial_port = setup_dict["serial"]["ground"]["imu_arduino"]["serial_port"]
        imu_baud_rate = setup_dict["serial"]["ground"]["imu_arduino"]["baud_rate"]

        imu_ip = setup_dict["zmq"]["ground_arduino"]["imu"]["server"]
        imu_port = setup_dict["zmq"]["ground_arduino"]["imu"]["port"]
        vicon_ip = setup_dict["zmq"]["ground_arduino"]["vicon"]["server"]
        vicon_port = setup_dict["zmq"]["ground_arduino"]["vicon"]["port"]

        imu_pub() = imu_vicon_publisher(imu_serial_port, imu_baud_rate,
                                        imu_ip, imu_port,
                                        vicon_ip, vicon_port;
                                        freq=100, debug=debug)
        return Threads.@spawn imu_pub()
    end
end