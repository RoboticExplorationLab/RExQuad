# This node is run of the Jetson, acts as the ZMQ publisher for the IMU and Vicon
# data coming through the telemetry radio and the Arduino.
module ImuViconPublisher
    using Revise

    import Mercury as Hg
    using ZMQ
    using StaticArrays
    using Rotations: RotXYZ
    using Printf
    using TOML

    # Import protobuf messages
    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/imu_msg_pb.jl")

    mutable struct ImuViconNode <: Hg.Node
        # Required by Abstract Node type
        nodeio::Hg.NodeIO
        rate::Float64
        should_finish::Bool

        # Specific to GroundLinkNode
        # ProtoBuf Messages
        imu::IMU
        vicon::VICON

        last_imu_time::Float64

        # Random
        debug::Bool

        function ImuViconNode(teensy_port::String, teensy_baud::Int,
                              imu_pub_ip::String, imu_pub_port::String,
                              vicon_pub_ip::String, vicon_pub_port::String,
                              rate::Float64, debug::Bool)
            # Adding the Ground Vicon Subscriber to the Node
            imuViconNodeIO = Hg.NodeIO(Context(1))
            rate = rate
            should_finish = false

            # Adding the Quad Info Subscriber to the Node
            imu = IMU(acc_x=0., acc_y=0., acc_z=0.,
                      gyr_x=0., gyr_y=0., gyr_z=0.,
                      time=0.)
            imu_sub = Hg.ZmqPublisher(imuViconNodeIO.ctx, imu_pub_ip, imu_pub_port)
            Hg.add_publisher!(imuViconNodeIO, imu, imu_sub)

            vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                          quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                          time=0.)
            vicon_sub = Hg.ZmqPublisher(imuViconNodeIO.ctx, vicon_pub_ip, vicon_pub_port)
            Hg.add_publisher!(imuViconNodeIO, vicon, vicon_sub)

            imu_vicon = IMU_VICON(imu=imu, vicon=vicon)
            # sp = LibSerialPort.SerialPort(teensy_port, teensy_baud)
            # imu_vicon_pub = Hg.SerialSubscriber(sp)
            imu_vicon_sub = Hg.SerialSubscriber(teensy_port, teensy_baud);
            Hg.add_subscriber!(imuimuViconNodeIO, imu_vicon, imu_vicon_sub)

            debug = debug

            return new(imuViconNodeIO, rate, should_finish,
                       imu, vicon,
                       debug)
        end
    end

    function Hg.compute(node::ImuViconNode)
        imuViconNodeIO = Hg.getIO(node)

        # Rotation descibing VICON to IMU attitude
        Rot_imu_body = RotXYZ(0, 0, -π/2)

        # On recieving a new IMU_VICON message
        Hg.on_new(imuViconNodeIO.subs[1]) do imu_vicon_msg
            # Rotate the IMU frame to match the VICON frame
            acc_vec = SA[node.imu_vicon.imu.acc_x, node.imu_vicon.imu.acc_y, node.imu_vicon.imu.acc_z]
            gyr_vec = SA[node.imu_vicon.imu.gyr_x, node.imu_vicon.imu.gyr_y, node.imu_vicon.imu.gyr_z]
            node.imu_vicon.imu.acc_x, node.imu_vicon.imu.acc_y, node.imu_vicon.imu.acc_z = Rot_imu_body * acc_vec
            node.imu_vicon.imu.gyr_x, node.imu_vicon.imu.gyr_y, node.imu_vicon.imu.gyr_z = Rot_imu_body * gyr_vec
            node.imu_vicon.imu.time = time()

            if debug
                @printf("IMU accel: \t[%1.3f, %1.3f, %1.3f]\n",
                        node.imu_vicon.imu.acc_x, node.imu_vicon.imu.acc_y, node.imu_vicon.imu.acc_z)
                @printf("Vicon pos: \t[%1.3f, %1.3f, %1.3f]\n",
                        node.imu_vicon.vicon.pos_x, node.imu_vicon.vicon.pos_x, node.imu_vicon.vicon.pos_x)
            end
        end
        # Publish on all topics in NodeIO
        Hg.publish.(nodeio.pubs)
    end

    # Launch IMU publisher
    function main(; rate=100.0, debug=false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        imu_serial_port = setup_dict["serial"]["ground"]["imu_arduino"]["serial_port"]
        imu_baud_rate = setup_dict["serial"]["ground"]["imu_arduino"]["baud_rate"]

        imu_ip = setup_dict["zmq"]["ground_arduino"]["imu"]["server"]
        imu_port = setup_dict["zmq"]["ground_arduino"]["imu"]["port"]

        vicon_ip = setup_dict["zmq"]["ground_arduino"]["vicon"]["server"]
        vicon_port = setup_dict["zmq"]["ground_arduino"]["vicon"]["port"]

        node = ImuViconNode(imu_serial_port, imu_baud_rate,
                            imu_ip, imu_port,
                            vicon_ip, vicon_port,
                            rate, debug)
        return node
    end
end

# %%
import Mercury as Hg

filter_node = ImuViconPublisher.main(; rate=100.0, debug=true);

# %%
filter_node_task = Threads.@spawn Hg.launch(filter_node)

# %%
Hg.closeall(filter_node)

# %%
Base.throwto(filter_node_task, InterruptException())


