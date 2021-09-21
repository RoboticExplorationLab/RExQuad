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
        imu_vicon::IMU_VICON

        # Random
        debug::Bool

        function ImuViconNode(teensy_port::String, teensy_baud::Int,
                              imu_pub_ip::String, imu_pub_port::String,
                              vicon_pub_ip::String, vicon_pub_port::String,
                              rate::Float64, debug::Bool)
            # Adding the Ground Vicon Subscriber to the Node
            imuViconNodeIO = Hg.NodeIO(Context(1))
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

            imu2 = IMU(acc_x=0., acc_y=0., acc_z=0.,
                       gyr_x=0., gyr_y=0., gyr_z=0.,
                       time=0.)
            vicon2 = VICON(pos_x=0., pos_y=0., pos_z=0.,
                           quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                           time=0.)
            imu_vicon = IMU_VICON(imu=imu2, vicon=vicon2)
            # sp = LibSerialPort.SerialPort(teensy_port, teensy_baud)
            # imu_vicon_pub = Hg.SerialSubscriber(sp)
            imu_vicon_sub = Hg.SerialSubscriber(teensy_port, teensy_baud);
            Hg.add_subscriber!(imuViconNodeIO, imu_vicon, imu_vicon_sub)

            return new(imuViconNodeIO, rate, should_finish,
                       imu, vicon, imu_vicon,
                       debug)
        end
    end

    function Hg.compute(node::ImuViconNode)
        imuViconNodeIO = Hg.getIO(node)

        # Rotation descibing VICON to IMU attitude
        Rot_imu_body = RotXYZ(0, 0, -π/2)

        # On recieving a new IMU_VICON message
        Hg.on_new(imuViconNodeIO.subs[1]) do imu_vicon
            # Rotate the IMU frame to match the VICON frame
            acc_vec = SA[imu_vicon.imu.acc_x, imu_vicon.imu.acc_y, imu_vicon.imu.acc_z]
            gyr_vec = SA[imu_vicon.imu.gyr_x, imu_vicon.imu.gyr_y, imu_vicon.imu.gyr_z]

            node.imu.acc_x, node.imu.acc_y, node.imu.acc_z = Rot_imu_body * acc_vec
            node.imu.gyr_x, node.imu.gyr_y, node.imu.gyr_z = Rot_imu_body * gyr_vec
            node.imu.time = time()

            if node.debug
                @printf("IMU accel: \t[%1.3f, %1.3f, %1.3f]\n",
                        node.imu.acc_x, node.imu.acc_y, node.imu.acc_z)
                @printf("Vicon pos: \t[%1.3f, %1.3f, %1.3f]\n",
                        node.vicon.pos_x, node.vicon.pos_x, node.vicon.pos_x)
            end
        end

        # Publish on all topics in NodeIO
        Hg.publish.(imuViconNodeIO.pubs)
    end

    # Launch IMU publisher
    function main(; rate::Float64=100.0, debug::Bool=false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        imu_serial_port = setup_dict["serial"]["jetson"]["imu_arduino"]["serial_port"]
        imu_baud_rate = setup_dict["serial"]["jetson"]["imu_arduino"]["baud_rate"]

        imu_ip = setup_dict["zmq"]["jetson"]["imu"]["server"]
        imu_port = setup_dict["zmq"]["jetson"]["imu"]["port"]

        vicon_ip = setup_dict["zmq"]["jetson"]["vicon"]["server"]
        vicon_port = setup_dict["zmq"]["jetson"]["vicon"]["port"]

        imu_serial_port = "/dev/tty.usbmodem92225501"
        imu_baud_rate = 57600
        imu_ip = "127.0.0.1"
        imu_port = "5555"
        vicon_ip = "127.0.0.1"
        vicon_port = "5556"

        node = ImuViconNode(imu_serial_port, imu_baud_rate,
                            imu_ip, imu_port,
                            vicon_ip, vicon_port,
                            rate, debug)
        return node
    end
end

# %%
import Mercury as Hg

filter_node = ImuViconPublisher.main(; rate=100.0, debug=false);

# %%
filter_node_task = Hg.launch(filter_node)

# %%
filter_node_task = Threads.@spawn Hg.launch(filter_node)

# %%
Hg.closeall(filter_node)

# %%
Base.throwto(filter_node_task, InterruptException())


