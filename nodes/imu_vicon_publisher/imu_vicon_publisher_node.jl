# This node is run of the Jetson, acts as the ZMQ publisher for the IMU and Vicon
# data coming through the telemetry radio and the Arduino.
module ImuViconPublisher
    import Mercury as Hg
    using ZMQ
    using StaticArrays
    using Printf
    using TOML

    # Import protobuf messages
    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/imu_msg_pb.jl")

    struct IMU_VICON_C
        acc_x::Cfloat;
        acc_y::Cfloat;
        acc_z::Cfloat;
        gyr_x::Cfloat;
        gyr_y::Cfloat;
        gyr_z::Cfloat;

        pos_x::Cfloat;
        pos_y::Cfloat;
        pos_z::Cfloat;
        quat_w::Cfloat;
        quat_x::Cfloat;
        quat_y::Cfloat;
        quat_z::Cfloat;

        time::Cdouble;
    end

    mutable struct ImuViconNode <: Hg.Node
        # Required by Abstract Node type
        nodeio::Hg.NodeIO
        rate::Float64
        should_finish::Bool

        # Specific to GroundLinkNode
        # ProtoBuf Messages
        imu::IMU
        vicon::VICON
        imu_vicon::MVector{256}

        # Random
        debug::Bool

        function ImuViconNode(teensy_port::String, teensy_baud::Int,
                              imu_pub_ip::String, imu_pub_port::String,
                              vicon_pub_ip::String, vicon_pub_port::String,
                              rate::Float64, debug::Bool)
            # Adding the Ground Vicon Subscriber to the Node
            imuViconNodeIO = Hg.NodeIO(ZMQ.Context(1))
            should_finish = false

            # Adding the Quad Info Subscriber to the Node
            imu = IMU(acc_x=0., acc_y=0., acc_z=0.,
                      gyr_x=0., gyr_y=0., gyr_z=0.,
                      time=0.)
            imu_pub = Hg.ZmqPublisher(imuViconNodeIO.ctx, imu_pub_ip, imu_pub_port)
            Hg.add_publisher!(imuViconNodeIO, imu, imu_pub)

            vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                          quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                          time=0.)
            vicon_pub = Hg.ZmqPublisher(imuViconNodeIO.ctx, vicon_pub_ip, vicon_pub_port)
            Hg.add_publisher!(imuViconNodeIO, vicon, vicon_pub)

            imu_vicon = IMU_VICON(imu=imu, vicon=vicon)

            imu_vicon = @MVector rand(UInt8, 256)
            imu_vicon_sub = Hg.SerialSubscriber(teensy_port, teensy_baud);
            Hg.add_subscriber!(imuViconNodeIO, imu_vicon, imu_vicon_sub)

            return new(imuViconNodeIO, rate, should_finish,
                       imu, vicon, imu_vicon,
                       debug)
        end
    end

    function Hg.startup(node::ImuViconNode)
        # imuViconNodeIO = Hg.getIO(node)
        # open(imuViconNodeIO.subs[1].sub.serial_port)
    end

    function Hg.compute(node::ImuViconNode)
        imuViconNodeIO = Hg.getIO(node)

        # On recieving a new IMU_VICON message
        Hg.on_new(imuViconNodeIO.subs[1]) do imu_vicon
            msg_size = Hg.bytesreceived(imuViconNodeIO.subs[1].sub)
            println(msg_size)

            if msg_size == sizeof(IMU_VICON_C)
                imu_vicon_c = reinterpret(IMU_VICON_C, imu_vicon[1:msg_size])[1]

                acc_vec = SA[imu_vicon_c.acc_x, imu_vicon_c.acc_y, imu_vicon_c.acc_z]
                gyr_vec = SA[imu_vicon_c.gyr_x, imu_vicon_c.gyr_y, imu_vicon_c.gyr_z]
                pos_vec = SA[imu_vicon_c.pos_x, imu_vicon_c.pos_y, imu_vicon_c.pos_z]
                ori_vec = SA[imu_vicon_c.quat_w, imu_vicon_c.quat_x, imu_vicon_c.quat_y, imu_vicon_c.quat_z]

                node.imu.acc_x, node.imu.acc_y, node.imu.acc_z = acc_vec
                node.imu.gyr_x, node.imu.gyr_y, node.imu.gyr_z = gyr_vec
                node.imu.time = time()

                node.vicon.pos_x, node.vicon.pos_y, node.vicon.pos_z = pos_vec
                node.vicon.quat_w, node.vicon.quat_x, node.vicon.quat_y, node.vicon.quat_z = ori_vec
                node.vicon.time = imu_vicon_c.time

                if node.debug
                    @printf("IMU accel: \t[%1.3f, %1.3f, %1.3f]\n",
                            node.imu.acc_x, node.imu.acc_y, node.imu.acc_z)
                    @printf("Vicon pos: \t[%1.3f, %1.3f, %1.3f]\n",
                            node.vicon.pos_x, node.vicon.pos_y, node.vicon.pos_z)
                end
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
# Hg.closeall(filter_node)
