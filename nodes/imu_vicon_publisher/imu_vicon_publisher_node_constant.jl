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

    mutable struct ImuViconNode <: Hg.Node
        # Required by Abstract Node type
        nodeio::Hg.NodeIO

        # Specific to GroundLinkNode
        # ProtoBuf Messages
        imu::IMU
        vicon::VICON

        # Random
        debug::Bool

        function ImuViconNode(imu_pub_ip::String, imu_pub_port::String,
                              vicon_pub_ip::String, vicon_pub_port::String,
                              rate::Float64, debug::Bool)
            # Adding the Ground Vicon Subscriber to the Node
            imuViconNodeIO = Hg.NodeIO(ZMQ.Context(1); rate=rate)

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

            return new(imuViconNodeIO,
                       imu, vicon,
                       debug)
        end
    end

    function Hg.compute(node::ImuViconNode)
        imuViconNodeIO = Hg.getIO(node)

        # On recieving a new IMU_VICON message
        node.imu.acc_x, node.imu.acc_y, node.imu.acc_z = @SVector zeros(3)
        node.imu.gyr_x, node.imu.gyr_y, node.imu.gyr_z = @SVector zeros(3)
        node.imu.time = time()

        node.vicon.pos_x, node.vicon.pos_y, node.vicon.pos_z = @SVector zeros(3)
        node.vicon.quat_w, node.vicon.quat_x, node.vicon.quat_y, node.vicon.quat_z = @SVector [1,0,0,0]
        node.vicon.time = time()

        # Publish on all topics in NodeIO
        Hg.publish.(imuViconNodeIO.pubs)
    end

    # Launch IMU publisher
    function main(; rate::Float64=150.0, debug::Bool=false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        imu_ip = setup_dict["zmq"]["jetson"]["imu"]["server"]
        imu_port = setup_dict["zmq"]["jetson"]["imu"]["port"]
        # imu_port = "5566"

        vicon_ip = setup_dict["zmq"]["jetson"]["vicon"]["server"]
        vicon_port = setup_dict["zmq"]["jetson"]["vicon"]["port"]
        # vicon_port = "5544"

        node = ImuViconNode(imu_ip, imu_port,
                            vicon_ip, vicon_port,
                            rate, debug)
        return node
    end
end

# # %% For Testing
# @warn "Testing code is uncommented"
# import Mercury as Hg
# filter_node = ImuViconPublisher.main(; debug=true);

# # %%
# filter_node_task = Threads.@spawn Hg.launch(filter_node)

# # %%
# Hg.closeall(filter_node)
