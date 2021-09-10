# Node which subscribes to the vicon code and sends it to the jetson via
# the telemetry radio
module ViconRelay
    using TOML
    using ZMQ
    using ProtoBuf
    using SerialCOBS
    using Printf

    include("$(@__DIR__)/../utils/PubSubBuilder.jl")
    using .PubSubBuilder

    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")

    # Launch IMU publisher
    function main(; debug=false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        vicon_ip = setup_dict["vicon"]["server"]
        vicon_subject = setup_dict["vicon"]["subject"]

        zmq_vicon_ip = setup_dict["zmq"]["ground"]["vicon"]["server"]
        zmq_vicon_port = setup_dict["zmq"]["ground"]["vicon"]["port"]

        if (debug)
            println("$(@__DIR__)/RExLabVicon/build/vicon_pub $vicon_ip $vicon_subject $zmq_vicon_ip $zmq_vicon_port")
        end

        # Run the CPP ViconDriverZMQ file
        vicon_process = run(`$(@__DIR__)/RExLabVicon/build/vicon_pub $vicon_ip $vicon_subject $zmq_vicon_ip $zmq_vicon_port`, wait=false)
    end
end