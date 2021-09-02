module SimulatedViconRelay
    using TOML
    using ZMQ
    using ProtoBuf
    using SerialCOBS

    include("$(@__DIR__)/../utils/PubSubBuilder.jl")
    using .PubSubBuilder

    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")

    function vicon_relay(vicon_sub_ip::String, vicon_sub_port::String,
                         serial_port::String, baud_rate::Int;
                         freq::Int64=200, debug::Bool=false)
        ctx = Context(1)
        rate = 1/freq
        ard = Arduino(serial_port, baud_rate);

        vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                      quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                      time=time())
        iob = PipeBuffer()

        try
            open(ard) do sp
                while true
                    vicon.pos_x, vicon.pos_y, vicon.pos_z = 1., 1., 1. #rand(3)
                    vicon.quat_w, vicon.quat_x, vicon.quat_y, vicon.quat_z = 1., 1., 1., 1. #rand(4)
                    vicon.time = time()
                    msg_size = writeproto(iob, vicon);

                    # TODO: make sure this works
                    # print(iob.data[1:msg_size],"\r")
                    message(ard, take!(iob))


                    sleep(rate)
                    GC.gc(false)
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
    function main(; debug=false)
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        vicon_ip = setup_dict["zmq"]["ground"]["vicon"]["server"]
        vicon_port = setup_dict["zmq"]["ground"]["vicon"]["port"]

        serial_port = setup_dict["serial"]["ground"]["telemetry_radio"]["serial_port"]
        baud_rate = setup_dict["serial"]["ground"]["telemetry_radio"]["baud_rate"]
        # Launch the relay to send the Vicon data through the telemetry radio
        _vicon_relay() = vicon_relay(vicon_ip, vicon_port,
                                    serial_port, baud_rate;
                                    freq=100, debug=debug)
        vicon_thread = Task(_vicon_relay)
        schedule(vicon_thread)
    end
end