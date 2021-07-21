module SimulatedViconRelay
    # using Pkg
    # Pkg.activate(@__DIR__)

    using TOML
    using ZMQ
    using ProtoBuf
    using SerialCOBS

    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")

    function vicon_relay(vicon_sub_ip::String, vicon_sub_port::String, 
                         serial_port::String, baud_rate::Int; 
                         debug::Bool=false)
        ard = Arduino(serial_port, baud_rate);

        ctx = Context(1)
        vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                      quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                      time=time())
        iob = PipeBuffer();
        
        try
            open(ard) do sp
                while true
                    vicon.pos_x, vicon.pos_y, vicon.pos_z = rand(3)
                    vicon.quat_w, vicon.quat_x, vicon.quat_y, vicon.quat_z = rand(4)
                    vicon.time = rand()
                    writeproto(iob, vicon);

                    # TODO: make sure this works
                    message(ard, take!(iob))
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

        zmq_jetson_ip = setup_dict["zmq"]["vicon_relay"]["server"]
        zmq_vicon_port = setup_dict["zmq"]["vicon_relay"]["vicon_port"]
        
        relay_serial_port = setup_dict["serial"]["relay"]["serial_port"]
        relay_baud_rate = setup_dict["serial"]["relay"]["baud_rate"]
        # Launch the relay to send the Vicon data through the telemetry radio
        vicon_relay() = vicon_relay(zmq_jetson_ip, zmq_vicon_port, 
                                    relay_serial_port, relay_baud_rate; 
                                    debug=true)
        vicon_thread = Task(vicon_relay)
        schedule(vicon_thread)
    end
end