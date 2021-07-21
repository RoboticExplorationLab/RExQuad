module ViconRelay
    # using Pkg
    # Pkg.activate("$(@__DIR__)/../..")

    using TOML
    using ZMQ
    using ProtoBuf
    using SerialCOBS

    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")

    function create_sub(ctx, sub_ip, sub_port)
        s = Socket(ctx, SUB)
        ZMQ.subscribe(s)
        ZMQ.connect(s, "tcp://$sub_ip:$sub_port")
        return s
    end

    function vicon_relay(vicon_sub_ip::String, vicon_sub_port::String, 
                         serial_port::String, baud_rate::Int; 
                         debug::Bool=false)
        ard = Arduino(serial_port, baud_rate);

        ctx = Context(1)
        vicon_sub = create_sub(ctx, vicon_sub_ip, vicon_sub_port)
        vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                      quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                      time=time())

        try
            open(ard) do sp
                while true
                    bin_data = recv(vicon_sub)
                    iob = seek(convert(IOStream, bin_data), 0)
                    
                    # TODO: make sure this works
                    message(ard, iob.data)
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

        vicon_ip = setup_dict["vicon"]["server"]
        vicon_subject = setup_dict["vicon"]["subject"]
        zmq_relay_ip = setup_dict["zmq"]["relay"]["server"]
        zmq_vicon_port = setup_dict["zmq"]["relay"]["vicon_port"]
        # Run the CPP ViconDriverZMQ file
        vicon_process = run(`$(@__DIR__)/RExLabVicon/build/vicon_pub $vicon_ip $vicon_subject $zmq_relay_ip $zmq_vicon_port`, wait=false)

        relay_serial_port = setup_dict["serial"]["relay"]["serial_port"]
        relay_baud_rate = setup_dict["serial"]["relay"]["baud_rate"]
        # Launch the relay to send the Vicon data through the telemetry radio
        vicon_pub() = vicon_relay(zmq_relay_ip, zmq_vicon_port, 
                                  relay_serial_port, relay_baud_rate; 
                                  debug=true)
        vicon_thread = Task(vicon_pub)
        schedule(vicon_thread)
    end
end