# Node which subscribes to the vicon code and sends it to the jetson via 
# the telemetry radio
module ViconRelay
    using TOML
    using ZMQ
    using ProtoBuf
    using SerialCOBS

    include("$(@__DIR__)/../utils/PubSubBuilder.jl")
    using .PubSubBuilder

    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")


    function vicon_relay(vicon_sub_ip::String, vicon_sub_port::String, 
                         serial_port::String, baud_rate::Int; debug::Bool=false)
        ctx = Context(1)
        ard = Arduino(serial_port, baud_rate);

        vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                      quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                      time=time())
        vicon_sub() = subscriber_thread(ctx, vicon, vicon_sub_ip, vicon_sub_port)

        # Setup and Schedule Subscriber Tasks
        vicon_thread = Task(vicon_sub)
        schedule(vicon_thread)

        vicon_time = time()
        iob = PipeBuffer();

        try
            open(ard) do sp
                while true
                    print("Listening\r")
                    if vicon.time > vicon_time 
                        println("Sending message over serial!")
                        writeproto(iob, vicon);
                        message(ard, take!(iob))
                    end  
                end
            end
        catch e
            close(ctx)
            if e isa InterruptException
                println("Process terminated by you")
                Base.throwto(vicon_thread, InterruptException())
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
        zmq_vicon_ip = setup_dict["zmq"]["relay"]["vicon"]["server"]
        zmq_vicon_port = setup_dict["zmq"]["relay"]["vicon"]["port"]

        # Run the CPP ViconDriverZMQ file
        vicon_process = run(`$(@__DIR__)/RExLabVicon/build/vicon_pub $vicon_ip $vicon_subject $zmq_vicon_ip $zmq_vicon_port`, wait=false)

        relay_serial_port = setup_dict["serial"]["relay"]["serial_port"]
        relay_baud_rate = setup_dict["serial"]["relay"]["baud_rate"]

        # Launch the relay to send the Vicon data through the telemetry radio
        vicon_pub() = vicon_relay(zmq_vicon_ip, zmq_vicon_port, 
                                  relay_serial_port, relay_baud_rate; 
                                  debug=true)
        vicon_thread = Task(vicon_pub)
        schedule(vicon_thread)
        
        return vicon_thread
    end 
end