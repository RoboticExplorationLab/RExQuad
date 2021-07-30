#
module ImuPublisher
    using TOML
    using ZMQ
    using ProtoBuf

    include("$(@__DIR__)/../utils/PubSubBuilder.jl")
    using .PubSubBuilder

    include("$(@__DIR__)/../../msgs/message_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")

    function quad_link(imu_sub_ip::String, imu_sub_port::String;
                       freq::Int64=200, debug::Bool=false)
        ctx = Context(1)

        acc = Vector3_msg(x=0., y=0., z=0.)
        gyr = Vector3_msg(x=0., y=0., z=0.)
        imu = IMU_msg(acceleration=acc, gyroscope=gyr, time=0.)
        imu_pub = create_pub(ctx, imu_sub_ip, imu_sub_port)
        iob = IOBuffer()

        delay = 1 / freq

        try
            while true
                @time begin
                    acc.x = rand()
                    acc.y = rand()
                    acc.z = rand()

                    publish(imu_pub, acc, iob)
                    # seek(iob, 0)
                    sleep(delay)
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
    function main(; debug=false)::Task
        # Launch the relay to send the Vicon data through the telemetry radio
        link_pub() = quad_link("*", "5003";
                               freq=200, debug=debug)
        link_thread = Task(link_pub)
        schedule(link_thread)

        return link_thread
    end
end