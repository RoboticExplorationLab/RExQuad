# This file is run on the ground station
begin
    using Pkg
    Pkg.activate("$(@__DIR__)/..")

    include("$(@__DIR__)/../nodes/imu_vicon_publisher/imu_vicon_publisher.jl")
    include("$(@__DIR__)/../nodes/vicon_relay/vicon_relay_debug.jl")

    vicon_relay = ViconRelayDebug.main(; debug=false)
    imu_vicon_thread = ImuViconPublisher.main(; debug=true)

    try
        while true
            sleep(0.1)

            if istaskdone(vicon_relay)
                fetch(vicon_relay); break
            end
            if istaskdone(imu_vicon_thread)
                fetch(imu_vicon_thread); break
            end
        end
    catch e
        schedule(vicon_relay, InterruptException(), error=true)
        schedule(imu_vicon_thread, InterruptException(), error=true)
    end
end