# This file is run on the ground station
begin
    using Pkg
    Pkg.activate("$(@__DIR__)/..")

    include("$(@__DIR__)/../nodes/imu_vicon_publisher/imu_vicon_publisher.jl")

    imu_vicon_thread = ImuViconPublisher.main(; debug=true)

    try
        while true
            sleep(0.1)

            if istaskdone(imu_vicon_thread)
                fetch(imu_vicon_thread); break
            end
        end
    catch e
        schedule(imu_vicon_thread, InterruptException(), error=true)
    end
end