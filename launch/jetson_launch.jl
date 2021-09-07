begin
    using Pkg
    Pkg.activate("$(@__DIR__)/..")

    include("$(@__DIR__)/../nodes/imu_vicon_publisher/imu_vicon_publisher.jl")
    include("$(@__DIR__)/../nodes/filtered_state_publisher/filtered_state_publisher.jl")
    # include("$(@__DIR__)/../nodes/jetson_link/jetson_link.jl")

    # Launch Various thread
    imu_vicon_thread = ImuViconPublisher.main(; debug=false)
    # filter_thread = FilteredStatePublisher.main(; debug=false)
    # jetson_link_thread = JetsonLink.main(; debug=false)

    try
        while true
            sleep(0.1)

            if istaskdone(imu_vicon_thread)
                fetch(imu_vicon_thread); break
            end
            # if istaskdone(filter_thread)
            #     fetch(filter_thread); break
            # end
            # if istaskdone(jetson_link_thread)
            #     fetch(jetson_link_thread); break
            # end
        end
    catch e
        schedule(imu_vicon_thread, InterruptException(), error=true)
        # schedule(filter_thread, InterruptException(), error=true)
        # schedule(jetson_link_thread, InterruptException(), error=true)
    end
end