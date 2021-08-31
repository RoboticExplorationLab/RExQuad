# This file is run on the ground station
begin
    using Pkg
    Pkg.activate("$(@__DIR__)/..")

    node_dir = "$(@__DIR__)/../nodes"
    # include("$(node_dir)/filtered_state_publisher/filtered_state_publisher_debug.jl")
    # include("$(node_dir)/jetson_link/jetson_link.jl")
    # include("$(node_dir)/jetson_link/imu_subscriber.jl")
    include("$(node_dir)/jetson_link/imu_publisher.jl")

    # Launch Vicon Relay and Ground link communication
    # filter_thread = FilteredStatePublisher.main()
    # println("Launched Filtered State Publisher")

    # Launch Vicon Relay and Ground link communication
    # link_thread = ImuPublisher.main()
    # println("Launched Imu Publisher")
    link_thread = ImuPublisher.main()
    println("Launched Imu Publisher")


    try
        while true
            if istaskdone(link_thread)
                fetch(link_thread); break
            end
            if istaskdone(filter_thread)
                fetch(filter_thread); break
            end
        end
    catch e
        if e isa InterruptException  # clean up
            println("Process terminated by you")
        end

        Base.throwto(filter_thread, e)
        Base.throwto(link_thread, e)
    end
end