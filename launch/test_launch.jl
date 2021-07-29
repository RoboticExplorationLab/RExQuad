# This file is run on the ground station
a = @allocated begin
    using Pkg
    Pkg.activate("$(@__DIR__)/..")

    include("$(@__DIR__)/../nodes/filtered_state_publisher/filtered_state_publisher_debug.jl")


    # Launch Vicon Relay and Ground link communication
    filter_thread = FilteredStatePublisher.main()
    println("Launched Filtered State Publisher")

    try
        while !istaskdone(filter_thread)
            sleep(0.00001)
        end
        # Check to make sure the task didn't fail on something dumb
        fetch(filter_thread)
    catch e
        if e isa InterruptException  # clean up
            println("Process terminated by you")
        end

        Base.throwto(filter_thread, e)
    end
end

println(a)