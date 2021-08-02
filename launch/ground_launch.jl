# This file is run on the ground station
begin
    using Pkg
    Pkg.activate("$(@__DIR__)/..")

    include("$(@__DIR__)/../nodes/vicon_relay/vicon_relay.jl")
    include("$(@__DIR__)/../nodes/ground_link/ground_link.jl")


    # vicon_relay_thread = ViconRelay.main()
    ground_link_thread = GroundLink.main(; debug=true)

    try
        while (!istaskdone(ground_link_thread))
            sleep(0.0001)
        end
        # Check there were no issues in any of the threads
        fetch(ground_link_thread)

    catch e
        if e isa InterruptException  # clean up
            println("Process terminated by you")
        end

        Base.throwto(ground_link_thread, InterruptException())
    end
end