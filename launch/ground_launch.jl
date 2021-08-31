# This file is run on the ground station
begin
    using Pkg
    Pkg.activate("$(@__DIR__)/..")

    include("$(@__DIR__)/../nodes/vicon_relay/simulated_vicon_relay.jl")
    vicon_relay_thread = SimulatedViconRelay.main(; debug=false)

    # include("$(@__DIR__)/../nodes/vicon_relay/vicon_relay.jl")
    # vicon_relay_thread = ViconRelay.main(; debug=false)

    include("$(@__DIR__)/../nodes/ground_link/ground_link.jl")
    ground_link_thread = GroundLink.main(; debug=true)

    try
        while true
            sleep(0.1)

            # Check there were no issues in any of the threads
            if istaskdone(vicon_relay_thread)
                fetch(vicon_relay_thread); break
            end
            if istaskdone(ground_link_thread)
                fetch(ground_link_thread); break
            end
        end
    catch e
        if e isa InterruptException  # clean up
            println("Process terminated by you")
        end
        schedule(vicon_relay_thread, InterruptException(), error=true)
        schedule(ground_link_thread, InterruptException(), error=true)
    end
end