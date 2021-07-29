# This file is run on the ground station
using Pkg
Pkg.activate("$(@__DIR__)/..")

include("$(@__DIR__)/../nodes/vicon_relay/vicon_relay.jl")
include("$(@__DIR__)/../nodes/ground_link/ground_link.jl")


# Launch Vicon Relay and Ground link communication
vicon_relay_thread = ViconRelay.main()
ground_link_thread = GroundLink.main()

try
    while true
        sleep(0.00001)
    end
catch e
    if e isa InterruptException  # clean up
        println("Process terminated by you")
        Base.throwto(vicon_relay_thread, InterruptException())
        Base.throwto(vicon_relay_thread, InterruptException())
    else
        rethrow(e)
    end
end