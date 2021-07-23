using Pkg
Pkg.activate("$(@__DIR__)/..")

include("$(@__DIR__)/../nodes/vicon_relay/vicon_relay.jl")

# Launch Vicon Relay
vicon_relay_thread = ViconRelay.main()

try
    while true 
        sleep(0.00001)
    end
catch e
    if e isa InterruptException  # clean up
        println("Process terminated by you")
        Base.throwto(vicon_relay_thread, InterruptException())
    else 
        rethrow(e)
    end 
end