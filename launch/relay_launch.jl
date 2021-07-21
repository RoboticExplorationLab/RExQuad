using Pkg
Pkg.activate("$(@__DIR__)/..")

include("$(@__DIR__)/../nodes/vicon_relay/vicon_relay.jl")

# Launch Vicon Relay
ViconRelay.main()

while true 
    sleep(0.00001)
end
