using Pkg
Pkg.activate("$(@__DIR__)/..")

include("$(@__DIR__)/../nodes/imu_vicon_pub/imu_vicon_pub.jl")
include("$(@__DIR__)/../nodes/filtered_state_pub/filtered_state_pub.jl")
include("$(@__DIR__)/../nodes/lqr_hover_controller/lqr_hover_controller.jl")
include("$(@__DIR__)/../nodes/jetson_link/jetson_link.jl")

# Launch Vicon Relay
vicon_relay_thread = ViconRelay.main()

imu_vicon_thread = ImuViconPublisher.main()
filtered_state_thread = FilteredStatePublisher.main()
lqr_controller_thread = LqrHoverController.main()
jetson_link_thread = JetsonLink.main()

try
    while true
        sleep(0.0001)
    end
catch e
    if e isa InterruptException  # clean up
        println("Process terminated by you")
        Base.throwto(imu_vicon_thread, InterruptException())
        Base.throwto(filtered_state_thread, InterruptException())
        Base.throwto(lqr_controller_thread, InterruptException())
        Base.throwto(jetson_link_thread, InterruptException())
    else
        rethrow(e)
    end
end