begin
    using Pkg
    Pkg.activate("$(@__DIR__)/..")


    include("$(@__DIR__)/../nodes/jetson_link/jetson_link.jl")
    include("$(@__DIR__)/../nodes/imu_vicon_publisher/imu_vicon_publisher.jl")
    include("$(@__DIR__)/../nodes/filtered_state_publisher/filtered_state_publisher.jl")
    # include("$(@__DIR__)/../nodes/lqr_hover_controller/lqr_hover_controller.jl")
    # include("$(@__DIR__)/../nodes/lqr_hover_controller/lqr_hover_controller_debug.jl")
    # include("$(@__DIR__)/../nodes/lqr_hover_controller/mpc_hover_controller.jl")
    # include("$(@__DIR__)/../nodes/lqr_hover_controller/mpc_hover_controller_debug.jl")

    # Launch Various thread
    jetson_link_thread = JetsonLink.main(; debug=false)
    imu_vicon_thread = ImuViconPublisher.main(; debug=false)
    filter_thread = FilteredStatePublisher.main(; debug=false)
    # lqr_thread = LqrHoverControllerDebug.main(; debug=false)
    # lqr_thread = LqrHoverController.main(; debug=false)

    try
        while true
            sleep(0.1)

            if istaskdone(jetson_link_thread)
                fetch(jetson_link_thread); break
            end
            if istaskdone(imu_vicon_thread)
                fetch(imu_vicon_thread); break
            end
            if istaskdone(filter_thread)
                fetch(filter_thread); break
            end
            # if istaskdone(lqr_thread)
            #     fetch(lqr_thread); break
            # end
        end

    catch e
        schedule(jetson_link_thread, InterruptException(), error=true)
        schedule(imu_vicon_thread, InterruptException(), error=true)
        schedule(filter_thread, InterruptException(), error=true)
        # schedule(lqr_thread, InterruptException(), error=true)
    end
end