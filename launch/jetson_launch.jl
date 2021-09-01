begin
    using Pkg
    Pkg.activate("$(@__DIR__)/..")


    include("$(@__DIR__)/../nodes/imu_vicon_publisher/imu_vicon_publisher.jl")
    include("$(@__DIR__)/../nodes/filtered_state_publisher/filtered_state_publisher.jl")
    include("$(@__DIR__)/../nodes/lqr_hover_controller/lqr_hover_controller.jl")
    include("$(@__DIR__)/../nodes/lqr_hover_controller/lqr_hover_controller_debug.jl")
    # include("$(@__DIR__)/../nodes/lqr_hover_controller/mpc_hover_controller.jl")
    # include("$(@__DIR__)/../nodes/lqr_hover_controller/mpc_hover_controller_debug.jl")
    include("$(@__DIR__)/../nodes/jetson_link/jetson_link.jl")

    # Launch Various thread
    imu_vicon_thread = ImuViconPublisher.main(; debug=false)
    jetson_link_thread = JetsonLink.main(; debug=true)
    filter_thread = FilteredStatePublisher.main(; debug=false)
    # lqr_thread = LqrHoverControllerDebug.main(; debug=false)
    # lqr_thread = LqrHoverController.main(; debug=false)

    try
        while true
            sleep(0.1)

            if istaskdone(imu_vicon_thread)
                fetch(imu_vicon_thread); break
            end
            if istaskdone(jetson_link_thread)
                fetch(jetson_link_thread); break
            end
            if istaskdone(filter_thread)
                fetch(filter_thread); break
            end
            # if istaskdone(lqr_thread)
            #     fetch(lqr_thread); break
            # end
        end

    catch e
        if e isa InterruptException  # clean up
            println("Process terminated by you")
        end
        schedule(imu_vicon_thread, InterruptException(), error=true)
        schedule(jetson_link_thread, InterruptException(), error=true)
        schedule(filter_thread, InterruptException(), error=true)
        # schedule(lqr_thread, InterruptException(), error=true)
    end
end