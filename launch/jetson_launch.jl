begin
    using Pkg
    Pkg.activate("$(@__DIR__)/..")

    include("$(@__DIR__)/../nodes/imu_vicon_publisher/imu_vicon_publisher.jl")
    include("$(@__DIR__)/../nodes/filtered_state_publisher/filtered_state_publisher.jl")
    include("$(@__DIR__)/../nodes/lqr_hover_controller/lqr_hover_controller.jl")
    include("$(@__DIR__)/../nodes/jetson_link/jetson_link.jl")

    # Launch Various thread
    imu_vicon_thread = ImuViconPublisher.main(; debug=false)
    jetson_link_thread = JetsonLink.main(; debug=false)
    # filter_thread = FilteredStatePublisher.main()
    # lqr_thread = LqrHoverController.main()

    try
        while true
            sleep(0.1)

            if istaskdone(imu_vicon_thread)
                fetch(imu_vicon_thread); break
            end
            if istaskdone(jetson_link_thread)
                fetch(jetson_link_thread); break
            end
        end

    catch e
        if e isa InterruptException  # clean up
            println("Process terminated by you")
        end
    finally
        Base.throwto(imu_vicon_thread, InterruptException())
        Base.throwto(jetson_link_thread, InterruptException())
        # Base.throwto(filtered_state_thread, InterruptException())
        # Base.throwto(lqr_controller_thread, InterruptException())
    end
end