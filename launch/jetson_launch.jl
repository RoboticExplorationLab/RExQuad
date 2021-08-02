begin
    using Pkg
    Pkg.activate("$(@__DIR__)/..")

    include("$(@__DIR__)/../nodes/imu_vicon_publisher/imu_vicon_publisher.jl")
    include("$(@__DIR__)/../nodes/filtered_state_publisher/filtered_state_publisher.jl")
    include("$(@__DIR__)/../nodes/lqr_hover_controller/lqr_hover_controller.jl")
    include("$(@__DIR__)/../nodes/jetson_link/jetson_link.jl")

    # Launch Various thread
    # imu_vicon_thread = ImuViconPublisher.main(; debug=false)
    # filter_thread = FilteredStatePublisher.main()
    # lqr_thread = LqrHoverController.main()
    jetson_link_thread = JetsonLink.main(; debug=true)

    try
        # while (!istaskdone(imu_vicon_thread) && !istaskdone(filter_thread) &&
        #        !istaskdone(lqr_thread) && !istaskdone(jetson_link_thread))
        # while (!istaskdone(jetson_link_thread) && !istaskdone(imu_vicon_thread))
        while true 
            sleep(0.1)

            if istaskdone(jetson_link_thread)
                fetch(jetson_link_thread); break
            end

            # if istaskdone(jetson_link_thread)
            #     fetch(jetson_link_thread); break
            # end
        end

        # Check there were no issues in any of the threads
        # if istaskdone(imu_vicon_thread) 
        #     fetch(imu_vicon_thread) 
        # end
        # if istaskdone(filter_thread) 
        #     fetch(filter_thread)
        # end
        # if istaskdone(lqr_thread) 
        #     fetch(lqr_thread) 
        # end        

    catch e
        if e isa InterruptException  # clean up
            println("Process terminated by you")
        end
    finally 
        # Base.throwto(imu_vicon_thread, InterruptException())
        # Base.throwto(filtered_state_thread, InterruptException())
        # Base.throwto(lqr_controller_thread, InterruptException())
        # Base.throwto(jetson_link_thread, InterruptException())
    end
end