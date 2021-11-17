module RExQuad

    # Get all the built message types inside the
    msg_dir = joinpath(@__DIR__, "..", "deps", "msgs")
    if isdir(msg_dir)
        searchdir(path, key) = filter(x->occursin(key,x), readdir(path))

        msg_files = searchdir(msg_dir, "_pb.jl")
        for msg_file in msg_files
            include(joinpath(msg_dir, msg_file))
        end
    else
        @error "RExQuad package hasn't been built yet!"
    end

    include(joinpath(@__DIR__, "constants.jl"))
    include(joinpath(@__DIR__, "quadrotor_model_robot_dynamics.jl"))

    include(joinpath(@__DIR__, "state_estimator", "state_estimator_node.jl"))
    include(joinpath(@__DIR__, "lqr_hover_controller", "lqr_hover_controller_node.jl"))
    include(joinpath(@__DIR__, "lqr_hover_controller", "motor_spin_up.jl"))
    include(joinpath(@__DIR__, "jetson_link", "jetson_link_node.jl"))
    include(joinpath(@__DIR__, "ground_link", "ground_link_node.jl"))
    include(joinpath(@__DIR__, "ground_link", "vicon_listener.jl"))

    export StateEstimator
    export LQRcontroller
    export MotorSpinUp
    export JetsonLink
    export GroundLink
    export ViconListener

    include(joinpath(@__DIR__, "launch", "launch.jl"))
end # module
