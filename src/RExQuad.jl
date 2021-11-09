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

    include(joinpath(@__DIR__, "state_estimator", "state_estimator_node.jl"))
    include(joinpath(@__DIR__, "jetson_link", "jetson_link_node.jl"))

    export StateEstimator
    export JetsonLink


end # module
