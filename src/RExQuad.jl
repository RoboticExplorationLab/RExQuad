module RExQuad

    include(joinpath(@__DIR__, "state_estimator", "state_estimator_node.jl"))
    include(joinpath(@__DIR__, "jetson_link", "jetson_link.jl"))

    export StateEstimator
    export JetsonLink


end # module
