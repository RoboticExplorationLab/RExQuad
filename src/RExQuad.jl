module RExQuad
using TOML
using ProtoBuf
using EKF
using ZMQ
using LinearAlgebra

NUM_PUBS = 1;
function genpublishername()
    global NUM_PUBS
    name = "publisher_$NUM_PUBS"
    NUM_PUBS += 1
    return name
end
NUM_SUBS = 1;
function gensubscribername()
    global NUM_SUBS
    name = "subscriber_$NUM_SUBS"
    NUM_SUBS += 1
    return name
end

get_node_setup() = TOML.tryparsefile(joinpath(@__DIR__,"..","nodes","setup.toml"))

for (root, dirs, files) in Base.walkdir(joinpath(@__DIR__,"..","msgs"))
    for msgfile in files
        # @show joinpath(root, msgfile)
        include(joinpath(root,msgfile))
    end
end

include("PubSubBuilder.jl")
include("imu_states.jl")
include("node.jl")
include("filter_node.jl")

end  # module