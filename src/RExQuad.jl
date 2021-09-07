module RExQuad
using TOML
using ProtoBuf
using EKF

NUM_PUBS = 1;
function genpublishername()
    name = "publisher_$NUM_PUBS"
    NUM_PUBS += 1
    return name
end
NUM_SUBS = 1;
function gensubscribername()
    name = "subscriber_$NUM_SUBS"
    NUM_SUBS += 1
    return name
end

get_node_setup() = TOML.tryparsefile(joinpath("..","nodes","setup.toml"))

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