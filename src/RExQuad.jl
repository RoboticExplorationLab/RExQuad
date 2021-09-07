module RExQuad
using TOML
using ProtoBuf

NUM_PUBS::Int64 = 1;
function genpublishername()
    name = "publisher_$NUM_PUBS"
    NUM_PUBS += 1
    return name
end
NUM_SUBS::Int64 = 1;
function gensubscribername()
    name = "subscriber_$NUM_SUBS"
    NUM_SUBS += 1
    return name
end

get_node_setup() = TOML.tryparsefile(joinpath("..","nodes","setup.toml"))

include("PubSubBuilder.jl")
include("node.jl")

end  # module