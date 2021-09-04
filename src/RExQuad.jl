module RExQuad

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

include("PubSubBuilder.jl")

end  # module