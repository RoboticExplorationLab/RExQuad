

abstract type Node end

startup(::Node) = nothing 
compute(::Node) = throw(ErrorException("Compute hasn't been implemented on your node yet!"))
getdata(node::Node)::NoteData = node.data

struct PublishedMessage 
    msg::ProtoBuf.ProtoType
    pub::PubSubBuilder.Publisher
end

struct SubscribedMessage 
    msg::ProtoBuf.ProtoType
    sub::PubSubBuilder.Subscriber
end

mutable struct NodeData
    publishers::Vector{PublishedMessage}
    subscribers::Vector{SubscribedMessage}
    sub_locks::Vector{ReentrantLock}
    frequency::Float64  # Hz
end
add_publisher!(data::NodeData, msg::ProtoBuf.ProtoType, args...) = 
    push!(data.publishers, PublishedMessage(msg, PubSubBuilder.Publisher(args...)))

function add_subscriber!(data::NodeData, msg::ProtoBuf.ProtoType, args...)
    push!(data.sub_locks, ReentrantLock())
    push!(data.subscribers, SubscribedMessage(msg, PubSubBuilder.Subscriber(args...)))
end

function run(node::Node)
    rate = 1 / frequency(node)

    # Launch the subscriber tasks asynchronously
    nodedata = getdata(node)
    for (i,sub_msg) in enumerate(nodedata.subscribers)
        sub_task = @task PubSubBuilder.subscribe(sub_msg.sub, sub_msg.msg, nodedata.sub_locks[i]) 
        schedule(sub_task)
    end

    # Run any necessary startup
    startup(node)

    # Loop over the compute method at a fixed rate
    try
        while true
            compute(node)
            sleep(rate)
            GC.gc(false)
        end
    catch e
        for pub in publishers(node)
            close(pub)
        end
        for sub in subscribers(node)
            close(sub)
        end

        rethrow(e)
    end
end
