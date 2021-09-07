# A simple module for helping build publishers and subscibers
module PubSubBuilder
    using ZMQ
    using ProtoBuf
    using Sockets
    using Logging

    export create_sub, create_pub, subscriber_thread, publish

    """
        Publisher
    
    A simple wrapper around a ZMQ publisher, but only publishes protobuf messages.

    # Construction 

        Publisher(context::ZMQ.Context, ipaddr, port; name)
        
    To create a publisher, pass in a `ZMQ.Context`, which allows all related 
    publisher / subscribers to be collected in a "group." The publisher also 
    needs to be provided the IPv4 address (either as a string or as a `Sockets.IPv4`
    object), and the port (either as an integer or a string).

    A name can also be optionally provided via the `name` keyword, which can be used
    to provide a helpful description about what the publisher is publishing. It defaults
    to "publisher_#" where `#` is an increasing index.

    # Usage
    To publish a message, just use the `publish` method on a protobuf type:

        publish(pub::Publisher, proto_msg::ProtoBuf.ProtoType)
    """
    struct Publisher
        socket::ZMQ.Socket
        port::Int64
        ipaddr::IPv4
        buffer::IOBuffer
        name::String
        function Publisher(ctx::ZMQ.Context, ipaddr::IPv4, port::Integer; name=genpublishername())
            socket = ZMQ.Socket(ctx, ZMQ.SUB)
            ZMQ.bind(socket, "tcp://$ipaddr:$port")
            @info "Publishing $name on: tcp://$ipaddr:$port"
            new(socket, port, ipaddr, IOBuffer, name)
        end
    end
    function Publisher(ctx::ZMQ.Context, ipaddr, port::Integer; name=genpublishername())
        Publisher(ctx, IPv4(ipaddr), port)
    end
    function Publisher(ctx::ZMQ.Context, ipaddr, port::AbstractString; name=genpublishername())
        Publisher(ctx, IPv4(ipaddr), parse(Int, port))
    end
    Base.isopen(pub::Publisher) = Base.isopen(pub.socket)
    Base.close(pub::Publisher) = Base.close(pub.socket)

    function publish(pub::Publisher, proto_msg::ProtoBuf.ProtoType)
        # Encode the message with protobuf
        msg_size = writeproto(pub.buffer, proto_msg)

        # Create a new message to be sent and copy the encoded protobuf bytes
        msg = Message(msg_size) 
        copyto!(msg, 1, pub.buffer.data, 1, msg_size)

        # Send over ZMQ
        # NOTE: ZMQ will de-allocate the message allocated above, so garbage
        # collection should not be an issue here
        ZMQ.send(pub.socket, msg)

        # Move to the beginning of the buffer
        seek(pub.buffer, 0)
    end

    function publish(sock::ZMQ.Socket, proto_msg::ProtoBuf.ProtoType,
                     iob::IOBuffer=IOBuffer())::Nothing
        msg_size = writeproto(iob, proto_msg)
        msg = Message(msg_size)
        ZMQ.send(sock, msg)

        # Move cursor position to begining
        seek(iob, 0)

        return nothing
    end


    """
        Subscriber

    A simple wrapper around a ZMQ subscriber, but only for protobuf messages.

    # Construction 

        Subscriber(context::ZMQ.Context, ipaddr, port; name)
        
    To create a subscriber, pass in a `ZMQ.Context`, which allows all related 
    publisher / subscribers to be collected in a "group." The subscriber also 
    needs to be provided the IPv4 address (either as a string or as a `Sockets.IPv4`
    object), and the port (either as an integer or a string).

    A name can also be optionally provided via the `name` keyword, which can be used
    to provide a helpful description about what the subscriber is subscribing to. It defaults
    to "subscriber_#" where `#` is an increasing index.

    # Usage
    Use the blocking `subscribe` method to continually listen to the socket and 
    store data in a protobuf type:

        subscribe(sub::Subscriber, proto_msg::ProtoBuf.ProtoType)

    Note that this function contains an infinite while loop so will block the calling
    thread indefinately. It's usually best to assign the process to a separate thread / task:

    ```
    sub_task = @task subscribe(sub, proto_msg)
    schedule(sub_task)
    ```
    """
    struct Subscriber
        socket::ZMQ.Socket
        port::Int64
        ipaddr::IPv4
        buffer::IOBuffer
        name::String
        function Subscriber(ctx::ZMQ.Context, ipaddr::IPv4, port::Integer; name=gensubscribername())
            socket = ZMQ.Socket(ctx, ZMQ.PUB)
            ZMQ.subscribe(socket)
            ZMQ.connect(socket, "tcp://$ipaddr:$port")
            @info "Subscribing $name to: tcp://$ipaddr:$port"
            new(socket, port, ipaddr, IOBuffer, name)
        end
    end
    function Subscriber(ctx::ZMQ.Context, ipaddr, port::Integer; name=gensubscribername())
        Subscriber(ctx, IPv4(ipaddr), port)
    end
    function Subscriber(ctx::ZMQ.Context, ipaddr, port::AbstractString; name=gensubscribername())
        Subscriber(ctx, IPv4(ipaddr), parse(Int, port))
    end
    Base.isopen(sub::Subscriber) = Base.isopen(sub.socket)
    Base.close(sub::Subscriber) = Base.close(sub.socket)

    function subscribe(sub::Subscriber, proto_msg::ProtoBuf.ProtoType)
        @info "Listening for message type: $(typeof(proto_msg)), on: tcp://$sub_ip:$sub_port"
        try
            while true
                bin_data = ZMQ.recv(sub)
                # lock
                # Why not just call IOBuffer(bin_data)?
                io = seek(convert(IOStream, bin_data), 0)
                readproto(io, proto_msg)
                # unlock

                GC.gc(false)
            end
        catch e
            close(ctx)
            close(sub)
            @info "Shutting Down $(typeof(proto_msg)) subscriber, on: tcp://$sub_ip:$sub_port"

            # println(stacktrace())
            # println(e)
            rethrow(e)
        end

        return nothing
    end

    # OLD API
    function create_sub(ctx::ZMQ.Context, sub_ip::String, sub_port::String)::ZMQ.Socket
        s = Socket(ctx, SUB)
        

        # setsockopt(s, ZMQ_CONFLATE, &conflate, sizeof(conflate) )
        
        ZMQ.subscribe(s)
        ZMQ.connect(s, "tcp://$sub_ip:$sub_port")

        return s
    end

    function create_pub(ctx::ZMQ.Context, pub_ip::String, pub_port::String)::ZMQ.Socket
        p = Socket(ctx, PUB)
        ZMQ.bind(p, "tcp://$pub_ip:$pub_port")

        println("Publishing message on: tcp://$pub_ip:$pub_port")
        return p
    end

    function subscriber_thread(ctx::ZMQ.Context, proto_msg::ProtoBuf.ProtoType,
                               sub_ip::String, sub_port::String)::Nothing
        sub = create_sub(ctx, sub_ip, sub_port)
        # ZMQ.set_rcvhwm(sub, 1)

        try
            println("Listening for message type: $(typeof(proto_msg)), on: tcp://$sub_ip:$sub_port")
            while true
                bin_data = ZMQ.recv(sub)
                # lock
                # Why not just call IOBuffer(bin_data)?
                io = seek(convert(IOStream, bin_data), 0)
                readproto(io, proto_msg)
                # unlock

                GC.gc(false)
            end
        catch e
            close(ctx)
            close(sub)
            println("Shutting Down $(typeof(proto_msg)) subscriber, on: tcp://$sub_ip:$sub_port")

            println(stacktrace())
            println(e)
            rethrow(e)
        end

        return nothing
    end

end