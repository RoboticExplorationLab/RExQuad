# A simple module for helping build publishers and subscibers
module PubSubBuilder
    using ZMQ
    using ProtoBuf

    export create_sub, create_pub, subscriber_thread, publish

    function create_sub(ctx::ZMQ.Context, sub_ip::String, sub_port::String)::ZMQ.Socket
        s = Socket(ctx, SUB)
        ZMQ.subscribe(s)
        ZMQ.connect(s, "tcp://$sub_ip:$sub_port")
        return s
    end

    function create_pub(ctx::ZMQ.Context, pub_ip::String, pub_port::String)::ZMQ.Socket
        p = Socket(ctx, PUB)
        ZMQ.bind(p, "tcp://$pub_ip:$pub_port")
        return p
    end

    function subscriber_thread(ctx::ZMQ.Context, proto_msg::ProtoBuf.ProtoType,
                               sub_ip::String, sub_port::String)::Nothing
        sub = create_sub(ctx, sub_ip, sub_port)
        try
            println("Listening for message type: $proto_msg, on: tcp://$sub_ip:$sub_port")
            while true
                bin_data = recv(sub)
                io = seek(convert(IOStream, bin_data), 0)
                data = readproto(io, proto_msg)
                for n in propertynames(proto_msg)
                    if hasproperty(proto_msg,n)
                        setproperty!(proto_msg, n, getproperty(data, n))
                    end
                end
            end
        catch e
            println(stacktrace())
            println(e)
        finally
            close(sub)
        end

        return Nothing
    end

    function publish(sock::ZMQ.Socket, proto_msg::ProtoBuf.ProtoType,
                     iob::IOBuffer=IOBuffer())::Nothing
        writeproto(iob, proto_msg)
        msg = Message(iob.size)
        msg[:] = iob.data

        ZMQ.send(sock, msg)

        return nothing
    end
end