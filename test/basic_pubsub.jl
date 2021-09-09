import Pkg; Pkg.activate(joinpath(@__DIR__, ".."))
using RExQuad
using ProtoBuf
using Sockets
using ZMQ

# struct ViconPub <: RExQuad.Node
#     data::RExQuad.NodeData
#     msg::RExQuad.VICON
# end

# function ViconPub()
#     data = RExQuad.NodeData()
#     data.frequency = 2
#     msg = VICON(pos_x=0., pos_y=0., pos_z=0.,
#                     quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
#                     time=0.)
#     setup = RExQuad.get_node_setup()
#     RExQuad.add_publisher!(data, msg, ZMQ.Context(1), setup["zmq"]["ground"]["vicon"]["server"],
#         setup["zmq"]["ground"]["vicon"]["port"])
#     @show isopen(data.publishers[end].pub)
#     ViconPub(data, msg)
# end

# function RExQuad.compute(node::ViconPub)
#     node.msg.pos_x += 1
#     node.msg.pos_y += 2
#     pub = node.data.publishers[1].pub
#     RExQuad.PubSubBuilder.publish(pub, node.msg)
#     if isopen(pub)
#     else
#         println("Publisher port is closed!")
#     end
# end


# mutable struct ViconSub <: RExQuad.Node
#     data::RExQuad.NodeData
#     msg::RExQuad.VICON
#     time::Float64
# end

# function ViconSub()
#     data = RExQuad.NodeData()
#     data.frequency = 1
#     msg = VICON(pos_x=0., pos_y=0., pos_z=0.,
#                     quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
#                     time=0.)
#     setup = RExQuad.get_node_setup()
#     RExQuad.add_subscriber!(data, msg, ZMQ.Context(1), ip"127.0.0.2", 5004)
#     ViconSub(data, msg, 0.0)
# end

# function RExQuad.compute(node::ViconSub)
#     vicon_lock = node.data.sub_locks[1]
#     x = [0,0,0]
#     lock(vicon_lock) do
#         x[1] = node.msg.pos_x
#         x[2] = node.msg.pos_y
#         x[3] = node.msg.pos_z
#     end
#     node.time += 1
#     println("Got x = [$(x[1]), $(x[2]), $(x[3])], t = $(node.time)")
# end

function basic_pub()
    ctx = ZMQ.Context(1)
    socket = ZMQ.Socket(ctx, ZMQ.PUB)
    ZMQ.bind(socket, "tcp://127.0.0.2:5005")
    proto_msg = VICON(pos_x=1., pos_y=2., pos_z=0.,
                    quat_w=0., quat_x=0., quat_y=0., quat_z=0., time=0.)

    iob = IOBuffer()
    i = 0
    try
        while true
            proto_msg.pos_x = i
            msg_size = writeproto(iob, proto_msg)
            msg = Message(msg_size)
            ZMQ.send(socket, msg)

            # Move cursor position to begining
            seek(iob, 0)
            i += 1
            println("i = $i")
            sleep(0.05)
        end
    catch e
        if e isa InterruptException
            println("Pub Thread Stopped")
            close(socket)
        else
            throw(e)
        end
    end
end

function basic_sub()
    ctx = ZMQ.Context(1)
    socket = ZMQ.Socket(ctx, ZMQ.SUB)
    ZMQ.subscribe(socket, "tcp://127.0.0.2:5005")
    proto_msg = VICON(pos_x=0., pos_y=0., pos_z=0.,
                    quat_w=0., quat_x=0., quat_y=0., quat_z=0., time=0.)
    println("Starting Subscriber loop...")
    try
        while true
            bin_data = ZMQ.recv(socket)
            # io = seek(convert(IOStream, bin_data), 0)
            # readproto(io, proto_msg)
            x = [proto_msg.pos_x, proto_msg.pos_y, proto_msg.pos_z]

            println("x = [$(x[1]), $(x[2]), $(x[3])]")
            GC.gc(false)
            sleep(0.1)
        end
    catch e
        if e isa InterruptException
            println("Sub Thread Stopped")
            close(socket)
        else
            throw(e)
        end
    end
end


##
using ZMQ, Sockets, Base.Threads
function simple_pub(ctx)

    pub = ZMQ.Socket(ctx, PUB)
    ZMQ.bind(pub, "tcp://127.0.0.2:5555")
    i = 0
    try
        while true
            send(pub, "test pub")
            println("Publishing, i = $i...")
            i += 1
            sleep(1e-3)
        end
    catch e
        close(pub)
        if e isa InterruptException
            println("Closing publisher")
        else
            throw(e)
        end
    end
end
ctx = ZMQ.Context(1)
pub_task = @task simple_pub(ctx)
schedule(pub_task)
schedule(pub_task, InterruptException(), error=true)

sub = ZMQ.Socket(ctx, SUB)
ZMQ.subscribe(sub)
ZMQ.connect(sub, "tcp://127.0.0.2:5555")
msg = recv(sub, String)
# send(s1, "test response")
# msg = recv(s2, String)
close(s1)
close(s2)

ctx = ZMQ.Context(1)
pub = ZMQ.Socket(ctx, ZMQ.PUB)
sub = ZMQ.Socket(ctx, ZMQ.SUB)
ZMQ.bind(pub)


##
pub_task = @task basic_pub()
schedule(pub_task)
wait(pub_task)
schedule(pub_task, InterruptException(), error=true)

sub_task = @task basic_sub()
schedule(sub_task)
schedule(sub_task, InterruptException(), error=true)