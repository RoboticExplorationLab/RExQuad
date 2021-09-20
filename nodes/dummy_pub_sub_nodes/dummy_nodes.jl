# Node for communicating with the Jetson (run on the ground station)
module DummyNodes
    using Revise

    import Mercury as Hg
    using ZMQ
    using Printf
    using StaticArrays
    using TOML

    include("$(@__DIR__)/../../msgs/vicon_msg_pb.jl")

    mutable struct PubNode <: Hg.Node
        # Required by Abstract Node type
        nodeio::Hg.NodeIO
        rate::Float64
        should_finish::Bool

        # Specific to GroundLinkNode
        # ProtoBuf Messages
        vicon::VICON

        # Random
        debug::Bool

        function PubNode(vicon_ground_sub_ip::String, vicon_ground_sub_port::String,
                         rate::Float64, debug::Bool)
            # Adding the Ground Vicon Subscriber to the Node
            pubNodeIO = Hg.NodeIO(Context(1))
            # name = "PubNode"
            rate = rate
            should_finish = false

            vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                                quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                                time=0.)
            vicon_pub = Hg.ZmqPublisher(pubNodeIO.ctx, vicon_ground_sub_ip, vicon_ground_sub_port)
            @info "Is vicon_pub ZmqPublisher open?: $(isopen(vicon_pub))"
            Hg.add_publisher!(pubNodeIO, vicon, vicon_pub)

            debug = debug

            return new(pubNodeIO, rate, should_finish,
                       vicon,
                       debug)
        end
    end

    function Hg.compute(node::PubNode)
        nodeio = Hg.getIO(node)

        node.vicon.pos_x += 0.1
        # println("Published")

        Hg.publish.(nodeio.pubs)
    end

    mutable struct SubNode <: Hg.Node
        # Required by Abstract Node type
        nodeio::Hg.NodeIO
        rate::Float64
        should_finish::Bool

        # Specific to GroundLinkNode
        # ProtoBuf Messages
        vicon::VICON

        last_x::Float64

        # Random
        debug::Bool

        function SubNode(vicon_ground_sub_ip::String, vicon_ground_sub_port::String,
                         rate::Float64, debug::Bool)
            # Adding the Ground Vicon Subscriber to the Node
            subNodeIO = Hg.NodeIO(Context(1))
            # name = "SubNode"
            rate = rate
            should_finish = false

            ctx = Context(1)
            vicon = VICON(pos_x=0., pos_y=0., pos_z=0.,
                          quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                          time=0.)
            vicon_sub = Hg.ZmqSubscriber(subNodeIO.ctx, vicon_ground_sub_ip, vicon_ground_sub_port)
            @info "Is vicon_sub ZmqSubscriber open?: $(isopen(vicon_sub))"
            Hg.add_subscriber!(subNodeIO, vicon, vicon_sub)

            debug = debug

            last_x = vicon.pos_x

            return new(subNodeIO, rate, should_finish,
                       vicon, last_x,
                       debug)
        end
    end

    function Hg.compute(node::SubNode)
        if node.last_x != node.vicon.pos_x
            node.last_x = node.vicon.pos_x
            println(node.last_x)
        end
    end

    vicon_ground_ip = "127.0.0.1"
    vicon_ground_port = "5556"

    # Launch IMU publisher
    function pub(; rate=100.0, debug=false)
        pub_node = PubNode(vicon_ground_ip, vicon_ground_port,
                           rate, debug)
        return pub_node
    end

    function sub(; rate=100.0, debug=false)
        sub_node = SubNode(vicon_ground_ip, vicon_ground_port,
                           rate, debug)
        return sub_node
    end
end

# %%
import Mercury as Hg

sub_node = DummyNodes.sub();
sub_node_task = @async Hg.launch(sub_node)

# %%
pub_node = DummyNodes.pub();
pub_node_task = @async Hg.launch(pub_node)

# %%
Base.throwto(sub_node_task, InterruptException())
# %%
Base.throwto(pub_node_task, InterruptException())

# %%

# %%
# sub_node_task = @async Hg.subscribe(sub_node.nodeio.subs[1])

# %%


# %%
Hg.launch(sub_node)

# %%
Hg.closeall(sub_node)

# %%
wait(sub_node_task)

# %%
Base.throwto(sub_node_task, InterruptException())

# %%
println(Hg.node_sockets_are_open(sub_node))

# %%
Hg.launch(sub_node)

# %%
Hg.node_sockets_are_open(sub_node)

# %%
Hg.subscribe(sub_node.nodeio.subs[1])

# %%
Base.throwto(sub_node_task, InterruptException())

# %%
sub_node_task = @async Hg.launch(sub_node)

# %%
Hg.node_sockets_are_open(pub_node) && Hg.node_sockets_are_open(sub_node)


# %%
sub_node_task = @async Hg.launch(sub_node)

# %%
pub_node_task = @async Hg.launch(pub_node)

# schedule(sub_node_task)
# %%


# %%
rate = Hg.getrate(sub_node)
lrl = Hg.LoopRateLimiter(rate)

Hg.start_subscribers(sub_node)

Hg.startup(sub_node)

Hg.isnodedone(sub_node)
Hg.compute(sub_node)


# %%
pub_node_task = @task Hg.launch(pub_node)
schedule(pub_node_task)



# %%
sub_node_task = @task Hg.launch(sub_node)
schedule(sub_node_task)

# %%
Hg.closeall(pub_node)
Hg.closeall(sub_node)

# %%
pub_node_task = @task Hg.launch(pub_node)
schedule(pub_node_task)

# %%
Hg.closeall(pub_node)
Hg.closeall(sub_node)

# %%
Base.throwto(sub_node_task, InterruptException())
# %%
Base.throwto(pub_node_task, InterruptException())

# %%
if all([isopen(submsg.sub) for submsg in node.nodeio.subs])
    Hg.launch(node)
else
    Hg.closeall(node)
end

# %%
Hg.closeall(node)
