import Pkg; Pkg.activate(@__DIR__)
using ZMQ
using Sockets
using MeshCat
using StaticArrays
using Statistics
using Printf

include("dynamics.jl")
include("ratelimiter.jl")
include("visualization.jl")
include("messages.jl")

const ZMQ_CONFLATE = 54
function set_conflate(socket::ZMQ.Socket, option_val::Integer)
    rc = ccall(
        (:zmq_setsockopt, ZMQ.libzmq),
        Cint,
        (Ptr{Cvoid}, Cint, Ref{Cint}, Csize_t),
        socket,
        ZMQ_CONFLATE,
        option_val,
        sizeof(Cint),
    )
    if rc != 0
        throw(ZMQ.StateError(ZMQ.jl_zmq_error_str()))
    end
end

struct Simulator
    buf_out::ZMQ.Message
    ctx::ZMQ.Context
    pub::ZMQ.Socket
    sub::ZMQ.Socket
    vis::Visualizer
    msg_meas::Vector{NamedTuple{(:tsim,:twall,:y), Tuple{Float64, Float64, MeasurementMsg}}}
    msg_pose::Vector{NamedTuple{(:tsim,:twall,:x), Tuple{Float64, Float64, PoseMsg}}}
    msg_ctrl::Vector{NamedTuple{(:tsim,:twall,:u), Tuple{Float64, Float64, ControlMsg}}}
end

function Simulator(pub_port=5555, sub_port=5556)
    buf_out = ZMQ.Message(sizeof(MeasurementMsg))
    ctx = ZMQ.Context()
    pub = ZMQ.Socket(ctx, ZMQ.PUB)
    try
        ZMQ.bind(pub, "tcp://*:$pub_port")
    catch err
        if err isa StateError
            throw(StateError("Error opening Pubisher"))
        else
            rethrow(err)
        end
    end

    sub = ZMQ.Socket(ctx, ZMQ.SUB)
    try
        ZMQ.subscribe(sub)
        set_conflate(sub, 1)
        ZMQ.bind(sub, "tcp://127.0.0.1:$sub_port")
    catch err
        close(pub)
        if err isa StateError
            throw(StateError("Error opening Subscriber"))
        else
            rethrow(err)
        end
    end

    vis = Visualizer()
    RobotMeshes.setdrone!(vis)

    # logs
    msg_meas = Vector{NamedTuple{(:tsim,:twall,:y), Tuple{Float64, Float64, MeasurementMsg}}}()
    msg_pose = Vector{NamedTuple{(:tsim,:twall,:x), Tuple{Float64, Float64, PoseMsg}}}()
    msg_ctrl = Vector{NamedTuple{(:tsim,:twall,:u), Tuple{Float64, Float64, PoseMsg}}}()
    Simulator(buf_out, ctx, pub, sub, vis, msg_meas, msg_pose, msg_ctrl)
end

function reset!(sim::Simulator)
    empty!(sim.msg_ctrl)
    empty!(sim.msg_meas)
    empty!(sim.msg_pose)
end

function finish(sim::Simulator)
    ZMQ.close(sim.pub)
end

function runsim(sim::Simulator, x0; dt=0.0, tf=Inf)
    
    x = copy(x0)
    t = 0.0
    freq = 1/dt  # Hz
    lrl = LoopRateLimiter(10)
    t_start = time()
    u = trim_controls() 

    # TODO: flush out publishers / subscribers
    buf = ZMQ.Message(100)
    while ZMQ.msg_recv(sim.sub, buf, ZMQ.ZMQ_DONTWAIT) >= 0 end
    latency = Float64[]
    if tf < Inf
        sizehint!(latency, round(Int, tf / dt))
    else
        sizehint!(latency, 10_000)
    end
    while t < tf
        startloop(lrl)

        # Get measurement
        y = getmeasurement(sim, x, u, t)

        # Send measurement
        tsend = time() - t_start
        push!(sim.msg_meas, (;tsim=t, twall=tsend, y))
        sendmeasurement(sim, y, t)

        # Get control
        umsg = getcontrol(sim, x, t) 
        trecv = time() - t_start
        push!(sim.msg_ctrl, (;tsim=t, twall=trecv, u=umsg))
        u = tovector(umsg)

        # Get Pose
        tpose = time() - t_start
        xpose = getpose(sim, x, u, t)
        if !isnothing(xpose)
            push!(sim.msg_pose, (;tsim=t, twall=tpose, x=xpose))
        end
        push!(latency, tpose - tsend)

        # Propagate dynamics
        x = dynamics_rk4(x, u, dt)

        # Visualize
        RobotMeshes.visualize!(sim.vis, sim, x)

        println("time = ", t)
        t += dt
        # sleep(lrl)
    end
    @printf("Latency: %.3f ± %.3f ms\n", mean(latency) * 1000, std(latency) * 1000)
end

function getcontrol(sim, x, t)
    # TODO: Get this from ZMQ
    u = trim_controls() .+ 0.1
    return ControlMsg(u)
end

function getmeasurement(vis::Simulator, x, u, t)
    # TODO: add noise
    xdot = cont_dynamics(x, u)
    MeasurementMsg(
        x[1], x[2], x[3],
        x[4], x[5], x[6], x[7],
        xdot[8], xdot[9], xdot[10],
        x[11], x[12], x[13]
    )
end

function getpose(sim::Simulator, x, u, t)
    # TODO: add noise
    buf = ZMQ.Message(100)
    bytes_read = ZMQ.msg_recv(sim.sub, buf, 0)::Int32
    ZMQ.getproperty(sub.socket, :events)
    if bytes_read > sizeof(PoseMsg)
        PoseMsg(buf, 1)
    else
        return nothing
    end
end

function sendmeasurement(sim::Simulator, y::MeasurementMsg, t)
    zmsg = ZMQ.Message(msgsize(y))
    copyto!(zmsg, y)
    ZMQ.send(sim.pub, zmsg)
end

##
sim = Simulator(5560, 5561)
open(sim.vis)

##
x = [zeros(3); 1; zeros(3); zeros(6)]
reset!(sim)
length(sim.msg_meas)
runsim(sim, x, dt=0.01, tf=10.0)
tout = getfield.(sim.msg_meas, :twall) 
zout = map(msg->msg.y.z, sim.msg_meas)
tin = getfield.(sim.msg_pose, :twall) 
zin = map(msg->msg.x.z, sim.msg_pose)

j = 1
idx = zeros(Int,length(tin)) 
for i = 1:length(tin)
    z = zin[i]
    for k = j:length(tout)
        if zout[k] ≈ z
            idx[i] = k
            j = k  # start search at current time step
        end
    end
end
idx_in = findall(iszero |> !, idx)
idx_out = idx[idx_in]
matches = map(1:length(idx_out)) do k
    i = idx_in[k]
    j = idx_out[k]
    (
        tout=tout[j], tin=tin[i], 
        zout=zout[j], zin=zin[i]
    )
end
matches
all(x->x.zout == x.zin, matches)
all(x->x.tin > x.tout, matches)
latency = map(x->(x.tin - x.tout) * 1000, matches)
mean(latency)
std(latency)

##
x = [zeros(3); 1; zeros(3); zeros(6)]
u = trim_controls() 
t = 0.0
x[1] += 0.001
y = getmeasurement(sim, x, u, t)
buf = sendmeasurement(sim, y, t)
msgsize(y)
isopen(sim.pub)
isopen(sim.sub)

recv_task = @async ZMQ.recv(sim.sub)
istaskdone(recv_task)
msg = fetch(recv_task)
posemsg = StateControlMsg(msg, 0)
Int(msg[1])
posemsg.x == y.x
posemsg.y == y.y
posemsg.z == y.z
posemsg.qw == y.qw
posemsg.qx == y.qx
posemsg.qy == y.qy
posemsg.qz == y.qz
Int(msg[1]) == 11

finish(sim)