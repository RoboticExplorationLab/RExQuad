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
    xhist::Vector{Vector{Float64}}
    uhist::Vector{Vector{Float64}}
    thist::Vector{Float64}
    stats::Dict{String,Any}
    msg_meas::Vector{NamedTuple{(:tsim,:twall,:y), Tuple{Float64, Float64, MeasurementMsg}}}
    msg_data::Vector{NamedTuple{(:tsim,:twall,:xu), Tuple{Float64, Float64, StateControlMsg}}}
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
    xhist = Vector{Float64}[]
    uhist = Vector{Float64}[]
    thist = Vector{Float64}()
    msg_meas = Vector{NamedTuple{(:tsim,:twall,:y), Tuple{Float64, Float64, MeasurementMsg}}}()
    msg_data = Vector{NamedTuple{(:tsim,:twall,:xu), Tuple{Float64, Float64, StateControlMsg}}}()
    stats = Dict{String,Any}()
    Simulator(buf_out, ctx, pub, sub, vis, xhist, uhist, thist, stats, msg_meas, msg_data)
end

function reset!(sim::Simulator)
    empty!(sim.msg_meas)
    empty!(sim.msg_data)
    empty!(sim.xhist)
    empty!(sim.uhist)
    empty!(sim.thist)
end

function finish(sim::Simulator)
    ZMQ.close(sim.pub)
end

function runsim(sim::Simulator, x0; dt=0.0, tf=Inf, kwargs...)
    x = copy(x0)
    t = 0.0
    freq = 1/dt  # Hz
    lrl = LoopRateLimiter(10)
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
    sim.stats["latency"] = latency

    t_start = time()
    while t < tf
        startloop(lrl)

        step!(sim, x, u, t; t_start, kwargs...)

        println("time = ", t, ", z = ", x[3])
        t += dt
        # sleep(lrl)
    end
    @printf("Latency: %.3f ± %.3f ms\n", mean(latency) * 1000, std(latency) * 1000)
end

function step!(sim::Simulator, x, u, t; t_start=time(), visualize=true)
    # Get measurement
    y = getmeasurement(sim, x, u, t)

    # Send measurement
    tsend = time() - t_start
    push!(sim.msg_meas, (;tsim=t, twall=tsend, y))
    sendmeasurement(sim, y, t)

    # Get response from onboard computer
    statecontrol = getdata(sim, t)
    trecv = time() - t_start
    if !isnothing(statecontrol)
        push!(sim.msg_data, (;tsim=t, twall=trecv, xu=statecontrol))
        push!(sim.stats["latency"], trecv - tsend)
        u .= getcontrol(statecontrol)
    end

    # Propagate dynamics
    push!(sim.xhist, copy(x))
    push!(sim.uhist, copy(u))
    push!(sim.thist, t)
    x .= dynamics_rk4(x, u, dt)

    # Visualize
    visualize && RobotMeshes.visualize!(sim.vis, sim, x)
end

function getcontrol(sim, x, t)
    # TODO: Get this from ZMQ
    u = trim_controls() .+ 0.1
    return ControlMsg(u)
end

function getmeasurement(sim::Simulator, x, u, t)
    # TODO: add noise
    xdot = cont_dynamics(x, u)
    MeasurementMsg(
        x[1], x[2], x[3],
        x[4], x[5], x[6], x[7],
        xdot[8], xdot[9], xdot[10],
        x[11], x[12], x[13]
    )
end

function getdata(sim::Simulator, t)
    # TODO: add noise
    buf = ZMQ.Message(100)
    bytes_read = ZMQ.msg_recv(sim.sub, buf, 0)::Int32
    ZMQ.getproperty(sim.sub, :events)
    if bytes_read >= msgsize(StateControlMsg)
        return StateControlMsg(buf)
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
sim = Simulator(5562, 5563)
open(sim.vis)

##
x = [0; 0; 0.5; 1; zeros(3); zeros(6)]
u = trim_controls()
dt = 0.01
tf = 10.0
reset!(sim)
length(sim.msg_meas)
runsim(sim, x; dt, tf)
RobotMeshes.visualize_trajectory!(sim.vis, sim, tf, sim.xhist)

##
tout = getfield.(sim.msg_meas, :twall) 
zout = map(msg->msg.y.z, sim.msg_meas)
tin = getfield.(sim.msg_data, :twall) 
zin = map(msg->msg.xu.z, sim.msg_data)

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
x = [0;0;1; 1; zeros(3); zeros(6)]
u = trim_controls() 
t = 0.0

##
# x[11] += 10.0
# x[13] += -0.1
cont_dynamics(x, u)
y = getmeasurement(sim, x, u, t)
sendmeasurement(sim, y, t)
x[3] += 0.1
##
step!(sim, x, u, t)
t += dt
##
isopen(sim.pub)
isopen(sim.sub)

recv_task = @async ZMQ.recv(sim.sub)
istaskdone(recv_task)
msg = fetch(recv_task)
xu = StateControlMsg(msg, 0)
xu.x == y.x
xu.y == y.y
xu.z == y.z
xu.qw == y.qw
xu.qx == y.qx
xu.qy == y.qy
xu.qz == y.qz
getcontrol(posemsg)
Int(msg[1]) == msgid(StateControlMsg) 

finish(sim)