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

function floattobytes!(buf::AbstractVector{UInt8}, f::Float32, offset = 0)
    mask = 0x0000_00ff
    fi = reinterpret(UInt32, f)
    buf[1 + offset] = fi & mask
    buf[2 + offset] = (fi >> 8) & mask
    buf[3 + offset] = (fi >> 16) & mask
    buf[4 + offset] = (fi >> 24) & mask
    buf 
end

function bytestofloat(buf::AbstractVector{UInt8}, offset=0)
    reinterpret(Float32, view(buf, (1:4) .+ offset))[1]
end

function bytestofloat2(buf)
    reinterpret(Float32, UInt32(buf[1]) | UInt32(buf[2]) << 8 | UInt32(buf[3]) << 16 | UInt32(buf[4]) << 24)
end

struct MeasurementMsg
    x::Float32   # position
    y::Float32
    z::Float32
    qw::Float32  # orientation
    qx::Float32
    qy::Float32
    qz::Float32
    ax::Float32  # linear acceleration
    ay::Float32
    az::Float32
    wx::Float32  # angular velocity 
    wy::Float32
    wz::Float32
end
function MeasurementMsg()
    MeasurementMsg(
        0f0, 0f0, 0f0,
        1f0, 0f0, 0f0, 0f0,
        0f0, 0f0, 0f0,
        0f0, 0f0, 0f0,
    )
end

function copyto!(buf::AbstractVector{UInt8}, msg::MeasurementMsg)
    floattobytes!(buf, msg.x, 0 * 4)
    floattobytes!(buf, msg.y, 1 * 4)
    floattobytes!(buf, msg.z, 2 * 4)
    floattobytes!(buf, msg.qw, 3 * 4)
    floattobytes!(buf, msg.qx, 4 * 4)
    floattobytes!(buf, msg.qy, 5 * 4)
    floattobytes!(buf, msg.qz, 6 * 4)
    floattobytes!(buf, msg.ax, 7 * 4)
    floattobytes!(buf, msg.ay, 8 * 4)
    floattobytes!(buf, msg.az, 9 * 4)
    floattobytes!(buf, msg.wx, 10 * 4)
    floattobytes!(buf, msg.wy, 11 * 4)
    floattobytes!(buf, msg.wz, 12 * 4)
    buf
end

function MeasurementMsg(buf::AbstractVector{UInt8})
    MeasurementMsg(
        bytestofloat(buf, 0 * 4),
        bytestofloat(buf, 1 * 4),
        bytestofloat(buf, 2 * 4),
        bytestofloat(buf, 3 * 4),
        bytestofloat(buf, 4 * 4),
        bytestofloat(buf, 5 * 4),
        bytestofloat(buf, 6 * 4),
        bytestofloat(buf, 7 * 4),
        bytestofloat(buf, 8 * 4),
        bytestofloat(buf, 9 * 4),
        bytestofloat(buf, 10 * 4),
        bytestofloat(buf, 11 * 4),
        bytestofloat(buf, 12 * 4),
    )
end

struct PoseMsg
    x::Float32   # position
    y::Float32
    z::Float32
    qw::Float32  # orientation
    qx::Float32
    qy::Float32
    qz::Float32
end
function PoseMsg(buf::AbstractVector{UInt8}, off::Integer = 0)
    PoseMsg(
        bytestofloat(buf, 0 * 4 + off),
        bytestofloat(buf, 1 * 4 + off),
        bytestofloat(buf, 2 * 4 + off),
        bytestofloat(buf, 3 * 4 + off),
        bytestofloat(buf, 4 * 4 + off),
        bytestofloat(buf, 5 * 4 + off),
        bytestofloat(buf, 6 * 4 + off),
    )
end
function Base.copyto!(buf::AbstractVector{UInt8}, msg::PoseMsg, off::Integer=0)
    floattobytes!(buf, msg.x, 0 * 4 + off)
    floattobytes!(buf, msg.y, 1 * 4 + off)
    floattobytes!(buf, msg.z, 2 * 4 + off)
    floattobytes!(buf, msg.qw, 3 * 4 + off)
    floattobytes!(buf, msg.qx, 4 * 4 + off)
    floattobytes!(buf, msg.qy, 5 * 4 + off)
    floattobytes!(buf, msg.qz, 6 * 4 + off)
end

struct ControlMsg
    u1::Float32  # front left
    u2::Float32  # front right
    u3::Float32  # back right
    u4::Float32  # back left
end
function ControlMsg(buf::AbstractVector{UInt8}, off::Integer=0)
    ControlMsg(
        bytestofloat(buf, 0 * 4 + off),
        bytestofloat(buf, 1 * 4 + off),
        bytestofloat(buf, 2 * 4 + off),
        bytestofloat(buf, 3 * 4 + off),
    )
end
function Base.copyto!(buf::AbstractVector{UInt8}, msg::PoseMsg, off::Integer=0)
    floattobytes!(buf, msg.u1, 0 * 4 + off)
    floattobytes!(buf, msg.u2, 1 * 4 + off)
    floattobytes!(buf, msg.u3, 2 * 4 + off)
    floattobytes!(buf, msg.u4, 3 * 4 + off)
end
function tovector(msg::ControlMsg)
    SA[msg.u1, msg.u2, msg.u3, msg.u4]
end
function ControlMsg(u::AbstractVector{<:Real})
    ControlMsg(u[1], u[2], u[3], u[4])
end

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
        ZMQ.connect(sub, "tcp://127.0.0.1:$sub_port")
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
    zmsg = ZMQ.Message(sizeof(MeasurementMsg))
    copyto!(zmsg, y)
    ZMQ.send(sim.pub, zmsg)
end

##
sim = Simulator(5561, 5562)
open(sim.vis)

##
x = [zeros(3); 1; zeros(3); zeros(6)]
reset!(sim)
length(sim.msg_meas)
runsim(sim, x, dt=0.01, tf=1.0)
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

sim.msg_meas
sim.msg_pose
t = 0.0
u = trim_controls() 
x[1] += 0.001
y = getmeasurement(sim, x, u, t)
buf = sendmeasurement(sim, y, t)

recv_task = @async ZMQ.recv(sim.sub)
istaskdone(recv_task)
msg = fetch(recv_task)
posemsg = PoseMsg(msg, 1)
posemsg.x == y.x
posemsg.y == y.y
posemsg.z == y.z
posemsg.qw == y.qw
posemsg.qx == y.qx
posemsg.qy == y.qy
posemsg.qz == y.qz
Int(msg[1]) == 11

finish(sim)