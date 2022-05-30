using ZMQ
using Sockets
using MeshCat
using StaticArrays
using Statistics
using Printf
using Colors
using DataStructures

include("dynamics.jl")
include("ratelimiter.jl")
include("visualization.jl")
include("messages.jl")

function build_osqp_controller(N)

    # Hover state
    n = 12
    m = 4
    xhover = [0; 0; 1; 1; 0; 0; 0; zeros(6)]
    uhover = trim_controls()
    dt = 0.01  # 100 Hz

    # Get discrete error state Jacobians 
    commondir = joinpath(@__DIR__, "..", "common")
    E = error_state_jacobian(xhover)
    A = E'ForwardDiff.jacobian(_x -> dynamics_rk4(_x, uhover, dt), xhover) * E
    B = E'ForwardDiff.jacobian(_u -> dynamics_rk4(xhover, _u, dt), uhover)

    # Cost
    Qk = [1.1; 1.1; 10; fill(1.0, 3); fill(0.1, 3); fill(1.0, 3)]
    qk = zeros(12)
    Qf = Qd * 100
    qf = copy(qk)
    Rk = fill(1e-3, 4)
    rk = zeros(4)

    # Build cost 
    P = blockdiag(kron(sparse(I,N-1,N-1), Diagonal(Qk)), Diagonal(Qf), kron(sparse(I,N-1,N-1), Diagonal(Rk)))

    
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

Base.@kwdef mutable struct SimOpts
    recvtimeout_ms::Int = 100
end

struct Simulator
    buf_out::ZMQ.Message
    buf_in::ZMQ.Message
    ctx::ZMQ.Context
    pub::ZMQ.Socket
    sub::ZMQ.Socket
    vis::Visualizer
    xhist::Vector{Vector{Float64}}
    uhist::Vector{Vector{Float64}}
    thist::Vector{Float64}
    msg_meas::Vector{NamedTuple{(:tsim, :twall, :y),Tuple{Float64,Float64,MeasurementMsg}}}
    msg_data::Vector{NamedTuple{(:tsim, :twall, :xu),Tuple{Float64,Float64,StateControlMsg}}}
    pose_queue::Queue{PoseMsg}
    stats::Dict{String,Any}
    opts::SimOpts
end

function Simulator(pub_port=5555, sub_port=5556)
    buf_out = ZMQ.Message(msgsize(MeasurementMsg))
    buf_in = ZMQ.Message(2 * msgsize(StateControlMsg))
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
    RobotMeshes.setdrone!(vis["truth"])
    RobotMeshes.setdrone!(vis["estimate"], color=RGBA(1.0, 0.0, 0.0, 0.5))
    MeshCat.setvisible!(vis["estimate"], false)

    # logs
    xhist = Vector{Float64}[]
    uhist = Vector{Float64}[]
    thist = Vector{Float64}()
    msg_meas = Vector{NamedTuple{(:tsim, :twall, :y),Tuple{Float64,Float64,MeasurementMsg}}}()
    msg_data = Vector{NamedTuple{(:tsim, :twall, :xu),Tuple{Float64,Float64,StateControlMsg}}}()
    pose_queue = Queue{PoseMsg}()
    stats = Dict{String,Any}()
    opts = SimOpts()
    Simulator(buf_out, buf_in, ctx, pub, sub, vis, xhist, uhist, thist, msg_meas, msg_data, pose_queue, stats, opts)
end

function reset!(sim::Simulator; approx_size=10_000, visualize=:truth)
    empty!(sim.msg_meas)
    empty!(sim.msg_data)
    empty!(sim.xhist)
    empty!(sim.uhist)
    empty!(sim.thist)
    empty!(sim.stats)
    empty!(sim.pose_queue)

    # Flush out publishers / subscribers
    buf = ZMQ.Message(100)
    # while ZMQ.msg_recv(sim.sub, buf, ZMQ.ZMQ_DONTWAIT) >= 0
    # end

    # Initialize logs
    xhat = SVector{13,Float32}[]
    latency = Float64[]
    sizehint!(latency, approx_size)
    sizehint!(xhat, approx_size)
    sim.stats["latency"] = latency
    sim.stats["xhat"] = xhat
    sim.stats["imu_messages_sent"] = 0

    return sim
    if visualize == :truth
        MeshCat.setvisible!(sim.vis["truth"], true)
        MeshCat.setvisible!(sim.vis["estimate"], false)
    elseif visualize == :estimate
        MeshCat.setvisible!(sim.vis["truth"], false)
        MeshCat.setvisible!(sim.vis["estimate"], true)
    elseif visualize == :all
        MeshCat.setvisible!(sim.vis["truth"], true)
        MeshCat.setvisible!(sim.vis["estimate"], true)
    end
    sim
end

function finish(sim::Simulator)
    ZMQ.close(sim.pub)
    ZMQ.close(sim.sub)
end

function runsim(sim::Simulator, x0; dt=0.01, tf=Inf, visualize=:none, kwargs...)
    x = copy(x0)
    t = 0.0
    freq = 1 / dt  # Hz
    lrl = LoopRateLimiter(10)
    u = trim_controls()

    approx_size = tf < Inf ? round(Int, tf / dt) : 10_000
    reset!(sim; approx_size, visualize)

    t_start = time()
    push!(sim.stats["xhat"], SVector{13,Float32}(x))
    while t < tf
        startloop(lrl)

        step!(sim, x, u, t, dt; t_start, visualize, kwargs...)

        println("time = ", t, ", z = ", x[3])
        t += dt
        # sleep(lrl)
    end
    if !isempty(sim.stats["latency"])
        latency = sim.stats["latency"]
        @printf("Latency: %.3f ± %.3f ms\n", mean(latency) * 1000, std(latency) * 1000)
    end
end

function step!(sim::Simulator, x, u, t, dt; t_start=time(), visualize=:none, send_measurement=false, imu_per_pose=1, pose_delay=0, send_ground_truth=false)
    # Initialize state estimate
    xhat = sim.stats["xhat"][end]

    # Get measurement
    y = getmeasurement(sim, x, u, t)

    # Send measurement
    if send_measurement
        imu_messages_sent = sim.stats["imu_messages_sent"]::Int
        tsend = time() - t_start

        if send_ground_truth
            sendmessage(sim, y, t)
            if imu_per_pose > 1 || pose_delay > 0
                @warn "Ignoring imu_per_pose and pose_delay settings when sending ground truth."
            end
        else
            y_pose = PoseMsg(y.x, y.y, y.z, y.qw, y.qx, y.qy, y.qz)
            y_imu = IMUMeasurementMsg(y.ax, y.ay, y.az, y.wx, y.wy, y.wz)
            push!(sim.msg_meas, (; tsim=t, twall=tsend, y))

            if imu_messages_sent % imu_per_pose == 0
                enqueue!(sim.pose_queue, y_pose)
                if length(sim.pose_queue) > pose_delay
                    y_pose_delayed = dequeue!(sim.pose_queue)
                    # println("Sent pose message")
                    sendmessage(sim, y_pose_delayed, tsend)
                    sleep(0.001)  # wait a little bit before sending the imu message
                end
            end

            sendmessage(sim, y_imu, t)
            imu_messages_sent += 1
            sim.stats["imu_messages_sent"] = imu_messages_sent
        end
    end

    # Get response from onboard computer
    statecontrol = getdata(sim, t)
    trecv = time() - t_start
    if !isnothing(statecontrol)
        push!(sim.msg_data, (; tsim=t, twall=trecv, xu=statecontrol))
        if send_measurement
            push!(sim.stats["latency"], trecv - tsend)
        end
        xhat = getstate(statecontrol)
        push!(sim.stats["xhat"], xhat)
        u .= getcontrol(statecontrol)
        # @show u
    end

    # Propagate dynamics
    push!(sim.xhist, copy(x))
    push!(sim.uhist, copy(u))
    push!(sim.thist, t)
    x .= dynamics_rk4(x, u, dt)

    # Visualize
    if visualize in (:truth, :all)
        RobotMeshes.visualize!(sim.vis["truth"], sim, x)
    elseif visualize in (:estimate, :all)
        RobotMeshes.visualize!(sim.vis["estimate"], sim, xhat)
    end
    x
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
        x[8], x[9], x[10],
        xdot[8], xdot[9], xdot[10],
        x[11], x[12], x[13]
    )
end

function getdata(sim::Simulator, t)
    # TODO: add noise
    buf = sim.buf_in
    tstart = time_ns()
    bytes_read = 0
    while (time_ns() - tstart < sim.opts.recvtimeout_ms * 1e6)
        bytes_read = ZMQ.msg_recv(sim.sub, buf, ZMQ.ZMQ_DONTWAIT)
        if bytes_read > 0
            break
        end
    end
    ZMQ.getproperty(sim.sub, :events)
    if bytes_read >= msgsize(StateControlMsg)
        return StateControlMsg(buf)
    else
        @warn "Didn't receive any data before timing out. Is the interface process running?"
        return nothing
    end
end

function sendmeasurement(sim::Simulator, y::MeasurementMsg, t)
    zmsg = ZMQ.Message(msgsize(y))
    copyto!(zmsg, y)
    ZMQ.send(sim.pub, zmsg)
end

function sendmessage(sim::Simulator, y, t)
    zmsg = ZMQ.Message(msgsize(y))
    copyto!(zmsg, y)
    ZMQ.send(sim.pub, zmsg)
end