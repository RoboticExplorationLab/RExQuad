using ZMQ
using Sockets
using MeshCat
using StaticArrays
using Statistics
using Printf
using Colors
using DataStructures
using SparseArrays
using OSQP

include("dynamics.jl")
include("ratelimiter.jl")
include("visualization.jl")
include("messages.jl")
include("controllers.jl")

speye(n) = spdiagm(ones(n))


struct Simulator{C}
    ctrl::C
    vis::Visualizer
    xhist::Vector{Vector{Float64}}
    uhist::Vector{Vector{Float64}}
    thist::Vector{Float64}
    msg_meas::Vector{NamedTuple{(:tsim, :twall, :y),Tuple{Float64,Float64,MeasurementMsg}}}
    msg_data::Vector{NamedTuple{(:tsim, :twall, :xu),Tuple{Float64,Float64,StateControlMsg}}}
    stats::Dict{String,Any}
    opts::SimOpts
end

function Simulator(pub_port=5555, sub_port=5556)
    ctrl = ZMQController(pub_port, sub_port)
    Simulator(ctrl)
end

function Simulator(ctrl::C) where C
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
    stats = Dict{String,Any}()
    opts = SimOpts()
    Simulator{C}(ctrl, vis, xhist, uhist, thist, msg_meas, msg_data, stats, opts)
end

function reset!(sim::Simulator; approx_size=10_000, visualize=:truth)
    empty!(sim.msg_meas)
    empty!(sim.msg_data)
    empty!(sim.xhist)
    empty!(sim.uhist)
    empty!(sim.thist)
    empty!(sim.stats)

    # # Flush out publishers / subscribers
    # buf = ZMQ.Message(100)
    # # while ZMQ.msg_recv(sim.sub, buf, ZMQ.ZMQ_DONTWAIT) >= 0
    # # end

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
    finish(sim.ctrl)
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
    if isempty(sim.stats["xhat"])
        push!(sim.stats["xhat"], x)
    end
    xhat = sim.stats["xhat"][end]

    # Get measurement
    y = getmeasurement(sim, x, u, t)

    # Evaluate controller
    u = getcontrol(sim.ctrl, x, y, t)

    # Propagate dynamics
    push!(sim.xhist, copy(x))
    push!(sim.uhist, copy(u))
    push!(sim.thist, t)
    x .= dynamics_rk4(x, u, dt)
    # dx = ctrl.Ad*state_error(x, ctrl.xeq) + ctrl.Bd*(u - ctrl.ueq)
    # x .= add_state(x, dx)

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
