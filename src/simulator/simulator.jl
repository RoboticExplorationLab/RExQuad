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
include("estimator.jl")

speye(n) = spdiagm(ones(n))

Base.@kwdef mutable struct SimOpts
    recvtimeout_ms::Int = 100
    mocap_delay::Int = 0  # actual delay of mocap data
    imu_per_pose::Int = 1
    delay_comp::Int = 0   # approximate delay used by estimator
    imu_bias::Vector{Float64} = 0.1 * randn(6)
    Wf::Diagonal{Float64,Vector{Float64}} = 0.0001*I(6)  # mocap (measurement) covariance
    Vf::Diagonal{Float64,Vector{Float64}} = Diagonal([fill(0.0001, 9); fill(1e-6, 6)])  # imu (process) covariance
end


struct Simulator{C,F}
    ctrl::C
    filter::F
    vis::Visualizer
    xhist::Vector{Vector{Float64}}   # true state history
    uhist::Vector{Vector{Float64}}   # control history
    x̂hist::Vector{Vector{Float64}}   # state prediction history
    xfhist::Vector{Vector{Float64}}  # filter state history
    Pfhist::Vector{Matrix{Float64}}  # state covariance history
    xdhist::Vector{Vector{Float64}}  # delayed filter state history
    Pdhist::Vector{Matrix{Float64}}  # delayed state covariance history
    imuhist::Vector{Vector{Float64}}    # imu measurement history
    mocaphist::Vector{Pair{Float64,Vector{Float64}}}  # mocap measurement history
    thist::Vector{Float64}

    msg_meas::Vector{NamedTuple{(:tsim, :twall, :y),Tuple{Float64,Float64,MeasurementMsg}}}
    msg_data::Vector{NamedTuple{(:tsim, :twall, :xu),Tuple{Float64,Float64,StateControlMsg}}}
    mocap_queue::Queue{Vector{Float64}}
    stats::Dict{String,Any}
    opts::SimOpts
end

function Simulator(pub_port=5555, sub_port=5556)
    ctrl = ZMQController(pub_port, sub_port)
    Simulator(ctrl)
end

function Simulator(ctrl::C, filter::F) where {C,F}
    vis = Visualizer()
    RobotMeshes.setdrone!(vis["truth"])
    RobotMeshes.setdrone!(vis["estimate"], color=RGBA(1.0, 0.0, 0.0, 0.5))
    MeshCat.setvisible!(vis["estimate"], false)

    # logs
    xhist = Vector{Float64}[]
    uhist = Vector{Float64}[]
    x̂hist = Vector{Float64}[]
    xfhist = Vector{Float64}[]
    Pfhist = Matrix{Float64}[]
    xdhist = Vector{Float64}[]
    Pdhist = Matrix{Float64}[]
    imuhist = Vector{Float64}[]
    mocaphist = Pair{Float64,Vector{Float64}}[]
    thist = Vector{Float64}()
    msg_meas = Vector{NamedTuple{(:tsim, :twall, :y),Tuple{Float64,Float64,MeasurementMsg}}}()
    msg_data = Vector{NamedTuple{(:tsim, :twall, :xu),Tuple{Float64,Float64,StateControlMsg}}}()
    mocap_queue = Queue{Vector{Float64}}()
    stats = Dict{String,Any}()
    opts = SimOpts()

    Simulator{C,F}(ctrl, filter, vis, xhist, uhist, x̂hist, xfhist, Pfhist, xdhist, Pdhist, 
        imuhist, mocaphist, thist, msg_meas, msg_data, mocap_queue, stats, opts)
end

function reset!(sim::Simulator; approx_size=10_000, visualize=:truth)
    empty!(sim.msg_meas)
    empty!(sim.msg_data)
    empty!(sim.xhist)
    empty!(sim.uhist)
    empty!(sim.x̂hist)
    empty!(sim.xfhist)
    empty!(sim.Pfhist)
    empty!(sim.xdhist)
    empty!(sim.Pdhist)
    empty!(sim.imuhist)
    empty!(sim.mocaphist)
    empty!(sim.mocap_queue)
    empty!(sim.thist)
    empty!(sim.stats)

    # # Flush out publishers / subscribers
    # buf = ZMQ.Message(100)
    # # while ZMQ.msg_recv(sim.sub, buf, ZMQ.ZMQ_DONTWAIT) >= 0
    # # end

    # Initialize logs
    latency = Float64[]
    sizehint!(sim.xhist, approx_size)
    sizehint!(sim.uhist, approx_size)
    sizehint!(sim.x̂hist, approx_size)
    sizehint!(sim.xfhist, approx_size)
    sizehint!(sim.Pfhist, approx_size)
    sizehint!(sim.xdhist, approx_size)
    sizehint!(sim.Pdhist, approx_size)
    sizehint!(latency, approx_size)
    sim.stats["latency"] = latency
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

function initialize!(sim::Simulator, x0, xhat0, dt, tf)
    approx_size = tf < Inf ? round(Int, tf / dt) : 10_000
    reset!(sim; approx_size)

    # Initialize filter
    initialize!(sim.filter, x0, Wf=sim.opts.Wf, Vf=sim.opts.Vf, delay_comp=sim.opts.delay_comp)

    # Set initial state
    push!(sim.xhist, x0)
    push!(sim.thist, 0)

    # Set initial state estimate
    push!(sim.x̂hist, xhat0) 
    push!(sim.xfhist, [xhat0[1:10]; zeros(6)])  # set bias estimate to 0
    push!(sim.Pfhist, I(15))                    # set initial state covariance
    push!(sim.xdhist, copy(sim.xfhist[end]))
    push!(sim.Pdhist, I(15))                    # set initial state covariance
end

function runsim(sim::Simulator, x0; xhat0=copy(x0), dt=0.01, tf=Inf, visualize=:none, kwargs...)
    x = copy(x0)
    t = 0.0
    freq = 1 / dt  # Hz
    lrl = LoopRateLimiter(10)
    u = trim_controls()

    initialize!(sim, x0, xhat0, dt, tf)

    t_start = time()
    while t < tf
        startloop(lrl)

        step!(sim, t, dt; t_start, visualize, kwargs...)

        println("time = ", t, ", z = ", sim.xhist[end][3])
        t += dt
        # sleep(lrl)
    end
    if !isempty(sim.stats["latency"])
        latency = sim.stats["latency"]
        @printf("Latency: %.3f ± %.3f ms\n", mean(latency) * 1000, std(latency) * 1000)
    end
end

function step!(sim::Simulator, t, dt; t_start=time(), visualize=:none, send_measurement=false, imu_per_pose=1, pose_delay=0, send_ground_truth=false)
    x = sim.xhist[end]    # true state
    x̂ = sim.x̂hist[end]    # state estimate
    xf = sim.xfhist[end]  # filter state
    Pf = sim.Pfhist[end]   # filter state covariance
    xd = sim.xdhist[end]   # delayed filter state
    Pd = sim.Pdhist[end]   # delayed filter state covariance
    delay = min(sim.opts.delay_comp, length(sim.xhist))

    # Evaluate controller
    u = getcontrol(sim.ctrl, x̂, [], t)
    # u = getcontrol(sim.ctrl, x, [], t)

    # # Get measurement
    # y = getmeasurement(sim, x, u, t)

    # Use control to get simulated IMU measurement
    y_imu = imu_measurement(sim, x, u)
    push!(sim.imuhist, y_imu)


    # Get (delayed) mocap measurement
    y_mocap = mocap_measurement(sim, x)
    if !isnothing(y_mocap)
        push!(sim.mocaphist, t=>y_mocap)
    end

    # if !isnothing(y_mocap)
    #     push!(sim.mocaphist, t=>y_mocap)

    #     # Advance the delayed measurement using the IMU measurement from that time
    #     y_imu_delayed = sim.imuhist[end-delay]
    #     xpred, Ppred = filter_state_prediction(sim, xd, y_imu_delayed, Pd, dt)

    #     # Update the delayed filter estimate using mocap measurement
    #     xd, Pd = filter_mocap_update(sim, xpred, Ppred, y_mocap)
    # end
    # push!(sim.xdhist, xd)
    # push!(sim.Pdhist, Pd)

    # # Use history of IMU data to predict the state at the current time
    # xf .= xd
    # Pf .= Pd
    # for i = 1:delay-1
    #     y_imu_delayed = sim.imuhist[end-delay+i]
    #     xf,Pf = filter_state_prediction(sim, xf, y_imu_delayed, Pf, dt)
    # end
    # push!(sim.xfhist, xf)
    # push!(sim.Pfhist, Pf)

    # # Set state estimate to prediction from IMU
    # # y_gyro = gyro_measurement(sim, xn)
    # y_gyro = y_imu[4:6]     # get current angvel from IMU
    # b_gyro = xf[14:16]      # predicted gyro bias
    # ωhat = y_gyro - b_gyro  # predicted angular velocity
    # xhat = [xf[1:10]; ωhat]
    xhat = get_state_estimate!(sim.filter, y_imu, y_mocap, dt)
    push!(sim.x̂hist, copy(xhat))

    # Propagate state
    xn = dynamics_rk4(x, u, dt)
    push!(sim.xhist, xn)
    push!(sim.thist, t)

    # Visualize
    # if visualize in (:truth, :all)
    #     RobotMeshes.visualize!(sim.vis["truth"], sim, x)
    # elseif visualize in (:estimate, :all)
    #     RobotMeshes.visualize!(sim.vis["estimate"], sim, xhat)
    # end
    # x
end

function getcontrol(sim::Simulator, x, t)
    # TODO: Get this from ZMQ
    u = trim_controls() .+ 0.1
    return ControlMsg(u)
end

function getmeasurement(sim::Simulator, x, u, t)
    y_accel = 1/quad_mass * [zeros(2,4); fill(quad_motor_kf,1,4)]
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

function imu_measurement(sim::Simulator, x, u)
    Kf = [zeros(2,4); fill(quad_motor_kf,1,4)]
    Bf = [0;0;4*quad_motor_bf]
    bias = sim.opts.imu_bias
    Vf = sim.opts.Vf
    accel = (Kf * u + Bf) / quad_mass
    gyro = x[11:13]
    [accel; gyro]  + bias + sqrt(Vf)[1:6,1:6]*randn(6)  # this index seems off?
end

function gyro_measurement(sim::Simulator, x)
    bias = sim.opts.imu_bias
    Vf = sim.opts.Vf
    gyro = x[11:13]
    gyro + bias[4:6] + sqrt(Vf)[4:6,4:6]*randn(3)
end

function mocap_measurement(sim::Simulator, x)
    Wf = sim.opts.Wf
    errmap = Rotations.CayleyMap()
    noise = sqrt(Wf) * randn(6)
    q = QuatRotation(x[4:7])
    position = x[1:3] + noise[1:3] 
    attitude = q * errmap(noise[4:6]) 
    y_mocap = [position; Rotations.params(attitude)]

    # Handle pose delay
    enqueue!(sim.mocap_queue, y_mocap)
    if length(sim.mocap_queue) > sim.opts.mocap_delay
        return dequeue!(sim.mocap_queue)
    else
        return nothing
    end
end

function filter_state_prediction(sim::Simulator, xf,uf,Pf,h)
    Vf = sim.opts.Vf

    rf = xf[1:3]    # inertial frame
    qf = xf[4:7]    # body to inertial
    vf = xf[8:10]   # body frame
    ab = xf[11:13]  # accel bias
    ωb = xf[14:16]  # gyro bias
    
    af = uf[1:3]    # body frame acceleration (from IMU)
    ωf = uf[4:6]    # body frame linear velocity (from IMU)
    
    ahat = af - ab  # predicted acceleration (with bias removed)
    ωhat = ωf - ωb  # predicted angular velocity (with bias removed)

    Qf = QuatRotation(qf)
    g = SA[0,0,9.81]

    # IMU Prediction
    errmap = Rotations.CayleyMap()
    y = errmap(-0.5 * h * ωhat)       # rotation from this time step to the next
    rp = rf + h * Qf * vf             # position prediction
    qp = Qf * errmap(0.5 * h * ωhat)  # attitude prediction
    vpk = vf + h * (ahat - Qf\g)      # velocity in old body frame
    vp = y * vpk                      # velocity in new body frame
    xp = [rp; Rotations.params(qp); vp; ab; ωb]

    # Jacobian
    H = Rotations.hmat()
    L = Rotations.lmult
    R = Rotations.rmult
    G(q) = L(q) * H
    Y = Matrix(y)

    # Derivative of Q(q)*v wrt q
    dvdq = Rotations.∇rotate(Qf, vf) * G(Qf)

    # Derivative of Q(q)*g wrt g
    dgdq = Rotations.∇rotate(Qf', g) * G(Qf)

    # Derivative of vp wrt ωb
    dvdb = 0.5*h*Rotations.∇rotate(y, vpk) * Rotations.jacobian(errmap, -0.5 * h * ωhat)

    # Derivative of qp wrt ωb
    dqdb = -0.5*h*G(qp)'L(Qf) * Rotations.jacobian(errmap, 0.5 * h * ωhat)

    # Jacobian of prediction (xp wrt xf)
    #    r           q          v           ab          ωb
    Af = [
        I(3)       h*dvdq     h*Matrix(Qf) zeros(3,3) zeros(3,3)
        zeros(3,3) Y          zeros(3,3)   zeros(3,3) dqdb
        zeros(3,3) -h*y*dgdq  Y            -h*Y       dvdb
        zeros(6,3) zeros(6,3) zeros(6,3)            I(6)
    ]
    Pp = Af*Pf*Af' + Vf

    xp, Pp
end

function filter_mocap_update(sim::Simulator, xf, Pf, y_mocap)
    errmap = Rotations.CayleyMap()
    Wf = sim.opts.Wf
    Cf = Matrix(I,6,15)   # measurement Jacobian

    rf = xf[1:3]    # inertial frame
    qf = xf[4:7]    # body to inertial
    vf = xf[8:10]   # body frame
    ab = xf[11:13]  # acceleration bias
    ωb = xf[14:16]  # IMU bias

    rm = y_mocap[1:3]
    qm = y_mocap[4:7]

    Qf = QuatRotation(qf)
    Qm = QuatRotation(qm)

    z = [rm-rf; inv(errmap)(Qf'Qm)]  # innovation
    S = Cf * Pf * Cf' + Wf
    Lf = (Pf*Cf')/S  # Kalman filter gain
    Δx = Lf * z
    xn = [
        rf + Δx[1:3]; 
        Rotations.params(Qf*errmap(Δx[4:6])); 
        vf + Δx[7:9]; 
        ab + Δx[10:12]; 
        ωb + Δx[13:15]
    ]
    Pn = (I-Lf*Cf)*Pf*(I-Lf*Cf)' + Lf * Wf * Lf'
    return xn,Pn
end