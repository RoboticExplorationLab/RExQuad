
struct OSQPController{T}
    # QP data
    P::SparseMatrixCSC{T,Int}
    q::Vector{T}
    A::SparseMatrixCSC{T,Int}
    l::Vector{T}
    u::Vector{T}
    prob::OSQP.Model

    # Problem data
    Qd::Vector{T}
    Rd::Vector{T}
    Qf::Vector{T}
    Ad::Matrix{T}
    Bd::Matrix{T}
    xeq::Vector{T}
    ueq::Vector{T}
    xg::Vector{T}
    dt::T
    N::Int
    function OSQPController(xeq, ueq, dt, N; xg=copy(xeq))
        n,m = 12,4
        x0 = copy(xeq)
        dxg = state_error(xg, xeq)
        dx0 = state_error(x0, xeq)

        # Calculate discrete Jacobians
        E = error_state_jacobian(xeq)
        Ad = E'ForwardDiff.jacobian(_x->dynamics_rk4(_x, ueq, dt), xeq)*E
        Bd = E'ForwardDiff.jacobian(_u->dynamics_rk4(xeq, _u, dt), ueq)

        # Cost
        Qk = spdiagm([1.1;1.1;10; fill(1.0, 3); fill(0.1,3); fill(1.0,3)])
        qk = -Qk*dxg
        Qf = Qk * 100 
        qf = -Qf*dxg  
        Rk = spdiagm(fill(1e-3, 4))
        rk = zeros(4)
        
        # Build QP 
        P = blockdiag(kron(speye(N-1), Qk), Qf, kron(speye(N-1), Rk))
        q = [kron(ones(N-1), qk); qf; kron(ones(N-1), rk)]
        Ax = kron(speye(N), -speye(n)) + kron(spdiagm(-1=>ones(N-1)), Ad)
        Bu = kron(spdiagm(N,N-1,-1=>ones(N-1)), Bd)
        A_eq = [Ax Bu]
        l_eq = [-dx0; zeros((N-1)*n)]
        u_eq = copy(l_eq)
        A = copy(A_eq)
        l = copy(l_eq)
        u = copy(u_eq)
        
        prob = OSQP.Model()
        OSQP.setup!(prob; P, q, A, l, u, verbose=0)
        
        T = eltype(xeq)
        new{T}(P,q,A,l,u,prob, diag(Qk), diag(Rk), diag(Qf), Ad, Bd, xeq, ueq, xg, dt, N)
    end
end

function update_initial_state!(ctrl::OSQPController, x0)
    n = 12
    dx0 = state_error(x0, ctrl.xeq)
    ctrl.l[1:n] .= .-dx0
    ctrl.u[1:n] .= .-dx0
    OSQP.update!(ctrl.prob, l=ctrl.l, u=ctrl.u)
end

function update_goal_state!(ctrl::OSQPController, xg)
    dxg = state_error(xg, ctrl.xeq)
    n = 12
    for k = 1:ctrl.N
        ix = (k-1)*n .+ (1:n)
        Q = k == ctrl.N ? Diagonal(ctrl.Qd) : Diagonal(ctrl.Qf)
        ctrl.q[ix] .= .-Q*dxg
    end
    OSQP.update!(ctrl.prob, q=ctrl.q)
end

function getcontrol(ctrl::OSQPController, x, y, t)
    n = 12
    m = 4
    N = ctrl.N
    update_initial_state!(ctrl, x)
    res = OSQP.solve!(ctrl.prob)
    return res.x[n*N .+ (1:m)] + ctrl.ueq
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

struct ZMQController
    ctx::ZMQ.Context
    pub::ZMQ.Socket
    sub::ZMQ.Socket
    buf_out::ZMQ.Message
    buf_in::ZMQ.Message
    uprev::Vector{Float64}
    pose_queue::Queue{PoseMsg}
    opts::Dict{Symbol,Real}
end
function ZMQController(pub_port=5555, sub_port=pub_port+1)
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

    opts = Dict{Symbol,Real}(:send_ground_truth=>false, :imu_per_pose=>1, :pose_delay=>0, 
                             :recvtimeout_ms=>200, :imu_messages_sent=>0)
    uprev = zeros(4)
    pose_queue = Queue{PoseMsg}()
    ZMQController(ctx, pub, sub, buf_out, buf_in, uprev, pose_queue, opts)
end

function getcontrol(ctrl::ZMQController, x, y, t)
    # Get options
    opts = ctrl.opts
    send_ground_truth = opts[:send_ground_truth]::Bool
    imu_per_pose = opts[:imu_per_pose]::Int
    pose_delay = opts[:pose_delay]::Int
    imu_messages_sent = opts[:imu_messages_sent]::Int
    uprev = ctrl.uprev

    # Send measurement
    if send_ground_truth
        sendmessage(ctrl, y, t)
        if imu_per_pose > 1 || pose_delay > 0
            @warn "Ignoring imu_per_pose and pose_delay settings when sending ground truth."
        end
    else
        y_pose = PoseMsg(y.x, y.y, y.z, y.qw, y.qx, y.qy, y.qz)
        y_imu = IMUMeasurementMsg(y.ax, y.ay, y.az, y.wx, y.wy, y.wz)

        if imu_messages_sent % imu_per_pose == 0
            enqueue!(ctrl.pose_queue, y_pose)
            if length(ctrl.pose_queue) > pose_delay
                y_pose_delayed = dequeue!(ctrl.pose_queue)
                # println("Sent pose message")
                sendmessage(ctrl, y_pose_delayed, t)
                sleep(0.001)  # wait a little bit before sending the imu message
            end
        end

        sendmessage(ctrl, y_imu, t)
        opts[:imu_messages_sent] += 1
    end

    # Get response from onboard computer
    statecontrol = getdata(ctrl, t)
    if !isnothing(statecontrol)
        uprev .= getcontrol(statecontrol)
    end
    return uprev
end

function getdata(ctrl::ZMQController, t)
    # TODO: add noise
    buf = ctrl.buf_in
    tstart = time_ns()
    bytes_read = 0
    while (time_ns() - tstart < ctrl.opts[:recvtimeout_ms] * 1e6)
        bytes_read = ZMQ.msg_recv(ctrl.sub, buf, ZMQ.ZMQ_DONTWAIT)
        if bytes_read > 0
            break
        end
    end
    ZMQ.getproperty(ctrl.sub, :events)
    if bytes_read >= msgsize(StateControlMsg)
        return StateControlMsg(buf)
    else
        @warn "Didn't receive any data before timing out. Is the interface process running?"
        return nothing
    end
end

# function sendmeasurement(ctrl::ZMQController, y::MeasurementMsg, t)
#     zmsg = ZMQ.Message(msgsize(y))
#     copyto!(zmsg, y)
#     ZMQ.send(ctrl.pub, zmsg)
# end

function sendmessage(ctrl::ZMQController, y, t)
    # TODO: avoid this dynamic memory allocation
    zmsg = ZMQ.Message(msgsize(y))
    copyto!(zmsg, y)
    ZMQ.send(ctrl.pub, zmsg)
end

function finish(ctrl::ZMQController)
    ZMQ.close(ctrl.pub)
    ZMQ.close(ctrl.sub)
end