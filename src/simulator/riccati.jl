struct RiccatiSolver{T}
    Q::Vector{Diagonal{T,Vector{T}}}
    q::Vector{Vector{T}}
    R::Vector{Diagonal{T,Vector{T}}}
    r::Vector{Vector{T}}
    A::Vector{Matrix{T}}
    B::Vector{Matrix{T}}
    f::Vector{Vector{T}}
    xe::Vector{T}
    ue::Vector{T}
    x0::Vector{T}
    u0::Vector{T}
    xf::Vector{T}

    K::Vector{Matrix{T}}
    d::Vector{Vector{T}}
    P::Vector{Matrix{T}}
    p::Vector{Vector{T}}
    X::Vector{Vector{T}}
    U::Vector{Vector{T}}
    λ::Vector{Vector{T}}

    state_error::Function
end

function RiccatiSolver(n, m, N, n0=n, state_error = (x,xe)->x - xe)
    Q = [Diagonal(ones(n)) for k in 1:N]
    R = [Diagonal(ones(m)) for k in 1:(N - 1)]
    q = [zeros(n) for k in 1:N]
    r = [zeros(m) for k in 1:(N - 1)]
    A = [zeros(n, n) for k in 1:(N - 1)]
    B = [zeros(n, m) for k in 1:(N - 1)]
    f = [zeros(n) for k in 1:(N - 1)]
    xe = zeros(n0)
    ue = zeros(m)
    x0 = zeros(n)
    u0 = zeros(m)
    xf = zeros(n)

    K = [zeros(m, n) for k in 1:(N - 1)]
    d = [zeros(m) for k in 1:(N - 1)]
    P = [zeros(n, n) for k in 1:N]
    p = [zeros(n) for k in 1:N]
    X = [zeros(n) for k in 1:N]
    U = [zeros(m) for k in 1:(N - 1)]
    λ = [zeros(n) for k in 1:N]
    return RiccatiSolver(Q, q, R, r, A,B, f, xe, ue, x0, u0, xf, K, d, P, p, X, U, λ, state_error)
end

finish(::RiccatiSolver) = nothing

horizonlength(solver::RiccatiSolver) = length(solver.Q)
statedim(solver::RiccatiSolver) = length(solver.X[1])
controldim(solver::RiccatiSolver) = length(solver.U[1]) 

function setdynamics!(solver::RiccatiSolver, A,B,f, xe,ue) 
    N = horizonlength(solver)
    for k = 1:N-1
        solver.A[k] .= A
        solver.B[k] .= B
        solver.f[k] .= f
    end
    solver.xe .= xe
    solver.ue .= ue
end

function setcost!(solver::RiccatiSolver, Q,R,Qf,xf,u0)
    N = horizonlength(solver)
    for k = 1:N-1
        solver.Q[k] .= Q
        solver.R[k] .= R
        solver.r[k] .= -R*u0
    end
    let k = N
        solver.Q[k] .= Qf
    end
    setgoalstate!(solver, xf)
end

function setinitialstate!(solver::RiccatiSolver, x0)
    solver.x0 .= x0
end

function setgoalstate!(solver::RiccatiSolver, xf)
    N = horizonlength(solver)
    for k = 1:N
        Q = solver.Q[k]
        solver.q[k] .= -Q*xf
    end
    solver.xf .= xf
    nothing
end

function backwardpass!(solver::RiccatiSolver)
    A, B = solver.A, solver.B
    Q, q = solver.Q, solver.q
    R, r = solver.R, solver.r
    # K,d = solver.K, solver.d
    P, p = solver.P, solver.p
    N = length(Q)

    P[N] .= Q[N]
    p[N] .= q[N]

    for k in reverse(1:(N - 1))
        P′ = P[k + 1]
        Ak = A[k]
        Bk = B[k]

        Qx = q[k] + Ak' * (p[k + 1])
        Qu = r[k] + Bk' * (p[k + 1])
        Qxx = Q[k] + Ak'P′ * Ak
        Quu = R[k] + Bk'P′ * Bk
        Qux = Bk'P′ * Ak

        cholQ = cholesky(Symmetric(Quu))
        K = -(cholQ \ Qux)
        d = -(cholQ \ Qu)

        P[k] .= Qxx .+ K'Quu * K .+ K'Qux .+ Qux'K
        p[k] = Qx .+ K'Quu * d .+ K'Qu .+ Qux'd
        solver.K[k] .= K
        solver.d[k] .= d
    end
end

function forwardpass!(solver::RiccatiSolver)
    A, B = solver.A, solver.B
    X, U, λ = solver.X, solver.U, solver.λ
    K, d = solver.K, solver.d
    X[1] = solver.x0
    N = length(X)
    for k in 1:(N - 1)
        λ[k] = solver.P[k] * X[k] .+ solver.p[k]
        U[k] = K[k] * X[k] + d[k]
        X[k + 1] = A[k] * X[k] .+ B[k] * U[k] #.+ f[k]
    end
    return λ[N] = solver.P[N] * X[N] .+ solver.p[N]
end

function solve!(solver::RiccatiSolver)
    backwardpass!(solver)
    forwardpass!(solver)
    return nothing
end

function getcontrol(solver::RiccatiSolver, x, y, t)
    dx0 = solver.state_error(x, solver.xe)
    setinitialstate!(solver, dx0)
    solve!(solver)
    du = solver.U[1]
    du .+ solver.ue
end

