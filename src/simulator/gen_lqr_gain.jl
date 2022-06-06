using JSON
using SparseArrays
using OSQP
include("dynamics.jl")

function dlqr(A,B,Q,R; max_iters=200, tol=1e-6, verbose=false)
    P = Matrix(copy(Q))
    n,m = size(B)
    K = zeros(m,n)
    K_prev = copy(K)
    
    for k = 1:max_iters
        K .= (R + B'P*B) \ (B'P*A)
        P .= Q + A'P*A - A'P*B*K
        if norm(K-K_prev,Inf) < tol
            verbose && println("Converged in $k iterations")
            return K
        end
        K_prev .= K
    end
    return K
end

function generate_LQR_hover_gains(xhover, uhover, dt;
    Qd = 1*ones(12),
    Rd = fill(0.1, 4),
    save_to_file::Bool = true,
)
    # Setting x₀ hover state
    xhover = copy(xhover)
    uhover = copy(uhover)

    A = ForwardDiff.jacobian(_x->dynamics_rk4(_x, uhover, dt), xhover)
    B = ForwardDiff.jacobian(_u->dynamics_rk4(xhover, _u, dt), uhover)

    E2 = error_state_jacobian(xhover)
    Ã = E2' * A * E2
    B̃ = E2' * B

    # Cost weights
    Q = Array(Diagonal(Qd))
    R = Array(Diagonal(Rd))

    # LQR Gain
    K = dlqr(Ã, B̃, Q, R)

    return K
end

function print_gains(K, xeq, ueq)
    Kinclude = "const float kFeedbackGain[$(length(K))] = {"
    Kinclude *= join(string.(K), ", ") * "};"
    xinclude = "const float kStateEquilibrium[13] = {";
    xinclude *= join(string.(xeq), ", ") * "};"
    uinclude = "const float kInputEquilibrium[4] = {";
    uinclude *= join(string.(ueq), ", ") * "};"
    """
    #pragma once
    namespace rexquad {
    $Kinclude
    $xinclude
    $uinclude
    }  // namespace rexquad
    """
end
speye(n) = spdiagm(ones(n))

## Hover state
xe = [0;0;1; 1;0;0;0; zeros(6)]        # equilibrium state (linearization point)
ue = trim_controls()
dt = 0.01  # 100 Hz
xg = [0;0;1.0; 1; zeros(3); zeros(6)]  # goal state
x0 = [0;0;1.0; 1;0;0;0; zeros(6)]     # initial state
dxg = state_error(xg, xe)              # error state from equilibrium to goal
dx0 = state_error(x0, xe)
n,m = 12,4

# Get discrete error state Jacobians about equilibrium
commondir = joinpath(@__DIR__, "..", "common")
E = error_state_jacobian(xe)
A = E'ForwardDiff.jacobian(_x->dynamics_rk4(_x, ue, dt), xe)*E
B = E'ForwardDiff.jacobian(_u->dynamics_rk4(xe, _u, dt), ue)

# Cost
Qk = spdiagm([0.5;0.5;10; fill(10.0, 3); fill(0.1,3); fill(1.0,3)])
qk = -Qk*dxg
Qf = Qk * 1000
qf = -Qf*dxg
Rk = spdiagm(fill(1e-3, 4))
rk = zeros(4)

N = 11
P = blockdiag(kron(speye(N-1), Qk), Qf, kron(speye(N-1), Rk))
q = [kron(ones(N-1), qk); qf; kron(ones(N-1), rk)]
Ax = kron(speye(N), -speye(n)) + kron(spdiagm(-1=>ones(N-1)), A)
Bu = kron(spdiagm(N,N-1,-1=>ones(N-1)), B)
Aeq = [Ax Bu]
leq = [-dx0; zeros((N-1)*n)]
ueq = copy(leq)

prob = OSQP.Model()
OSQP.setup!(prob; P, q, A=Aeq, l=leq, u=ueq)
res = OSQP.solve!(prob)
res.x[1:n]
res.x[(N+1)*n .+ (1:m)]

open(joinpath(commondir, "mpc_data.json"), "w") do f
    data = Dict(
        "A"=>A', "B"=>B',  # store transpose for easy Python loading
        "Qk"=>diag(Qk), "Rk"=>diag(Rk), "Qf"=>diag(Qf),
        "qk"=>qk, "rk"=>rk, "qf"=>qf,
        "xe"=>xe, "ue"=>ue, "xg"=>xg,
        "dx0"=>dx0, "dxg"=>dxg
    )  
    indent = 2
    JSON.print(f, data, indent)
end

K = generate_LQR_hover_gains(xe, ue, dt; Qd, Rd)
dx = [0;0;1; zeros(9)]
K*dx + ue

open(joinpath(commondir, "lqr_constants.hpp"), "w") do f
    write(f, print_gains(K, xe, ue))
end

