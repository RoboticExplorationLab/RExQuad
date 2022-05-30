using JSON
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

# Hover state
xe = [0;0;1; 1;0;0;0; zeros(6)]
ue = trim_controls()
dt = 0.01  # 100 Hz
xg = [0;0;1.5; 1; zeros(3); zeros(6)]  # goal state
dxg = state_error(xg, xe)              # error state from equilibrium to goal

# Get discrete error state Jacobians about equilibrium
commondir = joinpath(@__DIR__, "..", "common")
E = error_state_jacobian(xhover)
A = E'ForwardDiff.jacobian(_x->dynamics_rk4(_x, uhover, dt), xe)*E
B = E'ForwardDiff.jacobian(_u->dynamics_rk4(xhover, _u, dt), ue)

# Cost
Qk = spdiagm([1.1;1.1;10; fill(1.0, 3); fill(0.1,3); fill(1.0,3)])
qk = -Qk*dxg
Qf = Qk * 100 
qf = -Qf*dxg  
Rk = spdiagm(fill(1e-3, 4))
rk = zeros(4)

N = 11
P = blockdiag(kron(sparse(I,N-1,N-1), Qk), Qf, kron(sparse(I,N-1,N-1), Rk))
q = [kron(ones(N-1), qk); qf; kron(ones(N-1), rk)]
A = 

open(joinpath(commondir, "mpc_data.json"), "w") do f
    data = Dict(
        "A"=>A', "B"=>B',  # store transpose for easy Python loading
        "Qk"=>Qk, "Rk"=>Rk, "Qf"=>Qf, 
        "qk"=>qk, "rk"=>rk, "qf"=>qf,
    )  
    indent = 2
    JSON.print(f, data, indent)
end

K = generate_LQR_hover_gains(xhover, uhover, dt; Qd, Rd)
dx = [0;0;1; zeros(9)]
K*dx + uhover

open(joinpath(commondir, "lqr_constants.hpp"), "w") do f
    write(f, print_gains(K, xhover, uhover))
end

