#Quaternion stuff
function hat(v)
    return [0 -v[3] v[2];
            v[3] 0 -v[1];
            -v[2] v[1] 0]
end

function L(q)
    s = q[1]
    v = q[2:4]
    L = [s    -v';
            v  s*I+hat(v)]
    return L
end

T = Diagonal([1; -ones(3)])
H = [zeros(1,3); I]

function qtoQ(q)
    return H'*T*L(q)*T*L(q)*H
end

function G(q)
    G = L(q)*H
end

function rptoq(ϕ)
    (1/sqrt(1+ϕ'*ϕ))*[1; ϕ]
end

function qtorp(q)
    q[2:4]/q[1]
end

#Quadrotor parameters
m = 0.5
ℓ = 0.1750
J = [0.01566089 0.00000318037 0; 0.00000318037 0.01562078 0; 0 0 0.02226868]
g = 9.81
kt=1.0
km=0.0245

h = 0.02 #50 Hz

Nx = 13     # number of states (quaternion)
Nx̃ = 12     # number of states (linearized)
Nu = 4     # number of controls
Tfinal = 5.0 # final time
Nt = Int(Tfinal/h)+1    # number of time steps
thist = Array(range(0,h*(Nt-1), step=h));

function E(q)
    E = BlockDiagonal([1.0*I(3), G(q), 1.0*I(6)])
end

function quad_dynamics(x,u)
    r = x[1:3]
    q = x[4:7]
    v = x[8:10]
    ω = x[11:13]
    Q = qtoQ(q)
    
    ṙ = Q*v
    q̇ = 0.5*L(q)*H*ω
    
    v̇ = Q'*[0; 0; -g] + (1/m)*[zeros(2,4); kt*ones(1,4)]*u - hat(ω)*v
    
    ω̇ = J\(-hat(ω)*J*ω + [0 ℓ*kt 0 -ℓ*kt; -ℓ*kt 0 ℓ*kt 0; km -km km -km]*u)
    
    return [ṙ; q̇; v̇; ω̇]
end

#for simulation purposes
function quad_dynamics_rk4(x,u)
    #RK4 integration with zero-order hold on u
    f1 = quad_dynamics(x, u)
    f2 = quad_dynamics(x + 0.5*h*f1, u)
    f3 = quad_dynamics(x + 0.5*h*f2, u)
    f4 = quad_dynamics(x + h*f3, u)
    xn = x + (h/6.0)*(f1 + 2*f2 + 2*f3 + f4)
    xn[4:7] .= xn[4:7]/norm(xn[4:7]) #re-normalize quaternion
    return xn
end

#Initial Conditions
uhover = (m*g/4)*ones(4)
r0 = [0.0; 0; 1.0]
q0 = [1.0; 0; 0; 0]
v0 = zeros(3)
ω0 = zeros(3)
x0 = [r0; q0; v0; ω0]
x̃0 = [r0; 0; 0; 0; v0; ω0];

#Linearize dynamics about hover
A = ForwardDiff.jacobian(x->quad_dynamics_rk4(x,uhover),x0)
B = ForwardDiff.jacobian(u->quad_dynamics_rk4(x0,u),uhover);

#Reduced system
Ã = Array(E(q0)'*A*E(q0))
B̃ = Array(E(q0)'*B);

# Cost weights
Q = Array(I(Nx̃));
R = Array(.1*I(Nu));

#LQR Gain

S = dare(Ã,B̃,Q,R) #cost-to-go
K = dlqr(Ã,B̃,Q,R);

#LQR Feedback controller
function lqr_controller(x)
    
    q0 = x0[4:7]
    q = x[4:7]
    ϕ = qtorp(L(q0)'*q)
    
    Δx̃ = [x[1:3]-r0; ϕ; x[8:10]-v0; x[11:13]-ω0]
    
    u = uhover - K*Δx̃
end

#MPC Setup
Nh = 50 #MPC horizon length (number of time steps)
model = RD.LinearModel(Ã, B̃;dt = h)
Qf = S #use LQR cost-to-go as terminal cost
objective = LQRObjective(Q, R, Qf, x̃0, Nh)

# Constraints
constraints = ConstraintList(Nx̃, Nu, Nh)
bound = BoundConstraint(Nx̃, Nu, u_min=-uhover, u_max=uhover)
add_constraint!(constraints, bound, 1:Nh)


problem = Problem(model, objective, x̃0, (Nh-1)*h, x0=x̃0, constraints=constraints, integration=RD.PassThrough)
solver = ALTROSolver(problem)

tol=1e-4
set_options!(solver,
    show_summary=false,
    projected_newton=false,
    constraint_tolerance=tol,
    cost_tolerance=tol,
    cost_tolerance_intermediate=tol,
    penalty_initial=10.,
    penalty_scaling=10.,
)

# Solve initial problem  ("cold start")
solve!(solver);

#MPC Feedback controller
function mpc_controller(x)
    
    q0 = x0[4:7]
    q = x[4:7]
    ϕ = qtorp(L(q0)'*q)
    
    Δx̃ = [x[1:3]-r0; ϕ; x[8:10]-v0; x[11:13]-ω0]
    
    problem.x0 .= Δx̃
    solve!(solver)
    Δu = controls(solver)
    
    u = uhover + Δu[1]

    return u
end