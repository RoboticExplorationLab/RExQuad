const NUM_INPUT = 4
const NUM_STATE = 13
const NUM_ERR_STATE = 12

const QUAD_MASS = 1.776
const QUAD_J = SA[0.01566089     0.00000318037  0.0;
                  0.00000318037  0.01562078     0.0;
                  0.0            0.0            0.02226868]
const QUAD_J_INV = pinv(QUAD_J)
const QUAD_MOTOR_DIST = 0.28
const QUAD_KT = 0.28
const QUAD_KM = 0.28


function quad_dynamics(x::SVector{NUM_STATE}, u::SVector{NUM_INPUT})
    g = SA[0, 0, -9.81]

    p = SA[x[1], x[2], x[3]]
    q = UnitQuaternion(x[4], x[5], x[6], x[7])
    vᴮ = SA[x[8], x[9], x[10]]
    ωᴮ = SA[x[11], x[12], x[13]]

    m = QUAD_MASS
    s = QUAD_MOTOR_DIST
    kt = QUAD_KT
    km = QUAD_KM
    J = QUAD_J
    Jinv = QUAD_J_INV

    # Computing forces on Quadrotor
    Fᴮ = SA[0; 0; sum(kt * u)] + q' * (-m * g)

    # Computing moments on Quadrotor
    k̃t = s * kt / sqrt(2)
    M = SA[k̃t  -k̃t  -k̃t   k̃t;
          -k̃t  -k̃t   k̃t   k̃t;
           km  -km   km  -km];
    τᴮ = M * SA[u[1], u[2], u[3], u[4]]

    ṗ = q * vᴮ
    v̇ᴮ = 1/m * Fᴮ - ωᴮ × vᴮ

    q̇ = Rotations.kinematics(q, ωᴮ)     # Quaternion kinematics
    ω̇ᴮ = Jinv * (τᴮ - ωᴮ × (J * ωᴮ))        # Body velocity dynamics

    ẋ = [ṗ, q̇, v̇ᴮ, ω̇ᴮ]
    return ẋ
end

function discrete_quad_dynamics(x::SVector{NUM_STATE}, u::SVector{NUM_INPUT}, δt::Float64)
    k1 = dynamics(x, u)
    k2 = dynamics(x + 0.5 * δt * k1, u)
    k3 = dynamics(x + 0.5 * δt * k2, u)
    k4 = dynamics(x + δt * k3, u)
    xnext = x + (δt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    return xnext
end

function discreteJacobian(x::SVector{NUM_STATE}, u::SVector{NUM_INPUT}, δt::Real)
    A = ForwardDiff.jacobian(_x->discreteDynamics(_x, u, δt), x)
    B = ForwardDiff.jacobian(_u->discreteDynamics(x, _u, δt), u)
    return (A, B)
end

