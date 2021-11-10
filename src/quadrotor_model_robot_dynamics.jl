using RobotDynamics
using Rotations
using StaticArrays
using LinearAlgebra

"""
    RExQuadBody{R}

A standard quadrotor model, with simple aerodynamic forces. The orientation is represent by
a general rotation `R`. The body z-axis point is vertical, so positive controls cause acceleration
in the positive z direction.

# Constructor
    Quadrotor(; kwargs...)
    Quadrotor{R}(; kwargs...)

where `R <: Rotation{3}` and defaults to `UnitQuaternion{Float64}` if omitted. The keyword arguments are
* `mass` - mass of the quadrotor, in kg (default = 0.5)
* `J` - inertia of the quadrotor, in kg⋅m² (default = `Diagonal([0.0023, 0.0023, 0.004])`)
* `gravity` - gravity vector, in kg/m² (default = [0,0,-9.81])
* `motor_dist` - distane between the motors, in m (default = 0.1750)
* `km` - motor torque constant (default = 0.0245)
* `bm` - motor torque bias (default = 0.0245)
* `kf` - motor force constant (default = 1.0)
* `bf` - motor force bias (default = 1.0)
"""
struct RExQuadBody <: RigidBody{UnitQuaternion}
    mass::Float64
    J::SMatrix{3,3,Float64,9}
    Jinv::SMatrix{3,3,Float64,9}
    gravity::SVector{3,Float64}
    motor_dist::Float64
    kf::Float64
    bf::Float64
    km::Float64
    bm::Float64
    bodyframe::Bool  # velocity in body frame?
    ned::Bool
end
RobotDynamics.control_dim(::RExQuadBody) = 4

function RExQuadBody(;
    mass = 1.776,
    J = (@SMatrix  [0.01566089 0.00000318037 0; 0.00000318037 0.01562078 0; 0 0 0.02226868]),
    gravity = SVector(0, 0, -9.81),
    motor_dist = 0.28,
    kf = 40.9666,
    bf = 1248.9,
    km = 3338.01,
    bm = 1227.37,
    bodyframe = true,
) where {R}
    @assert issymmetric(J)
    RExQuadBody(mass, J, inv(J), gravity, motor_dist, kf, bf, km, bm, bodyframe)
end

@inline RobotDynamics.velocity_frame(model::RExQuadBody) = model.bodyframe ? :body : :world

function trim_controls(model::RExQuadBody)
    @SVector fill(-model.gravity[3] * model.mass / 4.0 / model.kf, size(model)[2])
end

"""
* `x` - Quadrotor state
* `u` - Motor PWM commands
"""
function RobotDynamics.forces(model::RExQuadBody, x, u)
    q = orientation(model, x)
    kf, km = model.kf, model.km
    bf, bm = model.bf, model.bm
    L = model.motor_dist
    g = model.gravity
    m = model.mass

    w1 = u[1] #PWM on front left motor
    w2 = u[2] #PWM on front right motor
    w3 = u[3] #PWM on back right motor
    w4 = u[4] #PWM on back left motor

    F1 = clamp(kf * w1 + bf, MIN_THROTLE, MAX_THROTLE)
    F2 = clamp(kf * w2 + bf, MIN_THROTLE, MAX_THROTLE)
    F3 = clamp(kf * w3 + bf, MIN_THROTLE, MAX_THROTLE)
    F4 = clamp(kf * w4 + bf, MIN_THROTLE, MAX_THROTLE)

    force_body = SA[0; 0; (F1+F2+F3+F4)] + q' * (-m * g)

    return force_body
end

function RobotDynamics.moments(model::RExQuadBody, x, u)
    kf, km = model.kf, model.km
    bf, bm = model.bf, model.bm
    L = model.motor_dist

    w1 = u[1] #PWM on front left motor
    w2 = u[2] #PWM on front right motor
    w3 = u[3] #PWM on back right motor
    w4 = u[4] #PWM on back left motor

    F1 = clamp(kf * w1 + bf, MIN_THROTLE, MAX_THROTLE)
    F2 = clamp(kf * w2 + bf, MIN_THROTLE, MAX_THROTLE)
    F3 = clamp(kf * w3 + bf, MIN_THROTLE, MAX_THROTLE)
    F4 = clamp(kf * w4 + bf, MIN_THROTLE, MAX_THROTLE)

    M1 = km * w1 + bm
    M2 = km * w2 + bm
    M3 = km * w3 + bm
    M4 = km * w4 + bm

    r = L * sqrt(2) / 4  # moment arm (m)
    tau_body = SA[r*(F1-F2-F3+F4);
                  r*(-F1-F2+F3+F4);
                  (M1-M2+M3-M4)]
    return tau_body
end

RobotDynamics.inertia(model::RExQuadBody) = model.J
RobotDynamics.inertia_inv(model::RExQuadBody) = model.Jinv
RobotDynamics.mass(model::RExQuadBody) = model.mass

function Base.zeros(model::RExQuadBody)
    x = RobotDynamics.build_state(model, zero(RBState))
    u = @SVector fill(-model.mass * model.gravity[end] / 4, 4)
    return x, u
end

function gen_quadrotormodel()
    #Quadrotor parameters
    m = 1.776 # weight (kg)
    l = 0.28  # motor distance (m)
    J = [0.01566089 0.00000318037 0; 0.00000318037 0.01562078 0; 0 0 0.02226868]
    g = 9.81  # gravity (m/s^2)
    kt = 0.11   # motor force constant (N/(rad/s))
    km = 0.044  # motor torque constant (Nm/(rad/s))

    kf = 40.9666
    bf = 1248.9
    km = 3338.01
    bm = 1227.37

    model =
        RExQuadBody(mass = m, motor_dist = l, J = J, gravity = SA[0, 0, -g], km = km, bm = bm, kf = kf, bf = bf)
    return model
end

function statemsg2vector(x::RExQuad.FILTERED_STATE)
    SA[
        x.pos_x,
        x.pos_y,
        x.pos_z,
        x.quat_w,
        x.quat_x,
        x.quat_y,
        x.quat_z,
        x.vel_x,
        x.vel_y,
        x.vel_z,
        x.ang_x,
        x.ang_y,
        x.ang_z,
    ]
end
