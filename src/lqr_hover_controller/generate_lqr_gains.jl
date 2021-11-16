using RobotDynamics
using RobotZoo
using Rotations
using StaticArrays
using LinearAlgebra
using ControlSystems
using JSON

r0 = SA[0.0; 0.0; 1.0]
q0 = Rotations.params(UnitQuaternion(1.0, 0, 0, 0))
v0 = SA[0.0; 0.0; 0.0]
ω0 = SA[0.0; 0.0; 0.0]
const HOVER_STATE = SVector{13, Float64}([r0; q0; v0; ω0])

function compute_err_state(state::SVector{13, Float64})::SVector{12, Float64}
    r0 = SA[HOVER_STATE[1]; HOVER_STATE[2]; HOVER_STATE[3]]
    r1 = SA[state[1], state[2], state[3]]
    dr = r1 - r0

    q0 = UnitQuaternion(HOVER_STATE[4], HOVER_STATE[5], HOVER_STATE[6], HOVER_STATE[7])
    q1 = UnitQuaternion(SA[state[4], state[5], state[6], state[7]])
    dq = Rotations.rotation_error(q1, q0, Rotations.CayleyMap())

    v0 = SA[HOVER_STATE[8]; HOVER_STATE[9]; HOVER_STATE[10]]
    v1 = SA[state[8], state[9], state[10]]
    dv = v1 - v0

    ω0 = @SVector zeros(3)
    ω1 = SA[HOVER_STATE[11]; HOVER_STATE[12]; HOVER_STATE[13]]
    dω = ω1 - ω0

    dx = SA[dr; dq; dv; dω]
    return dx
end

"""
    generate_LQR_hover_gains([Qd, Rd; save_to_file])

Generates the LQR gains for the diagonal cost weights `Qd` and `Rd`. Assumes the
quadrotor is linearized about equilibrium for level flight.

The gains are returned and also saved to a file to be read in later.
"""
function generate_LQR_hover_gains(
    Qd = ones(12),
    Rd = fill(0.1, 4);
    save_to_file::Bool = true,
)

    model = RExQuad.gen_quadrotormodel()
    h = 0.02 # time step (s)

    #Initial Conditions
    Nx = RobotDynamics.state_dim(model)
    Nx̃ = RobotDynamics.state_diff_size(model)

    # Setting x₀ hover state
    xhover = HOVER_STATE
    uhover = RExQuad.trim_controls(model)

    # With RobotZoo
    linmodel = RobotDynamics.LinearizedModel(
        model,
        KnotPoint(xhover, uhover, h, 0.0),
        dt = h,
        integration = RobotDynamics.Exponential,
    )

    E2 = zeros(Nx, Nx̃)
    A = RobotDynamics.get_A(linmodel)
    B = RobotDynamics.get_B(linmodel)

    RobotDynamics.state_diff_jacobian!(E2, model, xhover)
    Ã = E2'A * E2
    B̃ = E2'B

    # Cost weights
    Q = Array(Diagonal(Qd))
    R = Array(Diagonal(Rd))

    # LQR Gain
    K = dlqr(Ã, B̃, Q, R)

    if (save_to_file)
        # Write gain to file
        file = open(RExQuad.LQR_gain_file, "w")
        JSON.print(file, K)
        close(file)
    end
    return K
end
