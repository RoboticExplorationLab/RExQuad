using Pkg;
Pkg.activate(joinpath(@__DIR__, "..", ".."));
using LinearAlgebra
using ForwardDiff
using BlockDiagonals
using RobotDynamics
using RobotZoo
using StaticArrays
using ControlSystems
using JSON

include(joinpath(@__DIR__, "..", "constants.jl"))
include(joinpath(@__DIR__, "..", "quadrotor_model.jl"))

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

    model = gen_quadrotormodel()
    h = 0.02 # time step (s)

    #Initial Conditions
    Nx = state_dim(model)
    Nx̃ = RobotDynamics.state_diff_size(model)
    r0 = SA[0.0; 0; 1.0]
    q0 = SA[1.0; 0; 0; 0]
    v0 = @MVector zeros(3)
    ω0 = @MVector zeros(3)
    x0 = [r0; q0; v0; ω0]
    x̃0 = [r0; SA[0; 0; 0]; v0; ω0]
    uhover = trim_controls(model)

    # With RobotZoo
    linmodel = RobotDynamics.LinearizedModel(
        model,
        KnotPoint(x0, uhover, h, 0.0),
        dt = h,
        integration = RobotDynamics.Exponential,
    )
    E2 = zeros(Nx, Nx̃)
    A = RobotDynamics.get_A(linmodel)
    B = RobotDynamics.get_B(linmodel)
    RobotDynamics.state_diff_jacobian!(E2, model, x0)
    Ã = E2'A * E2
    B̃ = E2'B

    # Cost weights
    Q = Array(Diagonal(Qd))
    R = Array(Diagonal(Rd))

    # LQR Gain
    K = dlqr(Ã, B̃, Q, R)

    if (save_to_file)
        # Write gain to file
        file = open(LQR_gain_file, "w")
        JSON.print(file, K)
        close(file)
    end
    return K
end
generate_LQR_hover_gains()
