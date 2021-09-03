begin
    using Pkg; Pkg.activate(joinpath(@__DIR__, "..",".."))
    using LinearAlgebra
    using ForwardDiff
    using BlockDiagonals
    using RobotDynamics
    using RobotZoo
    using StaticArrays
    using ControlSystems
    using JSON

    #Quadrotor parameters
    m = 0.5     # mass
    ℓ = 0.1750  # distance between motors
    J = [0.01566089 0.00000318037 0; 0.00000318037 0.01562078 0; 0 0 0.02226868]
    g = 9.81    # gravity
    kt=1.0      # motor force cosntant
    km=0.0245   # motor torque constant
    h = 0.02    # time step (50 Hz)

    model = RobotZoo.Quadrotor(
        mass = m,
        motor_dist = ℓ,
        J = J,
        gravity = SA[0,0,-g],
        km = km,
        kf = kt
    )
    fieldnames(RobotZoo.Quadrotor)

    #Initial Conditions
    uhover = (m*g/4)* @SVector ones(4)
    r0 = SA[0.0; 0; 1.0]
    q0 = SA[1.0; 0; 0; 0]
    v0 = @MVector zeros(3)
    ω0 = @MVector zeros(3)
    x0 = [r0; q0; v0; ω0]
    x̃0 = [r0; SA[0; 0; 0]; v0; ω0];

    # With RobotZoo
    linmodel = RobotDynamics.LinearizedModel(model, KnotPoint(x0, uhover, h, 0.0), dt=h, integration=RobotDynamics.Exponential)
    E2 = zeros(Nx,Nx̃)
    A = RobotDynamics.get_A(linmodel)
    B = RobotDynamics.get_B(linmodel)
    RobotDynamics.state_diff_jacobian!(E2, model, x0)
    Ã = E2'A*E2
    B̃ = E2'B

    # Cost weights
    Q = Array(I(Nx̃));
    R = Array(.1*I(Nu));

    # LQR Gain
    S = dare(Ã,B̃,Q,R) #cost-to-go
    K = dlqr(Ã,B̃,Q,R);

    # Write gain to file
    file = open(LQR_gain_file, "w")
    JSON.print(file, K)
    close(file)
end