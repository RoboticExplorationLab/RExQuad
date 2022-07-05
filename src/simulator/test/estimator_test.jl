import Pkg;
Pkg.activate(joinpath(@__DIR__, ".."));
include("../simulator.jl")

Nmpc = 21
ctrl = RiccatiSolver(12, 4, Nmpc, 13, state_error)
filter = DelayedMEKF()
sim = Simulator(ctrl, filter)
# open(sim.vis)

## State Prediction
xd = [
    1.0, 0.2, 1.3,               # position
    0.999, 0.0, 0.0, 0.04362,    # attitude
    0.1, -0.2, 0.3,              # velocity
    0.2, 0.2, 0.2,               # accel bias
    0.3, 0.3, 0.3                # gyro bias
]
y_imu = [
    0.25, 0.24, 0.23,
    0.33, 0.32, 0.31
]
Pd = Matrix(1.0I,15,15)
h = 0.01
xp,Pp = state_prediction(filter, xd, y_imu, Pd, h)

y_mocap = [1.1, 1.2, 2.3, 0.9997, 0.02618, 0, 0,]
xd, Pd = measurement_update(filter, xp, Pp, y_mocap)