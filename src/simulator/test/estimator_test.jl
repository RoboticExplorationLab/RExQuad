import Pkg;
Pkg.activate(joinpath(@__DIR__, ".."));
include("../simulator.jl")

Nmpc = 21
ctrl = RiccatiSolver(12, 4, Nmpc, 13, state_error)
filter = DelayedMEKF()
sim = Simulator(ctrl, filter)
open(sim.vis)