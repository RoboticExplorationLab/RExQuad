import Pkg; Pkg.activate(joinpath(@__DIR__, ".."));

using ZMQ
using Sockets
using MeshCat
using StaticArrays
using Statistics
using Printf
using Test
using Statistics
using Printf
using CoordinateTransformations
using RobotZoo

include("../simulator.jl")

## Initialize Simulator
sim = Simulator(5562, 5563)
open(sim.vis)

x0 = [0;1;0.5; 1; zeros(3); zeros(6)]
u = trim_controls()
dt = 0.01
t = 0.0
tf = 3.0
reset!(sim)
push!(sim.stats["xhat"], x0)
step!(sim, x0, u, t, dt)

reset!(sim)
sim.ctrl.opts[:send_ground_truth] = true
runsim(sim, x0, dt=dt, tf=tf)

RobotMeshes.visualize_trajectory!(sim.vis, sim, tf, sim.xhist)
sim.uhist

finish(sim)

## Initialize controller
xeq = [0;0;1; 1; zeros(3); zeros(6)]
ueq = trim_controls()
Nmpc = 201
dt = 0.01
# xg = [0;0;1.5; 1; zeros(3); zeros(6)]
xg = copy(xeq)
x0 = [0;0;0.5; 1; zeros(3); zeros(6)]
ctrl = OSQPController(xeq, ueq, dt, Nmpc; xg)

## 
sim = Simulator(ctrl)
open(sim.vis)
reset!(sim)
tf = 5.0
x0 = [0;1;0.5; 1; zeros(3); zeros(6)]
update_initial_state!(ctrl, x0)
xg[3] = 1.5
update_goal_state!(ctrl, xg)
Qd = [1.1;1.1;10; fill(1.0, 3); fill(0.1,3); fill(1.0,3)]
Rd = fill(1e-3, 4)
# update_problem!(ctrl; Qd, Rd, Qf=copy(Qd))
runsim(sim, x0, dt=dt, tf=tf)
RobotMeshes.visualize_trajectory!(sim.vis, sim, tf, sim.xhist)
ctrl.q
sim.xhist

finish(sim)