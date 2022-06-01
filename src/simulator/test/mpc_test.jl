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
Nmpc = 21
xg = [0;0;1.5; 1; zeros(3); zeros(6)]
x0 = [0;0;0.5; 1; zeros(3); zeros(6)]
ctrl = OSQPController(xeq, ueq, dt, Nmpc; xg)

update_goal_state!(ctrl, x0)
update_initial_state!(ctrl, x0)
getcontrol(ctrl, x0) - ueq
update_goal_state!(ctrl, [zeros(3); 1; zeros(3+6)])
getcontrol(ctrl, x0) - ueq

## 