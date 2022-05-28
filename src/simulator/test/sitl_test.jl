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
# render(sim.vis)

## Send Test Measurement
x = [0;0;1; 1; zeros(3); zeros(6)]
u = trim_controls() 
dt = 0.01
t = 0.0

x[2] = 0.5
x[3] = 1.0

reset!(sim)
push!(sim.stats["xhat"], SVector{13,Float32}(x))
step!(sim, x, u, t, imu_per_pose=1, pose_delay=1, send_measurement=true, send_ground_truth=false)
# step!(sim, x, u, t, imu_per_pose=2, pose_delay=1, send_measurement=true, send_ground_truth=false)
##

step!(sim, x, u, t, imu_per_pose=2, pose_delay=1, send_measurement=true, send_ground_truth=false)
##
reset!(sim)
push!(sim.stats["xhat"], SVector{13,Float32}(x))
tf = 3.0
runsim(sim, x, tf=tf, send_measurement=true, send_ground_truth=false)
RobotMeshes.visualize_trajectory!(sim.vis, sim, tf, sim.xhist)
sim.xhist