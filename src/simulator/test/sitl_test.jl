"""
    sitl_test.jl

This script runs a software-in-the-loop simulator. 

At each time step of the simulator, an IMU message is sent over ZMQ to the `fake_onboard`
C++ process, and optionally a delayed pose message. It then waits for a response from the 
C++ process, which provides the current state estimate and commanded control. These 
controls are then used to simulate the system forward using RK4.

Setup:
    - Clone https://github.com/RoboticExplorationLab/robot_meshes onto your computer and 
      update `robot_meshes` variable in `visualization.jl` to point to it.
    - Run `build/src/simulator/src/fake_onboard 5562 5563` in the terminal (after compiling)
    - Run this Julia script to run the simulator

Options:
From this script you can specify a few options for the simulator:
    - `imu_per_pose`: the number of IMU messages sent per pose message sent
    - `pose_delay`: number of times the pose message should be delayed 
                    Total delay = `dt * imu_per_pose * pose_delay`.
    - `send_ground_truth`: send the ground truth data to the onboard data to use (skipping the state estimator)
"""

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
include("../sitl.jl")


## Initialize SITL interface
sitl = SITL(5555, 5556)
sub = sitl.sub
pub = sitl.pub
y_imu = [
    0.25, 0.24, 0.23,
    0.33, 0.32, 0.31
]
y_mocap = [0.1; 0.2; 1.1; cay([0,0,0.1])]

xhat = [zeros(3); 1; zeros(3); zeros(6)]
getcontrol(sitl, xhat, [], 0.0)
get_state_estimate!(sitl, y_imu, y_mocap, 0.01)
finish(sitl)

## Initialize Simulator
sitl = SITL(5555, 5556)
filter = DelayedMEKF()
sim = Simulator(sitl, filter)
open(sim.vis)

## Run the simulator
x = [0; 2; 0.5; 1; zeros(3); zeros(6)]
u = trim_controls()
rate = 100  # Hertz
dt = 1/rate

tf = 4.0
d = 5
sim.opts.delay_comp = d
sim.opts.mocap_delay = d
sim.opts.use_ground_truth = false 

runsim(sim, x, tf=tf)
RobotMeshes.visualize_trajectory!(sim.vis, sim, tf, sim.xhist)

## Save a part of the trajectory to a file for testing
function write_vectors_to_c(header, impl, vecs, name)
    N = length(vecs)
    n = length(vecs[1])
    println(header, "double $name[$N][$n];")
    println(impl, "double $name[$N][$n] = {")
    for i = 1:N
        print(impl, "  {")
        for j = 1:n
            print(impl, vecs[i][j], ",")
        end
        println(impl, "},");
    end
    println(impl, "};")
end

N = 10
header = open(joinpath(@__DIR__, "../../common/test/filter_data.h"), "w")
impl  = open(joinpath(@__DIR__, "../../common/test/filter_data.c"), "w")
println(header, "const int MOCAP_DELAY = $d;")
println(impl, "#include \"filter_data.h\"")
write_vectors_to_c(header, impl, sim.imuhist[1:N], "imuhist")
write_vectors_to_c(header, impl, last.(sim.mocaphist[1:N-d]), "mocaphist")
write_vectors_to_c(header, impl, sim.x̂hist[1:N+1], "xhist")

close(header)
close(impl)

## Check that the results are repeatable
filter = DelayedMEKF()
x0 = sim.x̂hist[1]
initialize!(filter, x0, delay_comp=d)
h = 0.01
err = map(1:N) do i
    y_imu = sim.imuhist[i]
    y_mocap = i <= d ? nothing : last(sim.mocaphist[i-d])
    xhat = get_state_estimate!(filter, y_imu, y_mocap, h)
    norm(xhat - sim.x̂hist[i+1])
end
norm(err) ≈ 0