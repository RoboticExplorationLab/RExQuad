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

## Initialize Simulator
filter = DelayedMEKF()
sim = Simulator(sitl, filter)
open(sim.vis)

## Run the simulator
x = [0; 2; 0.5; 1; zeros(3); zeros(6)]
u = trim_controls()
rate = 100  # Hertz
dt = 1/rate

tf = 4.0
sim.opts.use_ground_truth = false 
runsim(sim, x, tf=tf)
RobotMeshes.visualize_trajectory!(sim.vis, sim, tf, sim.xhist)