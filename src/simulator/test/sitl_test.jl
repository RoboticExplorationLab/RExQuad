"""
    sitl_test.jl

This script runs a software-in-the-loop simulator. 

At each time step of the simulator, an IMU message is sent over ZMQ to the `fake_onboard`
C++ process, and optionally a delayed pose message. It then waits for a response from the 
C++ process, which provides the current state estimate and commanded control. These 
controls are then used to simulate the system forward using RK4.

Setup:
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

## Initialize Simulator
sim = Simulator(5562, 5563)
open(sim.vis)

## Run the simulator
x = [0;0.5;0.5; 1; zeros(3); zeros(6)]
u = trim_controls()
rate = 100  # Hertz
dt = 1/rate

tf = 3.0
runsim(sim, x, tf=tf, send_measurement=true, 
    imu_per_pose=1,  # number of IMU messages per pose message
    pose_delay=0,    # number of times the pose message should be delayed (total delay = dt * imu_per_pose * pose_delay)
    send_ground_truth=true # send the ground truth data to the onboard data to use (skipping the state estimator)
)
RobotMeshes.visualize_trajectory!(sim.vis, sim, tf, sim.xhist)
sim.xhist