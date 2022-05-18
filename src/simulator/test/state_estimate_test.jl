"""
    state_estimate_test.jl

Visualizes the state estimate from the onboard computer in MeshCat.

Setup:
1. Load "feather_tx" onto Feather M0 w/ RF69 Radio at /dev/ttyACM0
2. Load "feather_rx" onto Feather M0 w/ RF69 Radio at /dev/ttyACM1
3. Run simulator/src/fake_mocap (or connect to real MOCAP)
4. Run simulator/src/onboard_relay
"""

import Pkg;
Pkg.activate(joinpath(@__DIR__, ".."));

using ZMQ
using Sockets
using MeshCat
using StaticArrays
using Statistics
using Printf
using Test
using Statistics
using Printf

include("../simulator.jl")

## Initialize Simulator
sim = Simulator(5562, 5563)
open(sim.vis)

## Run visualizer
# Set `send_measurement=false` to disable sending measurements since here we're only 
# trying to visualize the state estimate, not run the simulator
x = [0;0;1; 1; zeros(3); zeros(6)]
runsim(sim, x, tf=2.0, visualize=:estimate, send_measurement=false)

finish(sim)