"""
    delay_test.jl

Setup:
1. Load "feather_tx" onto Feather M0 w/ RF69 Radio at /dev/ttyACM0
2. Load "feather_rx" onto Feather M0 w/ RF69 Radio at /dev/ttyACM1
3. Run simulator/src/mocap_relay 2 5555 -v (delay of 2)
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


function send_message()
  y = getmeasurement(sim, x, u, t)
  sendmeasurement(sim, y, t)
  push!(xhist, copy(x))
  x[3] += 0.001
  nothing 
end

function tryrecv(socket, len, timeout_ms=10)
  tstart = time_ns()
  msg = ZMQ.Message(len)
  while (time_ns() - tstart < timeout_ms * 1e6)
    if (ZMQ.msg_recv(socket, msg, ZMQ.ZMQ_DONTWAIT) > 0) 
      return msg
    end
  end
  nothing
end

## Initialize Simulator
sim = Simulator(5555, 5556)
# open(sim.vis)

## Send a test measurement
# NOTE: launch the mocap_relay node with a delay of 2, e.g.
#     src/mocap_relay 2 5555 -v
x = [0; 0; 1; 1; zeros(3); zeros(6)]
xhist = Vector{Float32}[]
x[10] = 1.0
x[13] = -0.1
u = trim_controls()
dt = 0.01
t = 0.0

# Send the first measurement
send_message()
msg = tryrecv(sim.sub, 69, 100)
@test isnothing(msg)

# Send the second measurement
send_message()
msg = tryrecv(sim.sub, 69, 100)
@test isnothing(msg)

# Send the third measurement
send_message()
msg = tryrecv(sim.sub, 69, 100)
@test !isnothing(msg)
xu = StateControlMsg(msg, 0)
@test xu.z == xhist[1][3]  # should match the first message sent

send_message()
msg = tryrecv(sim.sub, 69, 100)
@test !isnothing(msg)
xu = StateControlMsg(msg, 0)
@test xu.z == xhist[2][3]  # should match the second message sent