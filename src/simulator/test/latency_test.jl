"""
    latency_test.jl

Setup:
1. Load "feather_tx" onto Feather M0 w/ RF69 Radio at /dev/ttyACM0
2. Load "feather_rx" onto Feather M0 w/ RF69 Radio at /dev/ttyACM1
3. Run simulator/src/mocap_relay
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
sim = Simulator(5555, 5556)
# open(sim.vis)

## Send a test measurement
x = [0;0;1; 1; zeros(3); zeros(6)]
u = trim_controls() 
dt = 0.01
t = 0.0

##
x[10] = 1.0
x[13] = -0.1
cont_dynamics(x, u)
y = getmeasurement(sim, x, u, t)
sendmeasurement(sim, y, t)
x[3] += 0.001

## Make sure it's being received
recv_task = @async ZMQ.recv(sim.sub)
sendmeasurement(sim, y, t)
istaskdone(recv_task)
msg = fetch(recv_task)
xu = StateControlMsg(msg, 0)
@test xu.x == y.x
@test xu.y == y.y
@test xu.z == y.z
@test xu.qw == y.qw
@test xu.qx == y.qx
@test xu.qy == y.qy
@test xu.qz == y.qz

## Run sim and get latency
function measure_latency(sim, y, t)
    msg = ZMQ.Message(msgsize(StateControlMsg))
    tstart = time_ns()
    tend = tstart
    sendmeasurement(sim, y, t)
    timeout_ms = 5 
    while((time_ns() - tstart) < (timeout_ms * 1e6))
        bytes_received = ZMQ.msg_recv(sim.sub, msg, ZMQ.ZMQ_DONTWAIT)
        if bytes_received > 0
            tend = time_ns()
            break
        end
    end
    (tend - tstart) * 1e-9
end
function latency_test(sim; Nsamples=100)
    latency = Float64[]
    x = [0;0;1; 1; zeros(3); zeros(6)]
    dt = 0.01

    for i = 1:Nsamples
        println("Sending sample $i / $Nsamples")
        t = (i-1)*dt
        y = getmeasurement(sim, x, u, t)
        x[3] += 0.001
        sample = measure_latency(sim, y, t)
        if sample > 0
            push!(latency, sample)
        end
        sleep(0.02)
    end
    drop_rate = 1 - (length(latency) / Nsamples)
    @printf("Latency: %.3f ± %.3f ms\n", mean(latency) * 1000, std(latency) * 1000)
    @printf("Drop rate: %.2f%%\n", drop_rate * 100)
    latency
end
measure_latency(sim, y, 0.0)
latency_test(sim, Nsamples=50)

finish(sim)