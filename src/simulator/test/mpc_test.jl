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

##
reset!(sim)
tf = 5.0
x0 = [0;2.0;0.5; 1; zeros(3); zeros(6)]
xg[3] = 1.0
update_initial_state!(ctrl, x0)
update_goal_state!(ctrl, xeq)

Qd = [0.5;0.5;10; fill(10.0, 3); fill(0.1,3); fill(1.0,3)]
Rd = fill(1e-3, 4)
update_problem!(ctrl; Qd, Rd, Qf=copy(Qd)*1000, N=21)
runsim(sim, x0, dt=dt, tf=tf)
RobotMeshes.visualize_trajectory!(sim.vis, sim, tf, sim.xhist)
ctrl.q
sim.xhist

finish(sim)

function linear_sim(ctrl, x0, tf=2.0)
  n,m,N = 12,4,ctrl.N
  dt = ctrl.dt
  times = range(0,tf,step=dt)
  dx0 = state_error(x0, ctrl.xeq)
  dX = [copy(dx0) for t in times]
  X = [copy(x0) for t in times]
  Ad,Bd = ctrl.Ad, ctrl.Bd
  iu = n*N .+ (1:m)
  for k = 1:length(times)-1
    # u = getcontrol(ctrl, x, nothing, times[k]) - ctrl.ueq
    ctrl.l[1:n] .= .-dX[k]
    ctrl.u[1:n] .= .-dX[k]
    OSQP.update!(ctrl.prob, l=ctrl.l, u=ctrl.u)
    res = OSQP.solve!(ctrl.prob)
    du = res.x[iu]
    
    dX[k+1] = Ad*dX[k] + Bd*du
    X[k+1] = add_state(ctrl.xeq, dX[k+1])
  end
  dX,X
end

##
using Plots
x0 = [0;0.0;0.5; 1; zeros(3); zeros(6)]
tf = 5.0
Qd = [1.;1.;10; fill(10.1, 3); fill(0.1,3); fill(0.1,3)]
update_problem!(ctrl; Qd, Rd, Qf=copy(Qd)*1000, N=51)
dX,X = linear_sim(ctrl, x0, tf)
RobotMeshes.visualize_trajectory!(sim.vis, sim, tf, X)
z = getindex.(dX,3)
plot(z)

##
n,m,N = 12,4,ctrl.N
iu = n*N .+ (1:m)
dx = state_error(x0, ctrl.xeq)
##
Qd = [1.;1.;10; fill(10.1, 3); fill(0.1,3); fill(0.1,3)]
update_problem!(ctrl; Qd, Rd, Qf=copy(Qd)*1000, N=51)
times = range(0,length=ctrl.N,step=ctrl.dt)
ctrl.l[1:n] .= .-dx
ctrl.u[1:n] .= .-dx
OSQP.update!(ctrl.prob, l=ctrl.l, u=ctrl.u)
res = OSQP.solve!(ctrl.prob)
@show res.info.status
du = res.x[iu]
dx .= ctrl.Ad*dx + ctrl.Bd*du
Xs = reshape(res.x[1:N*n],n,N)
plot(times,Xs[3,:], ylim=(-0.1,0.05))