
##
sim = Simulator(5562, 5563)
open(sim.vis)

##
x = [0; 0.5; 0.5; 1; zeros(3); zeros(6)]
u = trim_controls()
dt = 0.01
tf = 10.0
reset!(sim)
length(sim.msg_meas)
runsim(sim, x; dt, tf)
RobotMeshes.visualize_trajectory!(sim.vis, sim, tf, sim.xhist)

##
tout = getfield.(sim.msg_meas, :twall) 
zout = map(msg->msg.y.z, sim.msg_meas)
tin = getfield.(sim.msg_data, :twall) 
zin = map(msg->msg.xu.z, sim.msg_data)

j = 1
idx = zeros(Int,length(tin)) 
for i = 1:length(tin)
    z = zin[i]
    for k = j:length(tout)
        if zout[k] ≈ z
            idx[i] = k
            j = k  # start search at current time step
        end
    end
end
idx_in = findall(iszero |> !, idx)
idx_out = idx[idx_in]
matches = map(1:length(idx_out)) do k
    i = idx_in[k]
    j = idx_out[k]
    (
        tout=tout[j], tin=tin[i], 
        zout=zout[j], zin=zin[i]
    )
end
matches
all(x->x.zout == x.zin, matches)
all(x->x.tin > x.tout, matches)
latency = map(x->(x.tin - x.tout) * 1000, matches)
mean(latency)
std(latency)

##
x = [0;0;1; 1; zeros(3); zeros(6)]
u = trim_controls() 
t = 0.0

##
# x[11] += 10.0
x[10] = 1.0
x[13] = -0.1
cont_dynamics(x, u)
y = getmeasurement(sim, x, u, t)
sendmeasurement(sim, y, t)
x[3] += 0.1
##
step!(sim, x, u, t)
t += dt
##
isopen(sim.pub)
isopen(sim.sub)

recv_task = @async ZMQ.recv(sim.sub)
istaskdone(recv_task)
msg = fetch(recv_task)
xu = StateControlMsg(msg, 0)
xu.x == y.x
xu.y == y.y
xu.z == y.z
xu.qw == y.qw
xu.qx == y.qx
xu.qy == y.qy
xu.qz == y.qz
getcontrol(posemsg)
Int(msg[1]) == msgid(StateControlMsg) 

finish(sim)