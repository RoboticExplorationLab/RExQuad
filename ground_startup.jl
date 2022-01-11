using Revise
import RExQuad
import Distributed


function shutdown_quadrotor()
    Distributed.@everywhere RExQuad.stopall()
end

begin
    @info "Setting up processes"
    @assert Distributed.nprocs() == 1

    Distributed.@everywhere using Revise
    Distributed.@everywhere using RExQuad

    ground_proc = 1

    @info "Launching Nodes"
    f1 = Distributed.@spawnat       ground_proc     RExQuad.launch_ground_link(; rate = 33., debug = false);

    t1 = Distributed.@fetchfrom     link_proc       RExQuad.jetson_master_running()
    println(t1)
end
