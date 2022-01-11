import Distributed

function shutdown_quadrotor()
    Distributed.@everywhere RExQuad.stopall()
end

begin
    @info "Setting up processes"

    if Distributed.nprocs() == 1
        Distributed.addprocs(3; exeflags="--project")
    end
    Distributed.@everywhere using Revise
    Distributed.@everywhere using RExQuad

    link_proc, state_est_proc, controller_proc = 2, 3, 4

    @info "Launching Nodes"
    f1 = Distributed.@spawnat    link_proc           RExQuad.launch_jetson_master(; rate = 100., debug = false);
    # f2 = Distributed.@spawnat    state_est_proc      RExQuad.launch_state_estimator(; rate = 100., debug = false);
    # f3 = Distributed.@spawnat    controller_proc     RExQuad.launch_motor_spin_up(; rate=100.0, debug = false);
    # f3 = Distributed.@spawnat    controller_proc     RExQuad.launch_lqr_controller(; rate = 100.0, debug = false);

    t1 = Distributed.@fetchfrom     link_proc           RExQuad.jetson_master_running()
    println(t1)
    # t2 = Distributed.@fetchfrom     state_est_proc      RExQuad.state_estimator_running()
    # t3 = Distributed.@fetchfrom     controller_proc     RExQuad.lqr_controller_running()
    # t3 = Distributed.@fetchfrom     controller_proc     RExQuad.motor_spin_up_running()
end
