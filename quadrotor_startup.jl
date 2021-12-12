using Revise
import RExQuad
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
    f1 = Distributed.@spawnat    link_proc           RExQuad.launch_jetson_link(; rate = 33., debug = false);
    f2 = Distributed.@spawnat    state_est_proc      RExQuad.launch_state_estimator(; rate = 100., debug = false);
    # f3 = Distributed.@spawnat    controller_proc     RExQuad.launch_motor_spin_up(; rate=100.0, debug = false);
    f3 = Distributed.@spawnat    controller_proc     RExQuad.launch_lqr_controller(; rate = 100.0, debug = false);

    # Add async check which shuts down all processes if any of them fail out
    # t1 = Distributed.fetch(f1)
    # t2 = Distributed.fetch(f2)
    # # t3 = Distributed.@fetch f3
    # println()
    # println(t1)
    # println(f2)
    # println()

    # Distributed.@fetchfrom link_proc istaskdone(t2)


    # println(!istaskdone(t1) && !istaskdone(t2))

    # while (!istaskdone(t1) && !istaskdone(t2))
    #     sleep(0.25)
    # end

    # println(istaskdone(t1), " ", istaskdone(t2))

    sleep(2)
    shutdown_quadrotor()
end
