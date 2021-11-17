import Mercury as Hg

RUNNING_NODES = Vector{Hg.Node}()

function launch_ground_link()
    node = GroundLink.main(; rate = 33.0, debug = false);
    node_task = Threads.@spawn Hg.launch(node)
    push!(RUNNING_NODES, node)

    return
end

function launch_jetson_link()
    node = JetsonLink.main(; rate = 33.0, debug=false);
    node_task = Threads.@spawn Hg.launch(node)
    push!(RUNNING_NODES, node)

    return
end

function launch_state_estimator(; debug = false)
    node = StateEstimator.main(; rate = 100.0, debug=debug);
    node_task = Threads.@spawn Hg.launch(node)
    push!(RUNNING_NODES, node)

    return
end

function launch_lqr_controller(; debug = false, recompute_gains = false)
    node = LQRcontroller.main(; rate = 100.0, debug = debug, recompute_gains = recompute_gains);
    node_task = Threads.@spawn Hg.launch(node)
    push!(RUNNING_NODES, node)

    return
end

function launch_motor_spin_up(; debug = false, recompute_gains = false)
    node = MotorSpinUp.main(; rate = 1.0, debug = debug)
    node_task = Threads.@spawn Hg.launch(node)
    push!(RUNNING_NODES, node)

    return
end

function stopall()
    for node in RUNNING_NODES
        Hg.stopnode(node)
    end

    global RUNNING_NODES = Vector{Hg.Node}()
    return
end

precompile(launch_ground_link, ( ))
precompile(launch_jetson_link, ( ))
precompile(launch_state_estimator, ( ))
precompile(launch_lqr_controller, ( ))
precompile(launch_motor_spin_up, ( ))

Base.atexit(stopall)
