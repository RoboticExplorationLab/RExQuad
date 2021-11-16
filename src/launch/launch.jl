import Mercury as Hg

RUNNING_NODES = Vector{Hg.Node}()

function launch_ground_link()
    node = GroundLink.main(; rate = 33.0, debug = false);
    node_task = Threads.@spawn Hg.launch(node)
    push!(RUNNING_NODES, node)

    return node
end

function launch_jetson_link()
    node = JetsonLink.main(; rate = 33.0, debug=false);
    node_task = Threads.@spawn Hg.launch(node)
    push!(RUNNING_NODES, node)

    return node
end

function launch_state_estimator(; debug = false)
    node = StateEstimator.main(; rate = 100.0, debug=debug);
    node_task = Threads.@spawn Hg.launch(node)
    push!(RUNNING_NODES, node)

    return node
end

function launch_lqr_controller(; debug = false, recompute_gains = false)
    node = LQRcontroller.main(; rate = 100.0, debug = debug, recompute_gains = recompute_gains);
    node_task = Threads.@spawn Hg.launch(node)
    push!(RUNNING_NODES, node)

    return node
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

Base.atexit(stopall)
