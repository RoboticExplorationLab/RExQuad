import Mercury as Hg

RUNNING_NODES = Vector{Hg.Node}()

function launch_ground_link()
    node = GroundLink.main(; rate = 33.0, debug = false);
    node_task = Threads.@spawn Hg.launch(node)
    push!(RUNNING_NODES, node)

    return
end

function launch_vicon_listener(; rate = 100.0, debug = false)
    node = ViconListener.ViconListenerNode(rate, debug, )
    Hg.setupIO!(node, Hg.getIO(node))
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

function launch_state_estimator(; rate = 100.0, debug = false)
    node = StateEstimator.main(; rate = rate, debug=debug);
    node_task = Threads.@spawn Hg.launch(node)
    push!(RUNNING_NODES, node)

    return
end

function launch_lqr_controller(; rate = 100.0, debug = false)
    node = LQRcontroller.LQRcontrollerNode(rate, debug)
    Hg.setupIO!(node, Hg.getIO(node))
    node_task = Threads.@spawn Hg.launch(node)

    push!(RUNNING_NODES, node)
    return
end

function launch_motor_spin_up(; debug = false)
    node = MotorSpinUp.MotorSpinNode(rate, debug)
    Hg.setupIO!(node, Hg.getIO(node))
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
