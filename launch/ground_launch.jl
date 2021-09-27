# This file is run on the ground station
begin
    import Mercury as Hg

    include("$(@__DIR__)/../nodes/ground_link/ground_link_node.jl")

    ground_link_node = GroundLink.main(; rate=100.0, debug=true);
    ground_link_node_task = Threads.@spawn Hg.launch(ground_link_node)

    try
        while true
            sleep(0.001)
        end
    catch e
        Base.throwto(ground_link_node_task, InterruptException())
        Hg.closeall(ground_link_node)
    end
end

# %%
Base.throwto(ground_link_node_task, InterruptException())

# %%
Hg.closeall(ground_link_node)

