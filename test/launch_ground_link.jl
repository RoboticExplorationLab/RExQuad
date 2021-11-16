
function launch_ground_link()
    node = GroundLink.main(; rate = 100.0, debug = false);
    node_task = Threads.@spawn Hg.launch(node)
    return node
end
# # %%
# Hg.stopnode(node)