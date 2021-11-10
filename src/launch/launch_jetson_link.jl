using RExQuad

# %% For Testing
import Mercury as Hg

link_node = JetsonLink.main(; rate=33.0, debug=false);

# %%
link_node_task = Threads.@spawn Hg.launch(link_node)

# # %%
# Hg.stopnode(link_node; timeout = 1.0)
