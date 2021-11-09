using RExQuad

# %% For Testing
import Mercury as Hg

node = GroundLink.main(; rate = 100.0, debug = false);

# %%
node_task = Threads.@spawn Hg.launch(node)

# # %%
# Hg.stopnode(node)