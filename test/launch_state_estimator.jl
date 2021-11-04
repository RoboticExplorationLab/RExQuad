using Revise
using RExQuad

# %% For Testing
import Mercury as Hg

filter_node = StateEstimator.main(; rate=1.0, debug=true);

# %%
filter_node_task = Threads.@spawn Hg.launch(filter_node)

# # %%
# Hg.stopnode(filter_node; timeout = 1.0)
