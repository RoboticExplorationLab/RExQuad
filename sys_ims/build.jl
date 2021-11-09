using PackageCompiler
using RExQuad

# PackageCompiler.create_sysimage(
#     ["RExQuad"];
#     sysimage_path="RExQuad_Filter.so",
#     precompile_execution_file=joinpath(@__DIR__, "..", "test", "launch_state_estimator.jl")
# )

PackageCompiler.create_sysimage(
    ["RExQuad"];
    sysimage_path="RExQuad_Ground.so",
    precompile_execution_file=joinpath(@__DIR__, "..", "test", "launch_ground_link.jl")
)
