using DelimitedFiles
using Statistics
using Printf

if isempty(ARGS)
    basename = "serial_chain_latency"
else
    basename = ARGS[1]
end
outdata = readdlm(joinpath(@__DIR__, "$(basename)_test_out.txt"), ',')
indata = readdlm(joinpath(@__DIR__, "$(basename)_test_in.txt"), ',')

idx = map(eachrow(indata)) do (t,x) 
    idx = searchsortedfirst(outdata[:,2], x)
end
outdata[idx,2] ≈ indata[:,2]

tout = outdata[idx,1]
tin = indata[:,1]
all(tout .< tin)
latency = tin - tout
t_elapsed = ((tin[end] - tin[1]) + (tout[end] - tout[1])) / 2
rate = length(tout) / t_elapsed * 1000

@printf("Average Rate = %0.2f Hz\n", rate)
@printf("Latency = %0.3g ± %0.4f ms\n", mean(latency), std(latency))