using DelimitedFiles

outdata = readdlm(joinpath(@__DIR__, "lora_latency_test_out.txt"), ',')
indata = readdlm(joinpath(@__DIR__, "lora_latency_test_in.txt"), ',')

idx = map(eachrow(indata)) do (t,x) 
    idx = searchsortedfirst(outdata[:,2], x)
end
outdata[idx,2] ≈ indata[:,2]
outdata
indata

tout = outdata[idx,1]
tin = indata[:,1]
all(tout .< tin)
latency = tin - tout
latency