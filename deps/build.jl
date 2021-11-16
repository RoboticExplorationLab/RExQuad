import ProtoBuf

# Generate ProtoBuf julia files
outdir = joinpath(@__DIR__, "msgs")
if !isdir(outdir)
    Base.Filesystem.mkdir(outdir)
end

protodir = joinpath(@__DIR__, "proto")
if isdir(protodir)
    searchdir(path, key) = filter(x->occursin(key,x), readdir(path))

    msgfiles = searchdir(protodir, ".proto")

    for msgfile in msgfiles
        ProtoBuf.protoc(`-I=$protodir --julia_out=$outdir $msgfile`)
        @info "Built $(msgfile)"
    end
end
