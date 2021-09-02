# syntax: proto3
using ProtoBuf
import ProtoBuf.meta

mutable struct FILTERED_STATE <: ProtoType
    __protobuf_jl_internal_meta::ProtoMeta
    __protobuf_jl_internal_values::Dict{Symbol,Any}
    __protobuf_jl_internal_defaultset::Set{Symbol}

    function FILTERED_STATE(; kwargs...)
        obj = new(meta(FILTERED_STATE), Dict{Symbol,Any}(), Set{Symbol}())
        values = obj.__protobuf_jl_internal_values
        symdict = obj.__protobuf_jl_internal_meta.symdict
        for nv in kwargs
            fldname, fldval = nv
            fldtype = symdict[fldname].jtyp
            (fldname in keys(symdict)) || error(string(typeof(obj), " has no field with name ", fldname))
            if fldval !== nothing
                values[fldname] = isa(fldval, fldtype) ? fldval : convert(fldtype, fldval)
            end
        end
        obj
    end
end # mutable struct FILTERED_STATE
const __meta_FILTERED_STATE = Ref{ProtoMeta}()
function meta(::Type{FILTERED_STATE})
    ProtoBuf.metalock() do
        if !isassigned(__meta_FILTERED_STATE)
            __meta_FILTERED_STATE[] = target = ProtoMeta(FILTERED_STATE)
            allflds = Pair{Symbol,Union{Type,String}}[:pos_x => Float64, :pos_y => Float64, :pos_z => Float64, :quat_w => Float64, :quat_x => Float64, :quat_y => Float64, :quat_z => Float64, :vel_x => Float64, :vel_y => Float64, :vel_z => Float64, :ang_x => Float64, :ang_y => Float64, :ang_z => Float64, :time => Float64]
            meta(target, FILTERED_STATE, allflds, ProtoBuf.DEF_REQ, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, ProtoBuf.DEF_PACK, ProtoBuf.DEF_WTYPES, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES)
        end
        __meta_FILTERED_STATE[]
    end
end
function Base.getproperty(obj::FILTERED_STATE, name::Symbol)
    if name === :pos_x
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :pos_y
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :pos_z
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :quat_w
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :quat_x
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :quat_y
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :quat_z
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :vel_x
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :vel_y
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :vel_z
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :ang_x
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :ang_y
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :ang_z
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :time
        return (obj.__protobuf_jl_internal_values[name])::Float64
    else
        getfield(obj, name)
    end
end

export FILTERED_STATE
