# syntax: proto3
using ProtoBuf
import ProtoBuf.meta

mutable struct QUAD_INFO <: ProtoType
    __protobuf_jl_internal_meta::ProtoMeta
    __protobuf_jl_internal_values::Dict{Symbol,Any}
    __protobuf_jl_internal_defaultset::Set{Symbol}

    function QUAD_INFO(; kwargs...)
        obj = new(meta(QUAD_INFO), Dict{Symbol,Any}(), Set{Symbol}())
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
end # mutable struct QUAD_INFO
const __meta_QUAD_INFO = Ref{ProtoMeta}()
function meta(::Type{QUAD_INFO})
    ProtoBuf.metalock() do
        if !isassigned(__meta_QUAD_INFO)
            __meta_QUAD_INFO[] = target = ProtoMeta(QUAD_INFO)
            allflds = Pair{Symbol,Union{Type,String}}[:state => FILTERED_STATE, :input => MOTORS, :measurement => VICON, :time => Float64]
            meta(target, QUAD_INFO, allflds, ProtoBuf.DEF_REQ, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, ProtoBuf.DEF_PACK, ProtoBuf.DEF_WTYPES, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES)
        end
        __meta_QUAD_INFO[]
    end
end
function Base.getproperty(obj::QUAD_INFO, name::Symbol)
    if name === :state
        return (obj.__protobuf_jl_internal_values[name])::FILTERED_STATE
    elseif name === :input
        return (obj.__protobuf_jl_internal_values[name])::MOTORS
    elseif name === :measurement
        return (obj.__protobuf_jl_internal_values[name])::VICON
    elseif name === :time
        return (obj.__protobuf_jl_internal_values[name])::Float64
    else
        getfield(obj, name)
    end
end

export QUAD_INFO
