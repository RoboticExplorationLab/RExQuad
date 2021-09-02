# syntax: proto3
using ProtoBuf
import ProtoBuf.meta

mutable struct GROUND_INFO <: ProtoType
    __protobuf_jl_internal_meta::ProtoMeta
    __protobuf_jl_internal_values::Dict{Symbol,Any}
    __protobuf_jl_internal_defaultset::Set{Symbol}

    function GROUND_INFO(; kwargs...)
        obj = new(meta(GROUND_INFO), Dict{Symbol,Any}(), Set{Symbol}())
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
end # mutable struct GROUND_INFO
const __meta_GROUND_INFO = Ref{ProtoMeta}()
function meta(::Type{GROUND_INFO})
    ProtoBuf.metalock() do
        if !isassigned(__meta_GROUND_INFO)
            __meta_GROUND_INFO[] = target = ProtoMeta(GROUND_INFO)
            allflds = Pair{Symbol,Union{Type,String}}[:deadman => Bool, :time => Float64]
            meta(target, GROUND_INFO, allflds, ProtoBuf.DEF_REQ, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, ProtoBuf.DEF_PACK, ProtoBuf.DEF_WTYPES, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES)
        end
        __meta_GROUND_INFO[]
    end
end
function Base.getproperty(obj::GROUND_INFO, name::Symbol)
    if name === :deadman
        return (obj.__protobuf_jl_internal_values[name])::Bool
    elseif name === :time
        return (obj.__protobuf_jl_internal_values[name])::Float64
    else
        getfield(obj, name)
    end
end

export GROUND_INFO
