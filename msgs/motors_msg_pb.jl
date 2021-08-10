# syntax: proto3
using ProtoBuf
import ProtoBuf.meta

mutable struct MOTORS <: ProtoType
    __protobuf_jl_internal_meta::ProtoMeta
    __protobuf_jl_internal_values::Dict{Symbol,Any}
    __protobuf_jl_internal_defaultset::Set{Symbol}

    function MOTORS(; kwargs...)
        obj = new(meta(MOTORS), Dict{Symbol,Any}(), Set{Symbol}())
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
end # mutable struct MOTORS
const __meta_MOTORS = Ref{ProtoMeta}()
function meta(::Type{MOTORS})
    ProtoBuf.metalock() do
        if !isassigned(__meta_MOTORS)
            __meta_MOTORS[] = target = ProtoMeta(MOTORS)
            allflds = Pair{Symbol,Union{Type,String}}[:front_left => Float64, :front_right => Float64, :back_right => Float64, :back_left => Float64, :time => Float64]
            meta(target, MOTORS, allflds, ProtoBuf.DEF_REQ, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, ProtoBuf.DEF_PACK, ProtoBuf.DEF_WTYPES, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES)
        end
        __meta_MOTORS[]
    end
end
function Base.getproperty(obj::MOTORS, name::Symbol)
    if name === :front_left
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :front_right
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :back_right
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :back_left
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :time
        return (obj.__protobuf_jl_internal_values[name])::Float64
    else
        getfield(obj, name)
    end
end

export MOTORS
