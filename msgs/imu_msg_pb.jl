# syntax: proto3
using ProtoBuf
import ProtoBuf.meta

mutable struct IMU <: ProtoType
    __protobuf_jl_internal_meta::ProtoMeta
    __protobuf_jl_internal_values::Dict{Symbol,Any}
    __protobuf_jl_internal_defaultset::Set{Symbol}

    function IMU(; kwargs...)
        obj = new(meta(IMU), Dict{Symbol,Any}(), Set{Symbol}())
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
end # mutable struct IMU
const __meta_IMU = Ref{ProtoMeta}()
function meta(::Type{IMU})
    ProtoBuf.metalock() do
        if !isassigned(__meta_IMU)
            __meta_IMU[] = target = ProtoMeta(IMU)
            fnum = Int[1,2,3,5,6,7,8]
            allflds = Pair{Symbol,Union{Type,String}}[:acc_x => Float64, :acc_y => Float64, :acc_z => Float64, :gyr_x => Float64, :gyr_y => Float64, :gyr_z => Float64, :time => Float64]
            meta(target, IMU, allflds, ProtoBuf.DEF_REQ, fnum, ProtoBuf.DEF_VAL, ProtoBuf.DEF_PACK, ProtoBuf.DEF_WTYPES, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES)
        end
        __meta_IMU[]
    end
end
function Base.getproperty(obj::IMU, name::Symbol)
    if name === :acc_x
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :acc_y
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :acc_z
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :gyr_x
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :gyr_y
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :gyr_z
        return (obj.__protobuf_jl_internal_values[name])::Float64
    elseif name === :time
        return (obj.__protobuf_jl_internal_values[name])::Float64
    else
        getfield(obj, name)
    end
end

export IMU
