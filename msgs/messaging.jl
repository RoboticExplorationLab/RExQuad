module messaging
  const _ProtoBuf_Top_ = @static isdefined(parentmodule(@__MODULE__), :_ProtoBuf_Top_) ? (parentmodule(@__MODULE__))._ProtoBuf_Top_ : parentmodule(@__MODULE__)
  include("filtered_state_msg_pb.jl")
  include("motors_msg_pb.jl")
  include("vicon_msg_pb.jl")
  include("quad_info_msg_pb.jl")
end
