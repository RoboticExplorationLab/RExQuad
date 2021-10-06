import Mercury as Hg

include("$(@__DIR__)/../nodes/imu_vicon_publisher/imu_vicon_publisher_node.jl")
# include("$(@__DIR__)/../nodes/filtered_state_publisher/filtered_state_publisher_node.jl")
include("$(@__DIR__)/../nodes/jetson_link/jetson_link_node.jl")

imu_vicon_node = ImuViconPublisher.main(; debug=false)
# filter_node = FilteredStatePublisher.main(; debug=true);
jetson_link_node = JetsonLink.main(; debug=false);

imu_vicon_node_task = Threads.@spawn Hg.launch(imu_vicon_node)
# filter_node_task = Threads.@spawn Hg.launch(filter_node)
jetson_link_node_task = Threads.@spawn Hg.launch(jetson_link_node)


# Hg.closeall(imu_vicon_node)
# Hg.closeall(filter_node)
# Hg.closeall(jetson_link_node)