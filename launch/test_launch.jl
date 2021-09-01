# This file is run on the ground station
begin
    using Pkg
    Pkg.activate("$(@__DIR__)/..")

    node_dir = "$(@__DIR__)/../nodes"
    include("$(node_dir)/jetson_link/jetson_link.jl")

    jetson_link_thread = JetsonLinkDebug.main(; debug=true)


    try
        while true
            if istaskdone(jetson_link_thread)
                fetch(jetson_link_thread); break
            end
        end
    catch e
        if e isa InterruptException  # clean up
            println("Process terminated by you")
        end

        schedule(jetson_link_thread, InterruptException(), error=true)
    end
end