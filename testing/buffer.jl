# module TimeBuffer
#     export Buffer, push!, nab!

    using Dates

    mutable struct Buffer{T}
        inputs::Vector{T}
        # times::Vector{DateTime}
        times::Vector{Float64}
        len::Int64

        function Buffer{T}(; size=100) where T
            inputs = Vector{T}(undef, size)
            # times = Vector{DateTime}(undef, size)
            times = Vector{Float64}(undef, size)
            len = 1

            return new{T}(inputs, times, len)
        end
    end

    function Base.length(buff::Buffer{T}) where T
        return buff.len - 1
    end

    # Add new element and associated time stamp to buffer
    function push!(buff::Buffer{T}, input::T, time::Float64=time()) where T
        if buff.len <= length(buff.inputs)
            buff.inputs[buff.len] = input
            buff.times[buff.len] = time
        else
            Base.push!(buff.inputs, input)
            Base.push!(buff.times, time)
        end

        buff.len = buff.len + 1
        return buff
    end

    # Get the element with the closest matching time stamp from the buffer
    function nab!(buff::Buffer{T}, time::Float64) where T
        @assert buff.len > 1 "Buffer must not be empty!"

        n_min = argmin(abs.(buff.times[1:buff.len-1] .- time))
        time_min = buff.times[n_min]
        input_min = buff.inputs[n_min]

        buff.len = 1  # Reset counter
        return input_min
    end


# end