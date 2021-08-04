# How to make you're code allocation free

## Use [BenchmarkTools.jl](https://github.com/JuliaCI/BenchmarkTools.jl)

The BenchmarkTools package provides a couple very useful macros which help with tracking down memory allocations
```julia
julia> function myFunction(x)
           # Bad way of multiplying x by 5
           y = fill(x, 5)
           return sum(y)
       end
julia> x = 3.0
julia> @btime myFunction($x)
  36.478 ns (1 allocation: 128 bytes)
15.0
```

When using the `@btime` and `@benchmark` macros be sure to add the `$` before any arguments that are passed to the function.

## Use [StaticArrays.jl](https://github.com/JuliaArrays/StaticArrays.jl)

Using the StaticArrays package is key to making your code allocation free. In particular use the `SArray` type and all aliases of it (`SVector` and `SMatrix`). Try to avoid using the `MArray` type, these are mutable arrays which must be allocated on the heap, and thus loose much of the efficency of statically sized arrays.

### Concatenating Matricies

Often it's useful to concatenate matricies both horizontally and vertically. To do this with `SArray`s with zero allocation you have to independently concatenate rows then columns of block matricies, as shown below.

```julia
julia> A = @SMatrix rand(2, 2);
julia> B = @SMatrix rand(2, 3);
julia> C = @SMatrix rand(3, 2);
julia> D = @SMatrix rand(3, 3);
julia> @btime [[$A  $B];  [$C  $D]]
  4.054 ns (0 allocations: 0 bytes)
5×5 SMatrix{5, 5, Float64, 25} with indices SOneTo(5)×SOneTo(5):
 0.412584  0.872161  0.698067  0.38682   0.450188
 0.389935  0.108013  0.465265  0.50287   0.198771
 0.182218  0.421294  0.557995  0.385125  0.0377156
 0.128915  0.547159  0.872414  0.442545  0.326073
 0.348874  0.17044   0.177955  0.253314  0.248989
```

### Converting MArray to SArray types

Often times you'll want to have a mutable array inside of a immutable struct so that you can update the parameters of the struct with no additional memory allocation. The best way to do this is to use the `MArray` and `SArray` types provided by the StaticArrays.jl package.

To do this the `MArray` attribute of the struct must be a concrete type. Once this is true you can convert `MArray`s into `SArray`s allocation free, while still being able to change the values inside the struct. The example below demonstrates this.

```julia
julia> struct PointWithCovariance{T}
           point::MVector{3, T}
           cov::MMatrix{3, 3, T, 9}

           function PointWithCovariance()
               point = @MVector rand(3)
               cov = @MMatrix rand(3, 3)
               return new{Float64}(point, cov)
           end
       end
julia> p = PointWithCovariance()
PointWithCovariance{Float64}([0.8730943182240327, 0.09370567777797634, 0.9295551095221855], [0.304226090393819 0.7333536333635879 0.7193749657963469; 0.9995343299818555 0.19259264827392486 0.40278946693526807; 0.6579957534018397 0.15626724279895354 0.6657286227161041])
```

The above struct is immutable with mutable elements. These mutable elements are placed on the heap, but because we have a immutable struct we can make sure we only have to allocate them once and update inplace from there on as shown below.

```julia
julia> @btime $p.cov .= @SMatrix rand(3,3) # Mantain mutablity of struct params
  23.217 ns (0 allocations: 0 bytes)
3×3 MMatrix{3, 3, Float64, 9} with indices SOneTo(3)×SOneTo(3):
 0.767945  0.270831   0.5915
 0.365027  0.0803052  0.835611
 0.209137  0.339286   0.580083
```

When doing math with an `MMatrix` the result is a `MMatrix` which allocates more memory. Thus whenever we are going to do math with an `MMatrix` it is critical that we be able to convert it into a `SMatrix` with zero allocations. For this to be possible we have to be sure that the compiler knows the exact size of the `MMatrix` before run time. To do this make sure the `MArray` attribute is a concrete type. In the above code you can see that the size of `PointWithCovariance.cov` is a concrete type the compiler knows the exact number of bytes it needs to set asside. We see this with the function `myFunction` below

```julia
julia> function myFunction(p::PointWithCovariance)
           t = @SMatrix rand(3, 3)
           c = SMatrix(p.cov)
           return t * c
       end
julia> @btime myFunction($p)
  30.170 ns (0 allocations: 0 bytes)
3×3 SMatrix{3, 3, Float64, 9} with indices SOneTo(3)×SOneTo(3):
 0.25801   0.265206  0.598741
 0.275018  0.176434  0.380314
 0.561555  0.209735  0.526756
```


## `begin...end` Blocks

When diagnosing memory usage in a large function which is not easily broken appart using `begin...end` code blocks helps track down which part of the function is allocating the most memory. You can wrap different parts of the function in `begin...end` blocks and then time and check memory usage using the `@btime` macro.

```julia
julia> function myLargeFunction(x)
           # Do alot here
           @btime begin
              # Check if this code requires a lot of memory allocation
           end
           # Do more here
       end
```

## Track Each Lines Allocation of Memory

When tracking the memory through multiple files its sometimes easiest to use the `--track-allocation=user` when you start up julia. This will generate `.mem` files with line by line memory usage.
Julia's memory management documentation [here](https://docs.julialang.org/en/v1/manual/profile/#Memory-allocation-analysis).

## Garbage Collection

When all else fails and you're unable to remove the memory allocations (sometimes this occurs inside some other library which you are unable to fix eg. `ProtoBuf.jl`) you can use `GC.gc(false)` inside youre loop. Specifying this means that rather than wait until memory is full and the garbage collector runs taking a long time and throwing off the loop rate, you can run the garbage collector each loop but only destroy "young" objects. With few allocations a loop this means the gc can clear the memory quickly and run at a near constant quick rate, rather than running quickly then stopping for long periods. For more on this see [this](https://discourse.julialang.org/t/how-difficult-is-it-to-write-allocation-free-code-to-avoid-gc-pauses/40235/6) julia discourse disscussion.