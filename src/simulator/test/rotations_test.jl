include("../rotations.jl")
using Rotations
using Random
using ForwardDiff
using FiniteDiff 
using LinearAlgebra
using Test

##
Random.seed!(1)
a = randn(3)
b = randn(3)
@test skew(a)*b ≈ cross(a,b)
@test skew(b)*a ≈ -cross(a,b)

q1 = normalize(randn(4))
q2 = normalize(randn(4))
@test lmat(q1)*q2 ≈ Rotations.params(QuatRotation(q1) * QuatRotation(q2))
@test rmat(q2)*q1 ≈ Rotations.params(QuatRotation(q1) * QuatRotation(q2))
@test lmat(q1)'q2 ≈ Rotations.params(QuatRotation(q1) \ QuatRotation(q2))
@test rmat(q2)'q1 ≈ Rotations.params(QuatRotation(q1) / QuatRotation(q2))

@test tmat()*q1 ≈ Rotations.params(inv(QuatRotation(q1)))
@test hmat()*a ≈ [0; a]

@test quat2rotmat(q1) ≈ Matrix(QuatRotation(q1))
@test G(q1) ≈ Rotations.∇differential(QuatRotation(q1))

phi = randn(3)
dcay(phi) ≈ dcay2(phi)
@test cay(phi) ≈ Rotations.params(Rotations.CayleyMap()(phi))
@test icay(q1) ≈ inv(Rotations.CayleyMap())(QuatRotation(q1))
@test icay(cay(phi)) ≈ phi
@test abs(cay(icay(q1))'q1) - 1 < 1e-10
@test abs(cay(icay(q2))'q2) - 1 < 1e-10

x = randn(3)
@test ForwardDiff.jacobian(cay, phi) ≈ dcay(phi)
rotate(q1,x)

# Use FiniteDiff because ForwardDiff is slow on this Jacobian
@test FiniteDiff.finite_difference_jacobian(q->rotate(q,x), q1) ≈ drotate(q1,x)

using Symbolics
@variables q[1:4], x[1:3], phi[1:3]
q = Symbolics.scalarize(q)
x = Symbolics.scalarize(x)
phi = Symbolics.scalarize(phi)

drotate(q,x)
cay(phi)
G(q)

ϕ = randn(3)
dcay(ϕ) ≈ dcay2(ϕ)

##
# Make the shared lib
curdir = pwd()
cd(joinpath(@__DIR__, ".."))
run(`gcc -c -Wall -Werror -fpic rotations.c -o rotations.o`)
run(`gcc -shared -o rotations.so rotations.o`)
cd(curdir)

const librotations = joinpath(@__DIR__, "..", "rotations.so")
isfile(librotations)

Q = zeros(3,3)
L = zeros(4,4)
R = zeros(4,4)
Gmat = zeros(4,3)
S = zeros(3,3)
D = zeros(4,3)
Dr = zeros(3,4)
x = randn(3)
phi = randn(3)
q = normalize(randn(4))
x2 = zeros(3)

ccall((:qmat_skewmat, librotations), Cvoid, 
    (Ref{Cdouble}, Ref{Cdouble}),
    S, x
)
@test S ≈ skew(x)

ccall((:qmat_lmat, librotations), Cvoid, 
    (Ref{Cdouble}, Ref{Cdouble}),
    L, q
)
@test L ≈ lmat(q)

ccall((:qmat_rmat, librotations), Cvoid, 
    (Ref{Cdouble}, Ref{Cdouble}),
    R, q
)
@test R ≈ rmat(q)

ccall((:qmat_gmat, librotations), Cvoid, 
    (Ref{Cdouble}, Ref{Cdouble}),
    Gmat, q
)
@test Gmat ≈ G(q)

ccall((:qmat_cay, librotations), Cvoid, 
    (Ref{Cdouble}, Ref{Cdouble}),
    q, phi 
)
@test q ≈ cay(phi) 

ccall((:qmat_icay, librotations), Cvoid, 
    (Ref{Cdouble}, Ref{Cdouble}),
    phi, q
)
@test phi ≈ icay(q) 

ccall((:qmat_dcay, librotations), Cvoid, 
    (Ref{Cdouble}, Ref{Cdouble}),
    D, phi 
)
@test D ≈ dcay(phi) 

ccall((:qmat_rotate, librotations), Cvoid, 
    (Ref{Cdouble}, Ref{Cdouble}, Ref{Cdouble}),
    x2, q, x, 
)
@test x2 ≈ rotate(q,x) 

ccall((:qmat_drotate, librotations), Cvoid, 
    (Ref{Cdouble}, Ref{Cdouble}, Ref{Cdouble}),
    Dr, q, x, 
)
@test Dr ≈ drotate(q, x)

ccall((:qmat_quat2rotmat, librotations), Cvoid, 
    (Ref{Cdouble}, Ref{Cdouble}),
    Q, q, 
)
@test Q ≈ quat2rotmat(q)