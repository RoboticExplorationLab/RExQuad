function qinv(q)
    return [q[1]; -q[2:4]]
end

function skew(x)
    [
        0 -x[3] x[2]
        x[3] 0 -x[1]
        -x[2] x[1] 0
    ]
end

function lmat(q)
    s = q[1]
    v = q[2:4]
    [
        s -v'
        v s*I + skew(v)
    ]
end

function rmat(q)
    s = q[1]
    v = q[2:4]
    [
        s -v'
        v s*I - skew(v)
    ]
end

tmat() = Diagonal([1,-1,-1,-1])
hmat() = [zeros(1,3); I(3)]

function quat2rotmat(q)
    hmat()'*tmat()*lmat(q)*tmat()*lmat(q)*hmat()
end

function G(q)
    lmat(q)*hmat()
end

function cay(phi)
    1/sqrt(1+phi'phi) * [1; phi]
end

function dcay(phi)
    1/sqrt(1+phi'phi) * [zeros(1,3); I(3)] - (((1+phi'phi)^(-3/2))*[1; phi])*phi'
end

function dcay2(phi)
    m = 1/sqrt(1+phi'phi)
    m3 = -m^3
    D = [
        m3 * phi[1] m3 * phi[2] m3 * phi[3]
        m3 * phi[1]*phi[1] m3 * phi[1]*phi[2] m3 * phi[1]*phi[3]
        m3 * phi[2]*phi[1] m3 * phi[2]*phi[2] m3 * phi[2]*phi[3]
        m3 * phi[3]*phi[1] m3 * phi[3]*phi[2] m3 * phi[3]*phi[3]
    ]
    D[2] += m
    D[7] += m
    D[12] += m
    D
end

function icay(q)
    q[2:4] ./ q[1]
end

function rotate(q,x)
    quat2rotmat(q)*x
end

function drotate(q,x)
    2hmat()'rmat(q)'rmat(hmat()*x)
end