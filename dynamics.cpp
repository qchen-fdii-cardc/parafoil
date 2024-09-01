//
// Created by dingd on 2024/9/1.
//
module;
#include <cassert>
#include <cstdlib>
module parafoil;


double parafoil_state::T() const { return h0 / vz; }

double parafoil_state::h(const double t) const { return h0 - vz * t; }

vec parafoil_simple_ode(const double t, const vec &x, const vec &u, const vec &p, const parafoil_state &para)
{
    // x -> state vector, [x, y, omega]
    // u -> control input, dot-omega
    // p -> wind vector, [wx, wy]
    assert(x.size() == 3 && "State vector must have 3 elements");
    assert(u.size() == 1 && "Control input must have 1 element");
    assert(p.size() == 2 && "Wind vector must have 2 elements");
    const auto n = x.size();

    if (t > para.T())
    {
        return {0.0, 0.0, 0.0};
    }

    vec dx(n);
    const auto omega = x[2];

    // copy x to dx
    dx[0] = para.V * cos(omega) + p[0];
    dx[1] = para.V * sin(omega) + p[1];
    dx[2] = u[0];

    return dx;
}
