module;
#include <functional>
#include <utility>
#include <valarray>
#include <map>
#include <tuple>
#include <vector>
#include <algorithm>
#include <iostream>


module parafoil;
// asumming x-y is a wind axis, 
// x -> wind speed in -x direction
// p = [-Vw, 0]

// thus state vector add para.T() aims to [0, 0, 0]
// where \omega --> [-pi, pi] and in the sense of absolute angle to minimize total relative velocity when touch down
parafoil_flight_state transfer(const parafoil_state& para, const double u, const double Vw, const double dt, const parafoil_flight_state& state_input)
{
    // state input
    auto u_func = [u](double t) -> vec { return { u }; };
    auto wind_func = [Vw](double t) -> vec { return { -Vw, 0.0 }; };
    auto [t0, x0] = state_input;    
        auto t = t0 + dt;

    // integrate state vector with ode45
    auto integrate_dt = std::copysign(std::min(std::abs(dt) / 10, 1e-3), dt);
    auto traj = ode45(t0, t, integrate_dt, x0, u_func, wind_func, para);
    // get the last state vector
    // auto [x, uu, pp] = traj[t];
    auto x = std::get<0>(traj[t]);
    return {t, x};
}

// 代价函数?



// dp算法?

