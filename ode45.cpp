
module;
#include <utility>
#include <valarray>
#include <iostream>

module parafoil;

state_input_wind ode45_step(const double t, const double dt, const vec &x, const time_point_func u,
                            const time_point_func p, const parafoil_state &para) {
    auto n = x.size();
    vec ut = u(t);
    vec pt = p(t);
    const auto k1 = parafoil_simple_ode(t, x, u(t), p(t), para);
    const auto k2 = parafoil_simple_ode(t + dt / 2, x + dt / 2 * k1, u(t + dt / 2), p(t + dt / 2), para);
    const auto k3 = parafoil_simple_ode(t + dt / 2, x + dt / 2 * k2, u(t + dt / 2), p(t + dt / 2), para);
    const auto k4 = parafoil_simple_ode(t + dt, x + dt * k3, u(t + dt), p(t + dt), para);

    vec x_plus_dt = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);

    return {x_plus_dt, ut, pt};
}

bool not_reach_end(const double t, const double tf, const double dt) {
    return (t < tf && dt > 0) || (t > tf && dt < 0);
}

dynamic_trajectory ode45(const double t0, const double tf, const double dt, const vec &x0, const time_point_func u,
                         const time_point_func p, const parafoil_state &para) {
    auto t = t0;
    auto x = x0;
    dynamic_trajectory this_trajectory;
    this_trajectory[t] = {x, u(t), p(t)};

    while (not_reach_end(t, tf, dt)) {
        const auto dt_modified = std::copysign(std::min(std::abs(dt), std::abs(tf - t)), dt);
        t += dt_modified;
        auto x_plus_dt = ode45_step(t, dt_modified, x, u, p, para);
        this_trajectory[t] = x_plus_dt;
        x = std::get<0>(x_plus_dt);
    }

    return this_trajectory;
}
