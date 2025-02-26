#ifndef ODE45_HPP
#define ODE45_HPP

#include <vector>
#include <functional>
#include <map>
#include <tuple>
#include "dynamics.hpp"

namespace parafoil {

using time_point_func = std::function<vec(double)>;
using state_input_wind = std::tuple<vec, vec, vec>;
using dynamic_trajectory = std::map<double, state_input_wind>;
using parafoil_flight_state = std::pair<double, vec>;

state_input_wind ode45_step(const double t, const double dt, const vec &x, 
                           const time_point_func u, const time_point_func p, 
                           const parafoil_state &para);

dynamic_trajectory ode45(const double t0, const double tf, const double dt, 
                        const vec &x0, const time_point_func u, const time_point_func p, 
                        const parafoil_state &para);

} // namespace parafoil

#endif // ODE45_HPP 