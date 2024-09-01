//
// Created by dingd on 2024/9/1.
//
module;
#include <functional>
#include <utility>
#include <valarray>
#include <map>
#include <tuple>


export module parafoil;

export typedef std::valarray<double> vec;

export typedef  std::function<vec(double)> time_point_func;
export struct parafoil_state
{
    double h0; // initial height
    double vz; // vertical velocity
    double V; // horizontal velocity
    [[nodiscard]] double T() const ;
    [[nodiscard]] double h(double t) const;
};

export vec parafoil_simple_ode(double t, const vec &x, const vec &u, const vec &p, const parafoil_state &para);

export typedef std::tuple<vec, vec, vec> state_input_wind;
export typedef std::map<double, state_input_wind> dynamic_trajectory;

export state_input_wind  ode45_step(double t, double dt, const vec &x, time_point_func u, time_point_func p, const parafoil_state &para);
export dynamic_trajectory ode45(double t0, double tf, double dt, const vec &x0, time_point_func u, time_point_func p, const parafoil_state &para);