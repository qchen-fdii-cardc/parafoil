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

export const double M_PI = 3.14159265358979323846;

export double rad2deg(const double rad) { return rad * 180.0 / M_PI; };
export double deg2rad(const double deg) { return deg * M_PI / 180.0; };


export double next_double(const double ub = 1.0, const double lb = 0.0) {
    const double ratio = 1.0 * rand() / RAND_MAX;
    return lb + (ub - lb) * ratio;
}


export double coerce_angle(const double omega)
{
    return std::fmod(omega + M_PI, 2 * M_PI) - M_PI;
}


// define parafoil_flight_state and transfer function
export struct parafoil_flight_state
{
    parafoil_state parafoil;
    vec x; // state vector, [x, y, omega]
    vec u; // control input, dot-omega, u in [t-dt, t]
    vec p; // wind vector, [-Vw, 0]
    double t; // time
    double dt; // time step
};

export parafoil_flight_state transfer(const vector& u, const parafoil_flight_state& state_input);