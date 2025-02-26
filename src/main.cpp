#include "dynamics.hpp"
#include "ode45.hpp"
#include "dynamicprogramming.hpp"
#include <iostream>
#include <vector>
#include <cmath>

using namespace parafoil;

int main()
{
    // Initialize parafoil state
    parafoil_state para{
        .h0 = 1000.0, // initial height (m)
        .vz = 5.0,    // vertical velocity (m/s)
        .V = 10.0     // airspeed (m/s)
    };

    // Initial state vector [x, y, omega]
    vec x0 = {0.0, 0.0, 0.0};

    // Time settings
    double t0 = 0.0;
    double tf = para.T(); // Final time based on height and vertical velocity
    double dt = 0.1;      // Time step

    // Control input function (constant turn rate)
    auto u_func = [](double t) -> vec
    { return {0.1}; }; // Small constant turn rate

    // Wind conditions
    double wind_speed = 2.0; // m/s
    auto wind_func = [wind_speed](double t) -> vec
    { return {-wind_speed, 0.0}; };

    // Simulate trajectory
    auto trajectory = ode45(t0, tf, dt, x0, u_func, wind_func, para);

    // Print results
    std::cout << "Parafoil trajectory simulation:\n";
    std::cout << "Time\tX\tY\tOmega\tHeight\n";

    for (const auto &[t, state_tuple] : trajectory)
    {
        const auto &state = std::get<0>(state_tuple);
        std::cout << t << "\t"
                  << state[0] << "\t"
                  << state[1] << "\t"
                  << state[2] << "\t"
                  << para.h(t) << "\n";
    }

    return 0;
}