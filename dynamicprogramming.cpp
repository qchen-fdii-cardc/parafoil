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


parafoil_flight_state transfer(const vector& u, const parafoil_flight_state& state_input)
{
    parafoil_flight_state state = state_input;
    state.u = u;
    state.t += state.dt;

    // integrate state vector with ode45
    auto integrate_dt = std::max(state.dt / 10, 1e-3);
    auto traj = ode45(state.t, state.dt, integrate_dt, state.x, state.u, state.p, state.parafoil);

    // get the last state vector
    auto (t, (x, u, p)) = traj.rbegin();
    state.x = x;
    return state;
}

