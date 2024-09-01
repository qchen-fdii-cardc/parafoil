#include <iostream>
#include <chrono>
#include <functional>
#include <cstdlib>

import parafoil;


double next_double(const double ub = 1.0, const double lb = 0.0) {
    const double ratio = 1.0 * rand() / RAND_MAX;
    return lb + (ub - lb) * ratio;
}
double rad2deg(const double rad) { return rad * 180.0 / 3.1415926; };
double deg2rad(const double deg) { return deg * 3.1415926 / 180.0; };

auto make_random_control(const double bound) {
    return [bound](double t) -> vec {return { next_double(bound, 0.0) }; };
}

auto make_random_wind(const double bound) {
    return [bound](double t) -> vec {return { next_double(bound, -bound), next_double(bound, -bound) }; };
}

// coerce function for type T
template<typename T>
T coerce(const T&x, const T&lb, const T&ub) {
    return std::max(lb, std::min(ub, x));
}


auto collocation_points_control(const parafoil_state& para, const vec& control_collocation_points) {
    return [para, control_collocation_points](double t) -> vec {
        const auto T = para.T();
        const auto n = control_collocation_points.size();
        const auto dt = T / (n-1);
        const auto idx = coerce<int>(static_cast<int>(std::floor(t / dt)), 0, n-1);

        return { control_collocation_points[idx] };
    };
}

auto maximum_op_collocation(const double bound, const int n) {
    vec control_collocation_points(n);
    for (int i = 0; i < n; i++) {
        control_collocation_points[i] = next_double()>0.5 ? bound : -bound;
    }
    return control_collocation_points;
}


int main(int argc, char* argv[]) {

    // parse a seed from command line
    if (argc > 1) {
        srand(std::atoi(argv[1]));
    } else {
        srand(0);
    }

    // parse second argument as number of collocation points
    int n = 10;
    if (argc > 2) {
        n = std::atoi(argv[2]);
    }


    const vec x = { 0.0, 0.0, deg2rad(45.0) };


    constexpr parafoil_state para = { 300.0, 10.0, 25.0 };


    //define control input as lambda function
    // auto u = [](double t) -> vec { return { next_double(deg2rad(25.0), -deg2rad(25.0)) }; };
    // auto u0 = [](double t) -> vec { return { 0.0 }; };
    // auto u = make_random_control(deg2rad(25.0));

    const auto control_collocation_points = maximum_op_collocation(deg2rad(25.0), n);
    auto u = collocation_points_control(para, control_collocation_points);
    // auto wind = [](double t) -> vec { return { next_double(1.0, -1.0), next_double(1.0, -1.0) }; };
    // auto wind0 = [](double t) -> vec { return { 0.0, 0.0 }; };
    // auto wind = make_random_wind(1.0);
    auto wind = make_random_wind(1.0);

    // measure time
    const auto start = std::chrono::high_resolution_clock::now();

    auto trajectory = ode45(para.T(), 0.0, -0.01, x, u, wind, para);

    const auto end = std::chrono::high_resolution_clock::now();


    for (auto& [t, scw] : trajectory) {
        auto [x, u, p] = scw;
        std::cout <<
            std::format("{:12.8f},{:12.8f},{:12.8f},{:12.8f}, {:12.8f},{:12.8f},{:12.8f}, {:12.8f}\n",
                t, para.h(t), x[0], x[1], rad2deg(x[2]), rad2deg(u[0]), p[0], p[1]);
    }

    // std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms\n";

}