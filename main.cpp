#include <iostream>
#include <chrono>
#include <functional>
#include <cstdlib>

import parafoil;





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


int flight_from_command_line_arguments(int argc, char* argv[], double dt) {

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

    auto trajectory = ode45(para.T(), 0.0, dt, x, u, wind, para);

    const auto end = std::chrono::high_resolution_clock::now();


    for (auto& [t, scw] : trajectory) {
        auto [x, u, p] = scw;
        std::cout <<
            std::format("{:12.8f},{:12.8f},{:12.8f},{:12.8f}, {:12.8f},{:12.8f},{:12.8f}, {:12.8f}\n",
                t, para.h(t), x[0], x[1], rad2deg(x[2]), rad2deg(u[0]), p[0], p[1]);
    }

    std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms\n";

    return 0;
}

int transfer(int argc, char* argv[], int section_count = 10) {
    // return flight_from_command_line_arguments(argc, argv);

    constexpr parafoil_state para = { 300.0, 10.0, 25.0 };    

    const vec x = { 0.0, 0.0, deg2rad(0.0) };
    const parafoil_flight_state flight_state = {para.T(), x};
    constexpr auto Vw = 1.0;
    // const double u = deg2rad(25.0);
    const double Dt = - para.T() / section_count;

    const vec possible_u = {deg2rad(25.0), -deg2rad(25.0), 0};

    auto fs = flight_state;
    std::cout.precision(4);
    std::cout.setf(std::ios::fixed);
    while (true) {
        auto [t1, x1] = fs;
        std::cout << t1 << ", " << x1 << "--->";
        // random select u from possible_u
        auto u = possible_u[rand() % possible_u.size()];
        std::cout << "u: " << u << std::endl;
        if (t1 <= 0.0) {
            break;
        }
        fs = transfer(para, u, Vw, Dt, fs);
    }

    // auto ret = transfer(para, u, Vw, Dt, flight_state);
    // auto [t1, x1] = ret;

    // std::cout vector
    return 0;
}

int main(int argc, char* argv[]) {
    // return flight_from_command_line_arguments(argc, argv, -0.7);
    // random seed with system time
    srand(std::chrono::system_clock::now().time_since_epoch().count());
    return transfer(argc, argv, 50);
}