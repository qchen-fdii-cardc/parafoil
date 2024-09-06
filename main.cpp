#include <iostream>
#include <chrono>
#include <functional>
#include <cstdlib>
#include <vector>
#include <fstream>

import parafoil;


template<typename T>
T get_argument(int argc, char* argv[], int i = 1, T default_value=0) {
    if (argc > i) {
        return (T)std::atof(argv[i]);
    } else {
        return default_value;
    }
}

bool get_bool_argument(int argc, char* argv[], int i = 1) {
    if (argc > i) {
        return std::atoi(argv[i]) != 0;
    } else {
        return false;
    }
}


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
    // if (argc > 1) {
    //     srand(std::atoi(argv[1]));
    // } else {
    //     srand(0);
    // }
    srand(get_argument<int>(argc, argv, 1, 0));

    // parse second argument as number of collocation points
    // int n = 10;
    // if (argc > 2) {
    //     n = std::atoi(argv[2]);
    // }
    int n = get_argument<int>(argc, argv, 2, 10);


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

int transfer_demo(int argc, char* argv[], int section_count = 10) {
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

auto make_arrive_control(const double t3, const double u3, const double t1, const double u1, const parafoil_state& para) {
    return [t3, u3, t1, u1, para](double t) -> vec {
        if (t > para.T() - t1) {
            return {u1};
        } else if (t < t3) {
            return {u3};
        } else {
            return {0.0};
        }
    };
}

// template<typename T>
// T get_argument(int argc, char* argv[], int i = 1, T default_value=0) {
//     if (argc > i) {
//         return (T)std::atof(argv[i]);
//     } else {
//         return default_value;
//     }
// }

double rad2deg_coerce(const double x) {
    return std::fmod(rad2deg(x) + 180.0, 360.0) - 180.0;
}


int main(int argc, char* argv[]) {
    // return flight_from_command_line_arguments(argc, argv, -0.7);
    // random seed with system time
    // srand(std::chrono::system_clock::now().time_since_epoch().count());
    // return transfer_demo(argc, argv, 50);

    bool is_output_trajectory = get_bool_argument(argc, argv, 3);

    const parafoil_state para = { 300.0, 10.0, 25.0 };
    const vec x_T = { 0.0, 0.0, deg2rad(0.0) };
    const double u_bar = deg2rad(25.0);

    const double T = para.T();
    const double tt_max = 2 * M_PI / u_bar;

    const std::vector<double> u_options = { u_bar, -u_bar};
    // wind speed
    const double Vw = get_argument<double>(argc, argv, 1, 1.0); 

    auto wind_func = [Vw](double t) -> vec { return {-Vw, 0 }; };

    const double delta_n = get_argument<double>(argc, argv, 2, 0.01);
    for (auto u1: u_options){
        for (auto u3: u_options){
            for (double n1 = 0; n1 < 1.0; n1 += delta_n){
                const double t1 = tt_max * n1;
                for (double n3 = 0; n3 < 1.0; n3 += delta_n){
                    const double t3 = tt_max * n3;

                    auto u_func = make_arrive_control(t3, u3, t1, u1, para);

                    auto traj = ode45(T, 0.0, -0.1, x_T, u_func, wind_func, para);                    
                    auto x = std::get<0>(traj[0.0]);
                    
                    if (is_output_trajectory) {
                        // generate filename
                        std::string filename = std::format("parafoil_{:04.2f}_{:04.2f}_{:04.2f}_{:04.2f}.csv", t3, u3, t1, u1);
                        std::ofstream file(filename);
                        file << "% " << t3 << ", " << u3 << ", " << t1 << ", " << u1 << "\n";
                        for (auto& [t, scw] : traj) {
                            auto [x, u, p] = scw;
                            file << t << "," << x[0] << "," << x[1] << "," << rad2deg_coerce(x[2]) << "," << rad2deg_coerce(u[0]) << "," << p[0] << "," << p[1] << "\n";
                        }
                        file.close();
                    }


                    auto deg_omega = rad2deg_coerce(x[2]);
                    std::cout << t3 << "," << u3 << ","<< T-t1-t3 << ",0.0," << t1 << "," << u1 << "," << x[0] << "," << x[1] << "," << deg_omega << "\n";
                }
            }
        }
    }


    return 0;
}