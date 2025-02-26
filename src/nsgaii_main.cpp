// NSGA II main program
#include <algorithm>
#include <functional>
#include <iostream>
#include <iterator>
#include <numeric>
#include <vector>

#include <iostream>
#include <valarray>
#include <algorithm>
#include <iterator>
#include <numeric>
#include <functional>
#include <cmath>
#include <vector>
#include <string>
#include <tuple>
#include <map>

#include "nsgaii.hpp"
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>

using namespace parafoil;

int main()
{
    std::srand(std::time(nullptr));

    // Define the problem
    int num_vars = 30; // Standard for ZDT1
    int num_objs = 2;

    // ZDT1 test problem
    auto objective_function = [](const Individual &x) -> Objectives
    {
        Objectives f(2);
        f[0] = x[0];

        double g = 1.0;
        for (size_t i = 1; i < x.size(); ++i)
        {
            g += 9.0 * x[i] / (x.size() - 1);
        }
        f[1] = g * (1.0 - std::sqrt(x[0] / g));
        return f;
    };

    // Problem bounds
    Individual lower_bounds(num_vars);
    Individual upper_bounds(num_vars);
    lower_bounds = 0.0;
    upper_bounds = 1.0;

    // Create NSGA-II solver
    NSGAII solver(objective_function, num_vars, num_objs,
                  lower_bounds, upper_bounds);

    // Set parameters based on literature recommendations
    NSGAIIParameters params;
    params.population_size = 200;          // Increased for better diversity
    params.max_generations = 1000;         // More generations for convergence
    params.mutation_rate = 1.0 / num_vars; // Standard mutation rate for real-coded GA
    params.crossover_rate = 0.9;           // High crossover rate for exploration
    params.verbosity = Verbosity::PROGRESS;

    std::cout << "Solving ZDT1 problem with optimized parameters..." << std::endl;
    std::cout << "Number of variables: " << num_vars << std::endl;
    std::cout << "Number of objectives: " << num_objs << std::endl;
    std::cout << "Population size: " << params.population_size << std::endl;
    std::cout << "Number of generations: " << params.max_generations << std::endl;
    std::cout << "Mutation rate: " << params.mutation_rate << std::endl;
    std::cout << "Crossover rate: " << params.crossover_rate << std::endl;

    // Run optimization
    auto final_population = solver.run(params);

    // Output Pareto front to file
    std::ofstream outfile("build/pareto_front.dat");
    outfile << std::setprecision(15);
    for (const auto &solution : final_population)
    {
        if (solution.rank == 0)
        {
            outfile << solution.f[0] << " " << solution.f[1] << "\n";
        }
    }
    outfile.close();

    std::cout << "\nPareto front has been written to build/pareto_front.dat" << std::endl;
    return 0;
}