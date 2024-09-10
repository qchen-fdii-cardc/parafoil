#include <valarray>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <numeric>
#include <functional>
#include <cmath>
#include <vector>
#include <string>
#include <tuple>

import parafoil;

typedef std::valarray<double> Individual;
typedef std::function<double(Individual)> ObjectiveFunction;
typedef std::tuple<Individual, double> IndividualObjective;

enum Verbosity {
    SILENT,
    PROGRESS,
    DEBUG
};

struct DEParameters {
    int NP;
    double F;
    double CR;
    int Generations;
    Verbosity verbosity;
};

struct Problem {
    ObjectiveFunction obj;
    Individual lower_bound;
    Individual upper_bound;
    int D;
    // constructor
    Problem(int D) : D(D) {
        lower_bound = Individual(D);
        upper_bound = Individual(D);
    }
};

IndividualObjective differential_evolution(Problem prob, DEParameters params) {
    if (params.verbosity == Verbosity::DEBUG) {
        std::cout << "Differential evolution parameters:" << std::endl;
        std::cout << "NP: " << params.NP << std::endl;
        std::cout << "F: " << params.F << std::endl;
        std::cout << "CR: " << params.CR << std::endl;
        std::cout << "Generations: " << params.Generations << std::endl;
    }

    // Initialize population
    ObjectiveFunction obj = prob.obj;
    std::vector<IndividualObjective> population(params.NP);
    
    auto obj_idx = [population](int idx) {
        if (idx < 0 || idx >= population.size()) {
            return std::numeric_limits<double>::infinity();
        }
        return std::get<1>(population[idx]);
    };


    int best_idx = 0;
    for (int i = 0; i < params.NP; ++i) {
        auto ind = Individual(prob.D);
        for (int j = 0; j < prob.D; ++j) {
            ind[j] = prob.lower_bound[j] + (prob.upper_bound[j] - prob.lower_bound[j]) * rand() / RAND_MAX;
        }
        auto objective_value = obj(ind);
        population[i] = {ind, objective_value};
        if (best_idx == -1 || objective_value < obj_idx(best_idx)) {
            best_idx = i;
        }
    }

    // Main loop
    for (int i = 0; i < params.Generations; ++i) {
        for (int j = 0; j < params.NP; ++j) {
            // Select three random indices
            int r1, r2, r3;
            do {
                r1 = rand() % params.NP;
            } while (r1 == j);
            do {
                r2 = rand() % params.NP;
            } while (r2 == j || r2 == r1);
            do {
                r3 = rand() % params.NP;
            } while (r3 == j || r3 == r1 || r3 == r2);

            // Mutation
            Individual mutant = std::get<0>(population[r1]) + params.F * (std::get<0>(population[r2]) - std::get<0>(population[r3]));

            // Crossover
            Individual trial = std::get<0>(population[j]);
            int k = rand() % prob.D;
            for (int k = 0; k < prob.D; ++k) {
                if (rand() / RAND_MAX < params.CR || k == i) {
                    trial[k] = mutant[k];
                }
            }
            auto trial_val = obj(trial);
            // Selection
            if (trial_val < std::get<1>(population[j])) {
                population[j] = {trial, trial_val};

                // update best
                if (trial_val < obj_idx(best_idx)) {
                    best_idx = j;
                }
            }
        }
        if (params.verbosity == Verbosity::PROGRESS || params.verbosity == Verbosity::DEBUG) {
            auto [best_x, best_obj] = population[best_idx];
            std::cout << "Generation " << i << ": "  << best_obj << " <-- " <<best_x << std::endl;
        }
    }

    // Return the best individual
    return population[best_idx];
}


int main() {
    // rand with time seed
    srand(time(NULL));


    DEParameters params;
    params.NP = 100;
    params.F = 0.5;
    params.CR = 0.9;
    params.Generations = 200;
    params.verbosity = Verbosity::DEBUG;

    Problem problem(10);
    problem.obj = [](Individual x) {
        return (x * x).sum();
    };
    problem.lower_bound = -100.0;
    problem.upper_bound = 100.0;

    auto [best_x, obj_x] = differential_evolution(problem, params);

    std::cout << "Best individual: " << best_x << std::endl;
    std::cout << "Objective value: " << obj_x << std::endl;

    return 0;
}