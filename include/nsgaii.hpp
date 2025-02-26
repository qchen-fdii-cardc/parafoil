#ifndef NSGAII_HPP
#define NSGAII_HPP

#include <vector>
#include <valarray>
#include <functional>
#include <iostream>
#include <limits>

namespace parafoil {

using Individual = std::valarray<double>;
using Objectives = std::valarray<double>;
using ObjectiveFunction = std::function<Objectives(const Individual&)>;

enum class Verbosity {
    SILENT,
    PROGRESS,
    DEBUG
};

struct NSGAIIParameters {
    int population_size;
    int max_generations;
    double mutation_rate;
    double crossover_rate;
    Verbosity verbosity;
};

struct NSGAIIIndividual {
    Individual x;
    Objectives f;
    int rank;
    double crowding_distance;
    std::vector<int> dominated_solutions;
    int domination_count;

    NSGAIIIndividual(int D, int N) 
        : x(D), f(N), rank(0), crowding_distance(0.0), domination_count(0) {}

    bool operator<(const NSGAIIIndividual& other) const {
        if (rank != other.rank) return rank < other.rank;
        return crowding_distance > other.crowding_distance;
    }
};

// Stream operators
std::ostream& operator<<(std::ostream& os, const Individual& ind);
std::ostream& operator<<(std::ostream& os, const Objectives& obj);
std::ostream& operator<<(std::ostream& os, const NSGAIIIndividual& ind);

class NSGAII {
public:
    NSGAII(ObjectiveFunction obj_func, int num_variables, int num_objectives,
           const Individual& lower_bounds, const Individual& upper_bounds);

    std::vector<NSGAIIIndividual> run(const NSGAIIParameters& params);

private:
    ObjectiveFunction objective_function;
    int num_vars;
    int num_objs;
    Individual lower_bounds;
    Individual upper_bounds;

    std::vector<std::vector<int>> fast_non_dominated_sort(std::vector<NSGAIIIndividual>& population);
    void calculate_crowding_distance(std::vector<NSGAIIIndividual>& front);
    bool dominates(const NSGAIIIndividual& ind1, const NSGAIIIndividual& ind2);
    NSGAIIIndividual create_random_individual();
    std::pair<NSGAIIIndividual, NSGAIIIndividual> crossover(
        const NSGAIIIndividual& parent1, const NSGAIIIndividual& parent2);
    void mutate(NSGAIIIndividual& individual);
};

} // namespace parafoil

#endif // NSGAII_HPP 