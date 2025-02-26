#include "nsgaii.hpp"
#include <random>
#include <algorithm>
#include <cmath>

namespace parafoil
{

    // Stream operators implementation
    std::ostream &operator<<(std::ostream &os, const Individual &ind)
    {
        os << "[";
        for (size_t i = 0; i < ind.size(); ++i)
        {
            os << ind[i];
            if (i < ind.size() - 1)
                os << ", ";
        }
        return os << "]";
    }

    std::ostream &operator<<(std::ostream &os, const NSGAIIIndividual &ind)
    {
        return os << "x: " << ind.x << ", f: " << ind.f
                  << ", rank: " << ind.rank
                  << ", crowding: " << ind.crowding_distance;
    }

    NSGAII::NSGAII(ObjectiveFunction obj_func, int num_variables, int num_objectives,
                   const Individual &lower_bounds, const Individual &upper_bounds)
        : objective_function(obj_func), num_vars(num_variables), num_objs(num_objectives), lower_bounds(lower_bounds), upper_bounds(upper_bounds)
    {
    }

    std::vector<NSGAIIIndividual> NSGAII::run(const NSGAIIParameters &params)
    {
        std::cout << "Starting NSGA-II run..." << std::endl;

        // Initialize population
        std::vector<NSGAIIIndividual> population;
        population.reserve(params.population_size);
        std::cout << "Creating initial population..." << std::endl;

        for (int i = 0; i < params.population_size; ++i)
        {
            try
            {
                auto ind = create_random_individual();
                std::cout << "Created individual " << i << ": x=" << ind.x << ", f=" << ind.f << std::endl;
                population.push_back(ind);
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error creating individual " << i << ": " << e.what() << std::endl;
                throw;
            }
        }

        std::cout << "Initial population created. Size: " << population.size() << std::endl;

        // Main loop
        for (int gen = 0; gen < params.max_generations; ++gen)
        {
            std::cout << "\nGeneration " << gen << std::endl;

            try
            {
                // Create offspring population through crossover and mutation
                std::cout << "Creating offspring..." << std::endl;
                std::vector<NSGAIIIndividual> offspring;
                offspring.reserve(params.population_size);

                while (offspring.size() < params.population_size)
                {
                    // Tournament selection
                    auto parent1 = population[rand() % population.size()];
                    auto parent2 = population[rand() % population.size()];

                    if (static_cast<double>(rand()) / RAND_MAX < params.crossover_rate)
                    {
                        auto [child1, child2] = crossover(parent1, parent2);

                        if (static_cast<double>(rand()) / RAND_MAX < params.mutation_rate)
                        {
                            mutate(child1);
                        }
                        if (static_cast<double>(rand()) / RAND_MAX < params.mutation_rate)
                        {
                            mutate(child2);
                        }

                        offspring.push_back(child1);
                        if (offspring.size() < params.population_size)
                        {
                            offspring.push_back(child2);
                        }
                    }
                    else
                    {
                        offspring.push_back(parent1);
                        if (offspring.size() < params.population_size)
                        {
                            offspring.push_back(parent2);
                        }
                    }
                }

                // Combine parent and offspring populations
                population.insert(population.end(), offspring.begin(), offspring.end());

                // Non-dominated sorting
                std::cout << "Non-dominated sorting..." << std::endl;
                auto fronts = fast_non_dominated_sort(population);

                std::cout << "Number of fronts: " << fronts.size() << std::endl;
                for (size_t i = 0; i < fronts.size(); ++i)
                {
                    std::cout << "Front " << i << " size: " << fronts[i].size() << std::endl;
                }

                // Select next generation
                std::vector<NSGAIIIndividual> next_generation;
                next_generation.reserve(params.population_size);

                for (auto &front : fronts)
                {
                    if (next_generation.size() + front.size() <= params.population_size)
                    {
                        // Add whole front
                        for (int idx : front)
                        {
                            next_generation.push_back(population[idx]);
                        }
                    }
                    else
                    {
                        // Calculate crowding distance
                        std::vector<NSGAIIIndividual> last_front;
                        for (int idx : front)
                        {
                            last_front.push_back(population[idx]);
                        }
                        calculate_crowding_distance(last_front);

                        // Sort by crowding distance
                        std::sort(last_front.begin(), last_front.end(),
                                  [](const auto &a, const auto &b)
                                  {
                                      return a.crowding_distance > b.crowding_distance;
                                  });

                        // Add best individuals to fill population
                        size_t to_add = params.population_size - next_generation.size();
                        for (size_t i = 0; i < to_add; ++i)
                        {
                            next_generation.push_back(last_front[i]);
                        }
                        break;
                    }
                }

                population = next_generation;

                if (params.verbosity != Verbosity::SILENT)
                {
                    std::cout << "Generation " << gen << ", Population size: "
                              << population.size() << std::endl;
                }
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error in generation " << gen << ": " << e.what() << std::endl;
                throw;
            }
        }

        return population;
    }

    std::vector<std::vector<int>> NSGAII::fast_non_dominated_sort(
        std::vector<NSGAIIIndividual> &population)
    {
        std::vector<std::vector<int>> fronts;
        if (population.empty())
        {
            return fronts;
        }

        // Reset all individuals
        for (size_t i = 0; i < population.size(); ++i)
        {
            population[i].dominated_solutions.clear();
            population[i].domination_count = 0;
            population[i].rank = -1;
        }

        // Find domination relationships
        for (size_t i = 0; i < population.size(); ++i)
        {
            for (size_t j = i + 1; j < population.size(); ++j)
            {
                if (dominates(population[i], population[j]))
                {
                    population[i].dominated_solutions.push_back(j);
                    population[j].domination_count++;
                }
                else if (dominates(population[j], population[i]))
                {
                    population[j].dominated_solutions.push_back(i);
                    population[i].domination_count++;
                }
            }
        }

        // Find first front
        std::vector<int> current_front;
        for (size_t i = 0; i < population.size(); ++i)
        {
            if (population[i].domination_count == 0)
            {
                population[i].rank = 0;
                current_front.push_back(i);
            }
        }

        // Add first front
        fronts.push_back(current_front);

        // Generate subsequent fronts
        size_t front_index = 0;
        while (!fronts[front_index].empty())
        {
            std::vector<int> next_front;

            // Check each individual in current front
            for (int p : fronts[front_index])
            {
                // Check each individual dominated by p
                for (int q : population[p].dominated_solutions)
                {
                    population[q].domination_count--;
                    if (population[q].domination_count == 0)
                    {
                        population[q].rank = front_index + 1;
                        next_front.push_back(q);
                    }
                }
            }

            if (next_front.empty())
            {
                break; // No more fronts to add
            }

            fronts.push_back(next_front);
            front_index++;
        }

        return fronts;
    }

    void NSGAII::calculate_crowding_distance(std::vector<NSGAIIIndividual> &front)
    {
        if (front.empty())
            return;

        const size_t n = front.size();
        if (n <= 2)
        {
            for (auto &ind : front)
            {
                ind.crowding_distance = std::numeric_limits<double>::infinity();
            }
            return;
        }

        // Initialize distances
        for (auto &ind : front)
        {
            ind.crowding_distance = 0.0;
        }

        // Calculate crowding distance for each objective
        for (int m = 0; m < num_objs; ++m)
        {
            // Sort by m-th objective
            std::sort(front.begin(), front.end(),
                      [m](const auto &a, const auto &b)
                      { return a.f[m] < b.f[m]; });

            // Set boundary points to infinity
            front.front().crowding_distance = std::numeric_limits<double>::infinity();
            front.back().crowding_distance = std::numeric_limits<double>::infinity();

            // Calculate crowding distances
            double obj_range = front.back().f[m] - front.front().f[m];
            if (obj_range > 1e-10)
            { // Avoid division by zero
                for (size_t i = 1; i < n - 1; ++i)
                {
                    front[i].crowding_distance +=
                        (front[i + 1].f[m] - front[i - 1].f[m]) / obj_range;
                }
            }
        }
    }

    bool NSGAII::dominates(const NSGAIIIndividual &ind1, const NSGAIIIndividual &ind2)
    {
        bool at_least_one_better = false;
        for (size_t i = 0; i < ind1.f.size(); ++i)
        {
            if (ind1.f[i] > ind2.f[i])
                return false;
            if (ind1.f[i] < ind2.f[i])
                at_least_one_better = true;
        }
        return at_least_one_better;
    }

    NSGAIIIndividual NSGAII::create_random_individual()
    {
        NSGAIIIndividual ind(num_vars, num_objs);
        for (int i = 0; i < num_vars; ++i)
        {
            double r = static_cast<double>(rand()) / RAND_MAX;
            ind.x[i] = lower_bounds[i] + r * (upper_bounds[i] - lower_bounds[i]);
        }
        ind.f = objective_function(ind.x);
        return ind;
    }

    std::pair<NSGAIIIndividual, NSGAIIIndividual> NSGAII::crossover(
        const NSGAIIIndividual &parent1, const NSGAIIIndividual &parent2)
    {
        NSGAIIIndividual child1(num_vars, num_objs);
        NSGAIIIndividual child2(num_vars, num_objs);

        // Simulated Binary Crossover (SBX)
        double eta_c = 20.0;
        for (int i = 0; i < num_vars; ++i)
        {
            double r = static_cast<double>(rand()) / RAND_MAX;
            double beta = (r <= 0.5) ? std::pow(2 * r, 1.0 / (eta_c + 1)) : std::pow(1.0 / (2 * (1 - r)), 1.0 / (eta_c + 1));

            child1.x[i] = 0.5 * ((1 + beta) * parent1.x[i] + (1 - beta) * parent2.x[i]);
            child2.x[i] = 0.5 * ((1 - beta) * parent1.x[i] + (1 + beta) * parent2.x[i]);

            // Bound checking
            child1.x[i] = std::clamp(child1.x[i], lower_bounds[i], upper_bounds[i]);
            child2.x[i] = std::clamp(child2.x[i], lower_bounds[i], upper_bounds[i]);
        }

        child1.f = objective_function(child1.x);
        child2.f = objective_function(child2.x);

        return {child1, child2};
    }

    void NSGAII::mutate(NSGAIIIndividual &individual)
    {
        // Polynomial mutation
        double eta_m = 20.0;
        for (int i = 0; i < num_vars; ++i)
        {
            if (static_cast<double>(rand()) / RAND_MAX < 1.0 / num_vars)
            {
                double r = static_cast<double>(rand()) / RAND_MAX;
                double delta = (r < 0.5) ? std::pow(2 * r, 1.0 / (eta_m + 1)) - 1 : 1 - std::pow(2 * (1 - r), 1.0 / (eta_m + 1));

                individual.x[i] += delta * (upper_bounds[i] - lower_bounds[i]);
                individual.x[i] = std::clamp(individual.x[i],
                                             lower_bounds[i], upper_bounds[i]);
            }
        }
        individual.f = objective_function(individual.x);
    }

} // namespace parafoil