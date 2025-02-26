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


import parafoil;

typedef std::valarray<double> Individual;
typedef std::valarray<double> Objectives;
typedef std::function<Objectives(Individual)> ObjectiveFunction;

enum Verbosity {
    SILENT,
    PROGRESS,
    DEBUG
};

struct NSGAIIParameters {
    int NP;
    int Generations;
    Verbosity verbosity;
};

struct Problem {
    ObjectiveFunction obj;
    Individual lower_bound;
    Individual upper_bound;
    int N;
    int D;
    // constructor
    Problem(int d, int n) : D(d), N(n) {
        lower_bound = Individual(D);
        upper_bound = Individual(D);        
    }

};

struct NSGAIIIndividual {
    Individual x;
    Objectives f;
    int rank;
    double crowding_distance;
    bool operator<(const NSGAIIIndividual& other) const {
        return rank < other.rank || (rank == other.rank && crowding_distance > other.crowding_distance);
    }
    // constructor
    NSGAIIIndividual(int D, int N) : x(D), f(N), rank(0), crowding_distance(0.0) {}
};

typedef std::vector<NSGAIIIndividual> ParetoFront;

std::ostream& operator<<(std::ostream& os, const NSGAIIIndividual& ind) {
    os << ind.x << ", " << ind.f << ", " << ind.rank << ", " << ind.crowding_distance;
    return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<NSGAIIIndividual>& pop) {
    for (const auto& ind : pop) {
        os << ind << std::endl;
    }
    return os;
}



// dominance comparison
bool dominates(const Objectives& p1, const Objectives& p2) {
    size_t n = 0;
    for (size_t i = 0; i < p1.size(); ++i) {
        if (p1[i] <= p2[i]) {
            n++;
        }
    }
    return n == p1.size();
}


ParetoFront NsgaII(Problem prob, NSGAIIParameters params) {
    if (params.verbosity == Verbosity::DEBUG) {
        std::cout << "NSGA-II parameters:" << std::endl;
        std::cout << "NP: " << params.NP << std::endl;
        std::cout << "Generations: " << params.Generations << std::endl;
    }

    // Initialize population
    ObjectiveFunction obj = prob.obj;
    std::vector<NSGAIIIndividual> population(params.NP);
    for (int i = 0; i < params.NP; ++i) {
        auto ind = Individual(prob.D);
        for (int j = 0; j < prob.D; ++j) {
            ind[j] = prob.lower_bound[j] + (prob.upper_bound[j] - prob.lower_bound[j]) * rand() / RAND_MAX;
        }
        population[i] = NSGAIIIndividual(prob.D, prob.N);
        population[i].x = ind;
        population[i].f = obj(ind);
    }

    // Main loop
    for (int gen = 0; gen < params.Generations; ++gen) {
        // Evaluate objectives
        for (auto& ind : population) {
            ind.f = obj(ind.x);
        }

        // Non-dominated sorting
        std::vector<std::vector<int>> fronts;
        std::vector<int> rank(population.size(), -1);
        std::vector<int> n(population.size(), 0);
        std::vector<std::vector<int>> S(population.size());
        for (int i = 0; i < population.size(); ++i) {
            for (int j = 0; j < population.size(); ++j) {
                if (i == j) {
                    continue;
                }
                if (dominates(population[j].f, population[i].f)) {
                    S[i].push_back(j);
                } else if (dominates(population[i].f, population[j].f)) {
                    n[i]++;
                }
            }
            if (n[i] == 0) {
                rank[i] = 0;
                // find empty front
                size_t k = 0;
                for (auto& front : fronts) {
                    if (front.empty()) {
                        break;
                    }
                    k++;
                } 
                if (k == fronts.size()) {
                    fronts.push_back(std::vector<int>());
                }
                fronts[0].push_back(i);
            }
        }
        int i = 0;
        while (!fronts[i].empty()) {
            std::vector<int> Q;
            for (int p : fronts[i]) {
                for (int q : S[p]) {
                    n[q]--;
                    if (n[q] == 0) {
                        rank[q] = i + 1;
                        Q.push_back(q);
                    }
                }
            }
            i++;
            fronts.push_back(Q);
        }
        
        // Crowding distance
        for (int i = 0; i < fronts.size(); ++i) {
            for (int j : fronts[i]) {
                population[j].crowding_distance = 0.0;
            }
            for (int m = 0; m < prob.D; ++m) {
                std::sort(std::begin(fronts[i]), std::end(fronts[i]), [m, &population](int a, int b) {
                    return population[a].f[m] < population[b].f[m];
                });
                population[fronts[i][0]].crowding_distance = std::numeric_limits<double>::infinity();
                population[fronts[i].back()].crowding_distance = std::numeric_limits<double>::infinity();
                for (int j = 1; j < fronts[i].size() - 1; ++j) {
                    population[fronts[i][j]].crowding_distance += (population[fronts[i][j + 1]].f[m] - population[fronts[i][j - 1]].f[m]) / (population[fronts[i].back()].f[m] - population[fronts[i][0]].f[m]);
                }
            }
        }

        // Select next generation
        std::sort(std::begin(population), std::end(population));
        std::vector<NSGAIIIndividual> next_population(params.NP);
        for (int i = 0; i < params.NP; ++i) {
            next_population[i] = population[i];
        }
        population = next_population;

        if (params.verbosity == Verbosity::DEBUG) {
            std::cout << "Generation " << gen << std::endl;
            std::cout << population << std::endl;
        }

    }

    // Return Pareto front
    ParetoFront pareto_front;
    for (const auto& ind : population) {
        if (ind.rank == 0) {
            pareto_front.push_back(ind);
        }
    }
    return pareto_front;
}

int main() {
    auto obj = [](Individual x) {
        Objectives f(2);
        f[0] = x[0];
        f[1] = (x[0] - 2) * (x[0] - 2);
        return f;
    };
    Problem prob(1, 2);
    prob.obj = obj;
    prob.lower_bound = -5.0;
    prob.upper_bound = 5.0;
    NSGAIIParameters params;
    params.NP = 100;
    params.Generations = 100;
    params.verbosity = Verbosity::DEBUG;
    auto pareto_front = NsgaII(prob, params);
    std::cout << "Pareto front:" << std::endl;
    std::cout << pareto_front << std::endl;
    return 0;
}