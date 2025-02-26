#ifndef PARAFOIL_HPP
#define PARAFOIL_HPP

#include <vector>
#include <functional>

namespace parafoil {

// State vector type definition
using state_vector = std::vector<double>;

// Control vector type definition
using control_vector = std::vector<double>;

// System dynamics function type
using dynamics_function = std::function<state_vector(const state_vector&, const control_vector&)>;

// Class for parafoil simulation and control
class ParafoilSystem {
public:
    ParafoilSystem();
    
    // Simulate parafoil dynamics
    state_vector simulate(const state_vector& initial_state, 
                         const control_vector& control_input,
                         double time_step,
                         double final_time);
    
    // Get system dynamics
    dynamics_function get_dynamics() const;
    
    // Set and get system parameters
    void set_parameters(const std::vector<double>& params);
    std::vector<double> get_parameters() const;

private:
    std::vector<double> parameters_;
    dynamics_function dynamics_;
};

} // namespace parafoil

#endif // PARAFOIL_HPP 