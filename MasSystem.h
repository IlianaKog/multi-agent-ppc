#pragma once

#include <Eigen/Dense>
#include <vector>

// Define the state type for Boost.Odeint. 
// The system has 16 ODE states (8 agents * 2 dimensions).
typedef Eigen::Matrix<double, 16, 1> state_type;

class MasSystem {
public:
    // Parameters
    double Tmati;
    
    // Communication graph matrices (4x4)
    Eigen::Matrix4d L;
    Eigen::Matrix4d B;

    // Control parameters (from MATLAB)
    Eigen::Vector2d K11, K12, K21, K22, K31, K32, K41, K42;
    double A_param;
    double w_param;

    // Discrete-time memory (persistent in MATLAB)
    Eigen::Vector2d x11_hold, x21_hold, x31_hold, x41_hold;
    double last_update_time;
    int k_step;
    Eigen::Vector2d x01; // Leader state position

    // Disturbance utility
    Eigen::Vector2d get_disturbance(double t) const;
    
    // Performance bounds T(x)
    Eigen::Vector2d T_func(const Eigen::Vector2d& x) const;

    // Constructor to initialize parameters
    MasSystem();

    // The ODE evaluation operator
    // This is called by Boost.Odeint at each time step
    void operator()(const state_type &x, state_type &dxdt, const double t);

private:
    // Helper function for performance bounds p(t)
    double p_bound_1(double t) const;
    double p_bound_2(double t) const;
};
