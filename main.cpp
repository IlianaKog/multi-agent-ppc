#include <iostream>
#include <fstream>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include "MasSystem.h"

using namespace boost::numeric::odeint;

// We will use rosenbrock4 since this is a stiff system. 
// Note: rosenbrock4 requires the Jacobian for full implicit speed. 
// Since we don't have an analytical Jacobian, we'll start with runge_kutta_dopri5
// which is explicit but adaptive step size, similar to ode45. 
// If it's too stiff, we can switch solvers, but dopri5 is easier without manually deriving the 16x16 Jacobian.
typedef runge_kutta_dopri5<state_type> stepper_type;

struct Observer {
    std::ofstream& m_out;
    Observer(std::ofstream& out) : m_out(out) {}

    void operator()(const state_type &x, double t) {
        m_out << t;
        for (size_t i = 0; i < x.size(); ++i) {
            m_out << "," << x[i];
        }
        m_out << "\n";
    }
};

int main() {
    std::cout << "Initializing Multi-Agent PPC Simulation..." << std::endl;

    MasSystem system;
    
    // Initial Conditions (matching init_v from MATLAB)
    // agents_12_init = [-0.1;-0.2; 0.15;0.1; 0;0; 0.1;-0.3];
    // agents_12_init = [-0.1;-0.2; 0;0; 0.15;0.1; 0;0];
    // agent_3 = [0;0;0;0;];
    // agent_4 = [0.1;-0.3; 0;0;];
    state_type x(16);
    x[0] = -0.1; x[1] = -0.2; // x11
    x[2] = 0.0;  x[3] = 0.0;  // x12
    x[4] = 0.15; x[5] = 0.1;  // x21
    x[6] = 0.0;  x[7] = 0.0;  // x22
    
    x[8] = 0.0;  x[9] = 0.0;  // x31
    x[10]= 0.0;  x[11]= 0.0;  // x32
    
    x[12]= 0.1;  x[13]= -0.3; // x41
    x[14]= 0.0;  x[15]= 0.0;  // x42

    double t_start = 0.0;
    double t_end = 30.0;
    double dt = 0.01; // Initial step size

    std::ofstream out_file("simulation_results.csv");
    out_file << "t,x11_1,x11_2,x12_1,x12_2,x21_1,x21_2,x22_1,x22_2,x31_1,x31_2,x32_1,x32_2,x41_1,x41_2,x42_1,x42_2\n";

    std::cout << "Starting Integration Loop..." << std::endl;

    // We use integrate_adaptive to let the solver choose steps to satisfy tolerances (resembling RelTol/AbsTol)
    auto stepper = make_controlled(1e-10, 1e-11, stepper_type());
    
    size_t steps = integrate_adaptive(stepper, system, x, t_start, t_end, dt, Observer(out_file));

    std::cout << "Integration finished after " << steps << " steps.\n";
    std::cout << "Results saved to simulation_results.csv.\n";

    return 0;
}
