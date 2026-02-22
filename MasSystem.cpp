#include "MasSystem.h"
#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

MasSystem::MasSystem() {
    Tmati = 0.01;
    
    L << 1, -1,  0,  0,
         0,  0,  0,  0,
         0, -1,  1,  0,
        -1,  0, -1,  2;

    B = Eigen::Matrix4d::Zero();
    B(1,1) = 1; // only agent-2 receives info directly from leader
    
    A_param = 0.2;
    w_param = 0.4;

    K11 << 2, 2;
    K12 << 85, 85;
    K21 << 2, 2;
    K22 << 85, 85;
    K31 << 2, 2;
    K32 << 80, 80;
    K41 << 2, 2;
    K42 << 70, 30;

    last_update_time = -1.0; 
    k_step = 0;
    x01 << A_param, 0.0; 
    
    // Initial holds to zero, will be properly populated at t=0
    x11_hold.setZero(); x21_hold.setZero(); x31_hold.setZero(); x41_hold.setZero();
}

Eigen::Vector2d MasSystem::get_disturbance(double t) const {
    Eigen::Vector2d z;
    z(0) = 0.3 * std::cos(3*M_PI*t) * std::cos(2*M_PI*t) * 0.2 * std::sin(1.5*M_PI*t) 
           - 0.1 * std::sin(M_PI*t) * 0.4 * std::cos(2*M_PI*t);
    z(1) = 0.5 * std::cos(3*M_PI*t) * 0.3 * std::sin(2*M_PI*t) 
           - 0.3 * std::sin(1.5*M_PI*t);
    return z;
}

Eigen::Vector2d MasSystem::T_func(const Eigen::Vector2d& x) const {
    // T(x) = (0.5*pi) * sec(pi*x/2)^2
    Eigen::Vector2d res;
    for(int i=0; i<2; ++i) {
        double inner = M_PI * x(i) / 2.0;
        double secant_sq = 1.0 / std::pow(std::cos(inner), 2);
        res(i) = (0.5 * M_PI) * secant_sq;
    }
    return res;
}

double MasSystem::p_bound_1(double t) const {
    // (0.8 - 1e-2) * exp(-t) + 1e-2
    return (0.8 - 0.01) * std::exp(-t) + 0.01;
}

double MasSystem::p_bound_2(double t) const {
    // (3 - 0.8) * exp(-t) + 0.8
    return (3.0 - 0.8) * std::exp(-t) + 0.8;
}

// Helper math: element-wise tan(x * pi/2)
Eigen::Vector2d compute_tan(const Eigen::Vector2d& error) {
    Eigen::Vector2d res;
    res(0) = std::tan(error(0) * (M_PI / 2.0));
    res(1) = std::tan(error(1) * (M_PI / 2.0));
    return res;
}

void MasSystem::operator()(const state_type &x, state_type &dxdt, const double t) {
    // 1. Extract continuous states
    Eigen::Vector2d x11(x[0], x[1]);
    Eigen::Vector2d x12(x[2], x[3]);
    Eigen::Vector2d x21(x[4], x[5]);
    Eigen::Vector2d x22(x[6], x[7]);
    Eigen::Vector2d x31(x[8], x[9]);
    Eigen::Vector2d x32(x[10], x[11]);
    Eigen::Vector2d x41(x[12], x[13]);
    Eigen::Vector2d x42(x[14], x[15]);

    // 2. Discrete-Time Updates (Communication Graph)
    if (t == 0.0 && last_update_time < 0) {
        x11_hold = x11;
        x21_hold = x21;
        x31_hold = x31;
        x41_hold = x41;
        last_update_time = 0.0;
        k_step = 0;
        // Leader initialization
        x01(0) = A_param * std::cos(w_param * M_PI * t);
        x01(1) = A_param * std::sin(w_param * M_PI * t);
    }
    
    // "t >= last_update_time + Tmati" 
    // We add a tiny epsilon to prevent double-firings from floating point issues
    if (t >= (last_update_time + Tmati - 1e-12)) {
        x11_hold = x11;
        x21_hold = x21;
        x31_hold = x31;
        x41_hold = x41;
        k_step++;
        last_update_time = k_step * Tmati;
        x01(0) = A_param * std::cos(w_param * M_PI * t);
        x01(1) = A_param * std::sin(w_param * M_PI * t);
    }

    // Disturbance
    Eigen::Vector2d z = get_disturbance(t);

    // Bounds at current time t
    double p11_v = p_bound_1(t); 
    Eigen::Vector2d p11 = Eigen::Vector2d::Constant(p11_v);
    
    double p12_v = p_bound_2(t);
    Eigen::Vector2d p12 = Eigen::Vector2d::Constant(p12_v);

    // All agents use same functional forms for bounds
    Eigen::Vector2d p21 = p11, p31 = p11, p41 = p11;
    Eigen::Vector2d p22 = p12, p32 = p12, p42 = p12;

    Eigen::Vector2d error, psy;

    // ----- AGENT 1 -----
    Eigen::Vector2d e11 = x11 - x21_hold;
    error = e11.cwiseQuotient(p11);
    Eigen::Vector2d psy_11 = -K11.cwiseProduct(compute_tan(error)); //  -K11 .* tan(error * pi/2)
    
    error = (x12 - psy_11).cwiseQuotient(p12);
    Eigen::Vector2d psy_12 = -K12.cwiseProduct(compute_tan(error)).cwiseQuotient(p12).cwiseProduct(T_func(error));
    Eigen::Vector2d u1 = psy_12;

    Eigen::Matrix2d G1;
    G1 << std::pow(x11(0), 2) + 1, std::cos(x11(1)),
          std::sin(x12(0)), std::pow(x12(1), 2) + 4;
    
    Eigen::Vector2d f11(std::sin(x11(0)), std::cos(x11(1)));
    Eigen::Vector2d f12(-2.0*x12(0) - x12(1), std::pow(x11(0), 2));

    Eigen::Vector2d dx11 = x12 + f11;
    Eigen::Vector2d dx12 = f12 + G1 * u1 + z;

    // ----- AGENT 2 -----
    Eigen::Vector2d e21 = x21 - x01; // B(2,2)=1 makes error x21 - x01
    error = e21.cwiseQuotient(p21);
    Eigen::Vector2d psy_21 = -K21.cwiseProduct(compute_tan(error));
    
    error = (x22 - psy_21).cwiseQuotient(p22);
    Eigen::Vector2d psy_22 = -K22.cwiseProduct(compute_tan(error)).cwiseQuotient(p22).cwiseProduct(T_func(error));
    Eigen::Vector2d u2 = psy_22;

    Eigen::Matrix2d G2;
    G2 << std::pow(x21(0), 2) + 1, std::sin(x21(1)),
          std::cos(x22(1)), std::pow(x22(1), 2) + 1;
    
    Eigen::Vector2d f21(2.0*x21(0), std::pow(x21(1), 3));
    Eigen::Vector2d f22(std::pow(x21(0), 2), std::pow(x22(1), 2) - std::pow(x21(0), 5));

    Eigen::Vector2d dx21 = x22 + f21;
    Eigen::Vector2d dx22 = f22 + G2 * u2 + z;

    // ----- AGENT 3 -----
    Eigen::Vector2d e31 = x31 - x21_hold;
    error = e31.cwiseQuotient(p31);
    Eigen::Vector2d psy_31 = -K31.cwiseProduct(compute_tan(error));
    
    error = (x32 - psy_31).cwiseQuotient(p32);
    Eigen::Vector2d psy_32 = -K32.cwiseProduct(compute_tan(error)).cwiseQuotient(p32).cwiseProduct(T_func(error));
    Eigen::Vector2d u3 = psy_32;

    Eigen::Matrix2d G3;
    G3 << std::pow(x31(0), 2) + 1, std::cos(x31(0)),
          std::sin(x31(1)), std::pow(x31(0), 2) + 3;
    
    Eigen::Vector2d f31(std::pow(std::cos(x31(0)), 2), std::pow(x31(0), 2));
    Eigen::Vector2d f32(std::sin(x32(1)) + x32(0), 5.0*std::pow(x32(0), 3));

    Eigen::Vector2d dx31 = x32 + f31;
    Eigen::Vector2d dx32 = f32 + G3 * u3 + z;

    // ----- AGENT 4 -----
    // e41 = (x41 - x11_hold) + (x41 - x31_hold)
    Eigen::Vector2d e41 = (x41 - x11_hold) + (x41 - x31_hold);
    error = e41.cwiseQuotient(p41);
    Eigen::Vector2d psy_41 = -K41.cwiseProduct(compute_tan(error));
    
    error = (x42 - psy_41).cwiseQuotient(p42);
    Eigen::Vector2d psy_42 = -K42.cwiseProduct(compute_tan(error)).cwiseQuotient(p42).cwiseProduct(T_func(error));
    Eigen::Vector2d u4 = psy_42;

    Eigen::Matrix2d G4;
    G4 << std::pow(x41(0), 2) + 2, std::cos(x41(0)),
          std::sin(x41(1)), std::pow(x41(0), 2) + 3;
    
    Eigen::Vector2d f41(std::pow(std::sin(x41(0)), 2), std::pow(x41(0), 2));
    Eigen::Vector2d f42(std::cos(x42(1)) + x42(0), 5.0*std::pow(x42(0), 2));

    Eigen::Vector2d dx41 = x42 + f41;
    Eigen::Vector2d dx42 = f42 + G4 * u4 + z;


    // 3. Populate dvdt (dxdt vector)
    dxdt[0] = dx11(0); dxdt[1] = dx11(1);
    dxdt[2] = dx12(0); dxdt[3] = dx12(1);
    
    dxdt[4] = dx21(0); dxdt[5] = dx21(1);
    dxdt[6] = dx22(0); dxdt[7] = dx22(1);
    
    dxdt[8] = dx31(0); dxdt[9] = dx31(1);
    dxdt[10] = dx32(0); dxdt[11] = dx32(1);
    
    dxdt[12] = dx41(0); dxdt[13] = dx41(1);
    dxdt[14] = dx42(0); dxdt[15] = dx42(1);
}