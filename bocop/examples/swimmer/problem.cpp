


// +++DRAFT+++ This class implements the OCP functions
// It derives from the generic class bocop3OCPBase
// OCP functions are defined with templates since they will be called
// from both the NLP solver (double arguments) and AD tool (ad_double arguments)
//#pragma once

#include <OCP.h>

// ///////////////////////////////////////////////////////////////////


template <typename Variable>
inline void OCP::finalCost(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable &final_cost)
{
    // maximal displacement along x-axis (with x(tf) < 0)
    final_cost = final_state[0];
}

template <typename Variable>
inline void OCP::dynamics(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *state_dynamics)
{
    //Purcell: central stick length 2; side sticks length 1
    //state: [x, y, th, b1, b3]
    //(x,y,th) position of the swimmer
    //alpha_i: angle between stick i and i+1
    //control: [b1' b3']
    //dynamics for (x,y,th): g1 u1 + g2 u2

    Variable th = state[2];
    Variable b1 = state[3];
    Variable b3 = state[4];
    Variable u1 = control[0];
    Variable u2 = control[1];

    // aux vars
    Variable aux = 543 + 186*cos(b1) + 37*cos(2*b1) + 12*cos(b1 - 2*b3) + 30*cos(b1 - b3) + 2*cos(2*(b1 - b3)) + 12*cos(2*b1 - b3) + 186*cos(b3) + 37*cos(2*b3) - 6*cos(b1 + b3) - 3*cos(2*(b1 + b3)) - 6*cos(2*b1 + b3) - 6*cos(b1 + 2*b3);


    //coefficients for dynamics
    Variable g11 = (-42*sin(b1 - th) - 2*sin(2*b1 - th) - 24*sin(th) - 300*sin(b1 + th) - 12*sin(2*b1 + th) - 6*sin(b1 - th - 2*b3) - sin(2*b1 - th - 2*b3) + 4*sin(th - 2*b3) - 12*sin(b1 + th - 2*b3) - sin(2*b1 + th - 2*b3) + 18*sin(b1 - th - b3) + 8*sin(th - b3) - 54*sin(b1 + th - b3) - 2*sin(2*b1 + th - b3) - 18*sin(b1 - th + b3) - 38*sin(th + b3) - 90*sin(b1 + th + b3) - 6*sin(b1 - th + 2*b3) - 18*sin(th + 2*b3) - 30*sin(b1 + th + 2*b3)) / (4.*aux);

    Variable g12 = (-42*cos(b1 - th) - 2*cos(2*b1 - th) + 24*cos(th) + 300*cos(b1 + th) + 12*cos(2*b1 + th) - 6*cos(b1 - th - 2*b3) - cos(2*b1 - th - 2*b3) - 4*cos(th - 2*b3) + 12*cos(b1 + th - 2*b3) + cos(2*b1 + th - 2*b3) + 18*cos(b1 - th - b3) - 8*cos(th - b3) + 54*cos(b1 + th - b3)+ 2*cos(2*b1 + th - b3) - 18*cos(b1 - th + b3) + 38*cos(th + b3) + 90*cos(b1 + th + b3) - 6*cos(b1 - th + 2*b3) + 18*cos(th + 2*b3) + 30*cos(b1 + th + 2*b3)) / (4.*aux);

    Variable g13 = -(105 + 186*cos(b1) + 2*cos(2*b1) + 12*cos(b1 - 2*b3) + 30*cos(b1 - b3) + cos(2*(b1 - b3)) - 4*cos(2*b3) - 6*cos(b1 + b3) - 6*cos(b1 + 2*b3)) / (2.*aux);

    Variable g21 = (8*sin(b1 - th) + 4*sin(2*b1 - th) + 24*sin(th) + 38*sin(b1 + th) + 18*sin(2*b1 + th) - 2*sin(b1 - th - 2*b3) - sin(2*b1 - th - 2*b3) - 2*sin(th - 2*b3) - sin(2*b1 + th - 2*b3) - 54*sin(b1 - th - b3) - 12*sin(2*b1 - th - b3) - 42*sin(th - b3) + 18*sin(b1 + th - b3) - 6*sin(2*b1 + th - b3) + 18*sin(b1 - th + b3) + 6*sin(2*b1 - th + b3) + 300*sin(th + b3) + 90*sin(b1 + th + b3) + 30*sin(2*b1 + th + b3) + 12*sin(th + 2*b3)) / (4.*aux);

    Variable g22 = (8*cos(b1 - th) + 4*cos(2*b1 - th) - 24*cos(th) - 38*cos(b1 + th) - 18*cos(2*b1 + th) - 2*cos(b1 - th - 2*b3) - cos(2*b1 - th - 2*b3) + 2*cos(th - 2*b3) + cos(2*b1 + th - 2*b3) - 54*cos(b1 - th - b3) - 12*cos(2*b1 - th - b3) + 42*cos(th - b3) - 18*cos(b1 + th - b3) + 6*cos(2*b1 + th - b3) + 18*cos(b1 - th + b3) + 6*cos(2*b1 - th + b3) - 300*cos(th + b3) - 90*cos(b1 + th + b3) - 30*cos(2*b1 + th + b3) - 12*cos(th + 2*b3)) / (4.*aux);

    Variable g23 = -(105 - 4*cos(2*b1) + 30*cos(b1 - b3) + cos(2*(b1 - b3)) + 12*cos(2*b1 - b3) + 186*cos(b3) + 2*cos(2*b3) - 6*cos(b1 + b3) - 6*cos(2*b1 + b3)) / (2.*aux);


    //dynamics: g1 u1 + g2 u2
    state_dynamics[0] = g11*u1 + g21*u2;
    state_dynamics[1] = g12*u1 + g22*u2;
    state_dynamics[2] = g13*u1 + g23*u2;
    state_dynamics[3] = u1;
    state_dynamics[4] = u2;

    // "energy" integral cost \int_0^T |u|^2 dt
    state_dynamics[5] = u1*u1 + u2*u2;

}

template <typename Variable>
inline void OCP::boundaryConditions(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable *boundary_conditions)
{
    // initial conditions: x0, y0, theta0, E0
    boundary_conditions[0] = initial_state[0];
    boundary_conditions[1] = initial_state[1];
    boundary_conditions[2] = initial_state[2];
    boundary_conditions[3] = initial_state[5];

    // final conditions: xf, yf, thetaf, Ef
    boundary_conditions[4] = final_state[0];
    boundary_conditions[5] = final_state[1];
    boundary_conditions[6] = final_state[2];
    boundary_conditions[7] = final_state[5];

    // periodicity conditions: theta, beta1, beta2
    boundary_conditions[8] = final_state[2] - initial_state[2];
    boundary_conditions[9] = final_state[3] - initial_state[3];
    boundary_conditions[10] = final_state[4] - initial_state[4];

    // beta1(0) (set sign to break symmetries)
    boundary_conditions[11] = initial_state[3];
}

template <typename Variable>
inline void OCP::pathConstraints(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *path_constraints)
{
}

void OCP::preProcessing()
{}

// ///////////////////////////////////////////////////////////////////
// explicit template instanciation for template functions, with double and double_ad 
// +++ could be in an included separate file ? 
// but needs to be done for aux functions too ? APPARENTLY NOT !
template void OCP::finalCost<double>(double initial_time, double final_time, const double *initial_state, const double *final_state, const double *parameters, const double *constants, double &final_cost);
template void OCP::dynamics<double>(double time, const double *state, const double *control, const double *parameters, const double *constants, double *state_dynamics);
template void OCP::boundaryConditions<double>(double initial_time, double final_time, const double *initial_state, const double *final_state, const double *parameters, const double *constants, double *boundary_conditions);
template void OCP::pathConstraints<double>(double time, const double *state, const double *control, const double *parameters, const double *constants, double *path_constraints);

template void OCP::finalCost<double_ad>(double initial_time, double final_time, const double_ad *initial_state, const double_ad *final_state, const double_ad *parameters, const double *constants, double_ad &final_cost);
template void OCP::dynamics<double_ad>(double time, const double_ad *state, const double_ad *control, const double_ad *parameters, const double *constants, double_ad *state_dynamics);
template void OCP::boundaryConditions<double_ad>(double initial_time, double final_time, const double_ad *initial_state, const double_ad *final_state, const double_ad *parameters, const double *constants, double_ad *boundary_conditions);
template void OCP::pathConstraints<double_ad>(double time, const double_ad *state, const double_ad *control, const double_ad *parameters, const double *constants, double_ad *path_constraints);
