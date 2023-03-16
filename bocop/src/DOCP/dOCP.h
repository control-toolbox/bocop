// dOCP.h
//

#pragma once

#include <NLP.h>

#include <tools.h>
#include <OCP.h>
#include <dODE.h>
#include <dState.h>
#include <dControl.h>

#include <solution.h>

#include <vector>

template <typename T>
using view_t = bcp::buffer_adaptor<T>;


/** ***************************************************************************
* \class dOCP
* \brief dOCP defines a discretized optimal control problem
*
* It derives from class NLP and compounds class OCP.
* The discretized problem is built from the original control problem by direct transcription,
* with the state dynamics discretised by a generalized Runge Kutta formula (defined with Butcher tables)
* **NB.currently hardcoded implicit midpoint...**
* The objective and constraints functions for the NLP solver are defined in dOCP_CppAD, with their derivatives.
*
* Direct transcription layout for NLP:
* - general layout of variables X: { [STATE] [CONTROL] [PARAMETERS] [ODE_INTERNAL] }\n
* more precisely X: {[state ... state] [(controlstage ... controlstage) ...(controlstage ... controlstage)] [parameters] [(kstage ... kstage) ... (kstage ... kstage)] }
* - general layout of constraints C: { [BOUNDARY CONDITIONS] [DISCRETIZED ODE] [PATH CONSTRAINTS] }\n
* more precisely C: {boundarycond [dynstep (dynstage...dynstage) pathcond] ... [dynstep (dynstage...dynstage) pathcond] }
*
* *Note: ideally, transcription should be as modular as possible to allow different formulations*
* ****************************************************************************/
class dOCP : public NLP
{
public:
     dOCP(void);
    ~dOCP(void);

    ////////////////////////////////////////////////////////////////////
    // overrides for NLP
    void initialize(void) override;

    // NLP solution
    std::string solutionFile(void) const override {return solution_file;}
    void setSolutionFile(const std::string solfile) override {solution_file = solfile;}
    void writeSolution(const int status, const int iter, const double objective, const double constraints_viol, const double *variables, const double *multipliers, const double *constraints) override;

    // NLP variables and bounds
    std::size_t variablesSize(void) const override {return variables_size;}
    std::size_t constraintsSize(void) const override {return constraints_size;}
    std::vector<double> startingPoint(void) const override {return starting_point;}
    std::vector<double> variablesLowerBounds(void) const override {return variables_lower_bounds;}
    std::vector<double> variablesUpperBounds(void) const override {return variables_upper_bounds;}
    std::vector<double> constraintsLowerBounds(void) const override {return constraints_lower_bounds;}
    std::vector<double> constraintsUpperBounds(void) const override {return constraints_upper_bounds;}

    /** OCP member */
    OCP *ocp;
    /** \name direct transcription parts */
    /**@{*/
    dODE *rk;
    dState *xd;
    dControl *ud;
    /**@}*/
    void setOCP(OCP *ocp){ this->ocp = ocp; }

    // time discretisation
    std::string RKmethod;
    std::size_t discretisation_steps;
    double time_step;
    std::vector<double> time_step_grid;
    std::vector<double> time_stage_grid;

    // starting point
    std::vector<double> starting_point;

    // solution
    std::string solution_file;

public:

    std::size_t discretisationSteps() {return discretisation_steps;}
    std::size_t RKStages() {return rk->RKStages();}
    double timeStep() {return time_step;}
    double timeAtStep(std::size_t step) const {return time_step_grid[step];}
    double timeAtStage(std::size_t step, std::size_t stage) const {return time_stage_grid[step * rk->RKStages() + stage];}

    /** \name Optimal Control Problem specific functions
    * note: later this class could be abstracted and specified according to the transcription method used ?
    * eg. full disc, direct collocation, CVP, ...
    * */
    /**@{*/
    template <typename C> view_t<typename C::value_type> stateAtStep(const C&, std::size_t);
    template <typename C> view_t<typename C::value_type> controlAtStage(const C&, std::size_t, std::size_t);
    template <typename C> view_t<typename C::value_type> getParameters(const C&);
    template <typename C> typename C::value_type kComponent(const C& , std::size_t, std::size_t, std::size_t);
    template <typename C> std::vector<typename C::value_type> stateAtStage(const C&, std::size_t, std::size_t);
    template <typename C> std::vector<typename C::value_type> controlAtStep(const C&, std::size_t);
    /**@}*/

    // +++todo: writeSolution(X) takes the solution X from NLP solver, parses it in terms of state, control, params, and saves it
    // to be called after the solve() of NLP (more generic than using finalize_solution from ipopt)


public:

    // dimensions
    std::size_t variables_size;
    std::size_t constraints_size;
    std::size_t variables_offset_state;
    std::size_t variables_offset_control;
    std::size_t variables_offset_param;

    // bounds
    std::vector<double> variables_lower_bounds;
    std::vector<double> variables_upper_bounds;
    std::vector<double> constraints_lower_bounds;
    std::vector<double> constraints_upper_bounds;


};

// ///////////////////////////////////////////////////////////////////


// +++ put this in dState
template <typename C>
inline auto dOCP::stateAtStep(const C& v, std::size_t step) -> view_t<typename C::value_type>
{
    using value_t = typename C::value_type;

    auto size = ocp->stateSize();
    size_t start = step * size;

    return view_t<value_t>(size, v.data() + start);
}

// +++ put this in dControl
template <typename C>
inline auto dOCP::controlAtStage(const C& v, std::size_t step, std::size_t stage) -> view_t<typename C::value_type>
{
    auto state_size = ocp->stateSize();
    auto control_size = ocp->controlSize();
    auto offset = (discretisation_steps + 1) * state_size;
    auto start = offset + (step * RKStages() + stage) * control_size;

    return view_t<typename C::value_type>(control_size, v.data() + start);
}

// +++ put this in dODE
template <typename C>
inline auto dOCP::getParameters(const C& v) -> view_t<typename C::value_type>
{
    auto state_size = ocp->stateSize();
    auto control_size = ocp->controlSize();
    auto param_size = ocp->parametersSize();
    auto start = (discretisation_steps + 1) * state_size
               + discretisation_steps * RKStages() * control_size;

    return view_t<typename C::value_type>(param_size, v.data() + start);
}

// +++ put this in dODE
template <typename C>
inline auto dOCP::kComponent(const C& v, std::size_t step, std::size_t stage, std::size_t index) -> typename C::value_type
{
    auto state_size = ocp->stateSize();
    auto control_size = ocp->controlSize();
    auto param_size = ocp->parametersSize();
    auto offset = (discretisation_steps + 1) * state_size + discretisation_steps * RKStages() * control_size + param_size;
    auto start = offset + (step * RKStages() + stage) * state_size;

    return v[start+index];
}

// +++ put this in dState
template <typename C>
inline auto dOCP::stateAtStage(const C& v, std::size_t step, std::size_t stage) -> std::vector<typename C::value_type>
{
    using value_t = typename C::value_type;

    double h = time_step;
    auto state = stateAtStep(v, step);
    std::vector<value_t> stage_state(state.begin(), state.end());

    for (std::size_t n = 0;  n < ocp->stateSize(); ++n) {
        value_t sum_ak_n = 0.0;
        for (std::size_t i = 0;  i < RKStages(); ++i) {
            sum_ak_n += rk->butcherA()[stage][i] * kComponent(v, step, i, n);
        }
        stage_state[n] += h * sum_ak_n;
    }
    return stage_state;
}

/**
* **+++ put this in dControl**
*/
template <typename C>
inline auto dOCP::controlAtStep(const C& v, std::size_t step) -> std::vector<typename C::value_type>
{
    using value_t = typename C::value_type;

    auto control_size = ocp->controlSize();

    std::vector<value_t> step_control(control_size, 0.0);
    for (std::size_t j = 0; j < RKStages(); ++j) {
        auto u_j = controlAtStage(v, step, j);
        for (std::size_t i = 0; i < control_size; ++i) {
            step_control[i] += rk->butcherB()[j] * u_j[i];
        }
    }
    return step_control;
}

//
// dOCP.h ends here
