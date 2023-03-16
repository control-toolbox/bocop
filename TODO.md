* Features

** feature/warmstart
- [high] implement basic discrete continuation in ad hoc python script

** feature/def
- [medium] pass def file name in python script
- [medium] same for callback solve()
- [medium] use default values for ode.discretization and the 4 usual ipopt parameters
- [medium] add default constant initialization (0.1) for all variables

** feature/runningcost
- [low] automate Lagrange cost with a key: use.running.cost true
Add a function 'running_cost', add the integral as last state and in objective
- [low] adjust visualisation in GUI/python to hide it

** feature/free_final_time
- [low] automate with a key: free.final.time true, add parameter and dynamics rescaling.
Set final time as last optimisation variable (to preserve the indices of user-defined optimisation variables)
Problem: find a way to keep variable 'time' as a double in the function calls ? 
Maybe provide a function getActualTime(time) in OCP class (to be callable from problem functions) ?

** feature/examples
- [high] add from bocop2 examples
- [high] add from personal collection of problems
- [high] add corresponding notebooks in ct-gallery: display problem.cpp and problem.def a the end of the notebook !

** feature/auto_diff
- [medium] compute Hessian wrt primal variables only (not multipliers). Use cppad DYNAMIC property for lambda.
- [low] test extension codegen for cppad in order to generate derivatives of OCP functions
- [low] use another tool with code generation instead of operator overloading (adic2, tapenade ?)

** feature/interpolation
- [medium] interpolation tools: classes for linear interpolation1D and interpolation2D. init(data...,) and interpolate((x[,y])
- [low] C2 splines

** feature/sol
- [medium] for .sol file use prefix from .def file
- [low] use proper functions in callback instead of manual split of X. See dOCPSolution
- [low] reintegrate solution.h/cpp in dOCP. Is the solution class really needed ? Check what the callback uses.

** feature/param_Id
- [high] measures for parameter Id. Use state/control interpolation instead of adding measure times to discretization.
Probably needs a specific function that is added to the final_cost. This function requires access to the whole unknow.
Cf pass X to all OCP functions ? or maybe better to have functions getStateAtTime etc (interpolations). see also delay problems

** feature/events
- [low] sequence of fixed/free times at which specific operations are performed. Base case: [t0,tf]. State jumps, phase change

** direct transcription method
- [low] control discretisation choices: stages, steps, parametrized (CVP)
- [low] state discretisation: simultaneous / sequential (ie state recomputed by integration)
- [low] path constraints: enforced at steps or stages ? Stage discretization seems to give more oscillations 
Note: move functions getStateAtStep etc ... to classes dState, dControl, dODE

** misc
- [medium] display ipopt iterations within GUI / notebook: try to use runCommand() instead of just calling bocopwrapper.solve()
- [medium] for delay problems and other: give access to the whole NLP unknown to the OCP functions, via getState/Control/Parameters.
These functions are in dOCP but need to be accessible from OCP, check the classes hierarchy (friend ?). Get access to [time] step too.
- [medium] python script to import a problem from bocop2 to bocop3 format ( .def file and .cpp file)
- [medium] def: new key <constraint>.equal value that counts as both lower and upper bound
- [medium] pass generic options to bocop.test(), .def options (discretization, steps, tol) and/or cmake options (eg. test debug, compiler)
- [low] pass prefix to function newProblem() (default='problem')?
- [low] bound blocks: ehhh, probaly easier to just use python script to generate .def file...

** gallery
- [high] use named args in all python calls

** python module
- [medium] batch optimisations and/or continuation with ad hoc scripts
- [medium] pass the name of the .def file to the solve/run functions
- [low] pass options (as dictionary) to solve/run functions, that override the .def file
- [low] parallel runs with different random initialisations, then collate solutions
- [low] HJB2NLP tool that reads a HJB trajectory, generates .init files for primal variables 
and use objective value to set a constraint J_NLP <= (1+epsilon) J_HJB
- [medium] OCP integration in nutopy: reuse c++ functions. Try  https://github.com/D3f0/ipython_gcc
Concerning templates, only wrap the double version with swig ? Note that functions are the same for all problems, so wrappings can be reused.

** matlab interface
- [low] restricted to the bare minimum for now (ie readsolution).
- [low] check matlab ability to execute python code https://www.mathworks.com/help/matlab/call-python-libraries.html
