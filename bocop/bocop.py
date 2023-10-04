"""
   bocop module description
"""

# Bocop python utils for bocop3
# Pierre Martinon
# Inria
# 2020


# High level interface
# - solve: solve given problem, ie build, run and return solution

# Build and run
# - build: build bocop executable for given problem
# - run: run bocop executable for given problem

# Setters
# - setInDef: set a key in problem definition file .def

# Getters
# - readDefFile: returns options from a .def file (dictionary) 
# - readSolFile: load a .sol file into a dOCPSolution
# -- getValue: read a single value with provided header
# -- getBlock1D: read a dim 1 vector with provided header
# -- getBlock2D: read a dim 2 matrix in file with provided header

# Classes
# - dOCPDefinition: problem definition, corresponding to .def file
# -- load: load .def file
# - dOCPSolution: problem solution, corresponding to .sol file
# -- plot: plot solution


# REALTIME OUTPUT TESTS:
# Notes: use of flush in print commands is not compatible with PySimpleGUIQt :(
# CONTEXT       COMMAND                         VERBOSE=1   VERBOSE=0
# LINUX
# python shell     runCommand(cmake/make)       OK             OK
#                   bocopwrapper.solve()              OK             same as verbose=1
# notebook        runCommand(cmake/make)       OK             OK
#                   bocopwrapper.solve()              in shell         same
# bocopGUI       runCommand(cmake/make)       OK*              OK
#                   bocopwrapper.solve()              in shell
#                   runCommand(bocopwrapper.solve()) OK ?
# * option -j seems to freeze the buildling output ?

import matplotlib.pyplot as plt
import numpy as np
import os
import platform
import shutil
import subprocess
import sys
import shlex
import time

bocop_root_path = os.path.dirname(__file__)

# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
# +++ optional: problem prefix (def/sol)
def newProblem(new_problem_path):

    # create folder if missing
    if not os.path.exists(new_problem_path):
        os.makedirs(new_problem_path)

    # copy default files (copytree before 3.8 cannot handle existing folders)
    default_problem_path = bocop_root_path + '/examples/default'
    shutil.copy(default_problem_path+'/problem.def', new_problem_path)
    shutil.copy(default_problem_path+'/problem.cpp', new_problem_path)
    shutil.copy(default_problem_path+'/problem.ipynb', new_problem_path)
    shutil.copy(default_problem_path+'/build.sh', new_problem_path)
    shutil.copy(default_problem_path+'/build.bat', new_problem_path)


# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
# TODO: pass bocop_options as dictionary ? 
def solve(problem_path = '.', prefix = 'problem', verbose = 1, clean = 1, debug = 0, graph = 1, separateProcess = 0, warmstart = None, cmake_options = ''):
    """
    Solve an OCP via direct trancription method

    The solver builds a discretized approximation of the original OCP problem, and solves the resulting NLP problem.
    This method currently wraps up the build and run process for the C++ bocop code.
    Results are returned a :class: dOCPSolution object.

    Parameters
    ----------

    problem_path: string
        Location of the problem definition (cpp and def files)

    clean: int
        Passed to build and run methods: clean=1 will delete any anterior existing otuput files, to guarantee a fresh run

    debug: int
        Passed to build method: debug=1 will build in debug mode

    graph: int
        Sets the level of graphs: 0: no graphs, 1: state and control, 2: state, control and costate

    verbose: int
        Sets the verbosity level: 0: minimal to 2: maximal ouptut

    """

    problem_path = os.path.abspath(problem_path)

    # build executable (will use bocop path)
    if verbose >= 0:
        print('build...',end='',flush=True)
    status = build(problem_path=problem_path, verbose=verbose, clean=clean, debug=debug, cmake_options=cmake_options)
    if verbose >= 0:
        print('\b\b\b\b\b\b\b\b',end='',flush=True)
    
    if status != 0:
        print("Build step failed with status",status)
        return dOCPSolution()

    # launch executable
    if verbose >= 0:
        print('run...  ',end='',flush=True)    
    run(problem_path=problem_path, prefix=prefix, verbose=verbose, clean=clean, debug=debug, graph=graph, separateProcess=separateProcess, warmstart=warmstart)
    if verbose >= 0:
        print('\b\b\b\b\b\b\b\b',end='',flush=True)
        
    # read solution
    solution = readSolution(problem_path + "/" + prefix + ".sol")

    # plot solution
    if graph > 0:
        solution.plot(graph)

    # return solution for further use
    return solution


# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
def build(problem_path = '.', verbose = 1, clean = 1, debug = 0, window = None, cmake_options = ''):

    # debug is currently not available on windows...
    if (platform.system() == 'Windows') and debug == 1:
        print("Warning: Python on Windows is not shipping debug libs, switching to Release build.")
        debug = 0

    status = 0
    problem_path = os.path.abspath(problem_path)
    fnull = open(os.devnull, 'w')
    origin_dir = os.getcwd()
    os.chdir(problem_path)

    # set exec name according to platform
    if (platform.system() == 'Windows'):
        exec_name = 'bocopApp.exe'
    else:
        exec_name = 'bocopApp'

    # clean old files if required (+++ if possible use commands that dont need to test existence first)
    if clean == 1:
        if os.path.exists('build'):
            shutil.rmtree('build')
        if os.path.exists(exec_name):
            os.remove(exec_name)

    # debug option
    if debug == 1:
        buildtype="Debug"
    else:
        buildtype="Release"

    # build folder
    if not os.path.exists('build'):
        os.mkdir('build')
    os.chdir('build')

    ## construct cmake/make build commands
    if (platform.system() != 'Windows'):

        # LINUX / MACOS
        # construct cmake command
        cmake_command = [ f'cmake -DCMAKE_BUILD_TYPE={buildtype} -DCOVERAGE=False -DEXEC=False -DWRAPPER=True -DPROBLEM_DIR={problem_path} {cmake_options} {bocop_root_path}' ]

        # construct make command
        make_command = ["cmake --build . -j"]

    else:
        # WINDOWS
        # construct cmake command (NB. copied from Visual Studio CmakeSettings.json)
        # +++ this unortunately ties the build to the specific VS16 ... try to make this more generic !
        cmake_configuration = {
            "name": "x64-RelWithDebInfo",
            "generator": "Visual Studio 16 2019",
            "configurationType": "Release",     # does not seem to work
            "buildRoot": "${projectDir}/out/build/${name}",
            "installRoot": "%CONDA_PREFIX%/Library/",
            "cmakeCommandArgs": "",   # use this instead of variables below ?
            "buildCommandArgs": "",   # use this instead of variables below ?
            "ctestCommandArgs": "",
            "inheritEnvironments": [ "msvc_x64_x64" ],
            "variables": [
                { "name": "CMAKE_PREFIX_PATH", "value": "%CONDA_PREFIX%/Library/", },
                { "name": "CMAKE_INSTALL_RPATH", "value": "%CONDA_PREFIX%/Library/lib/", },
                { "name": "CMAKE_INSTALL_NAME_DIR", "value": "%CONDA_PREFIX%/Library/lib/", },
                { "name": "PROBLEM_DIR", "value": problem_path },
                { "name": "EXEC", "value": "False" },
                { "name": "WRAPPER", "value": "True" },
                { "name": "CMAKE_BUILD_TYPE", "value": "Release" },
                { "name": "COVERAGE", "value": "False" }                              
            ]
        }
        cmake_command = [ "cmake" ]
        cmake_command.append("-G")
        cmake_command.append(cmake_configuration["generator"])
        for v in cmake_configuration["variables"]:
            cmake_command.append("-D" + v["name"] + "=" + v["value"])
        cmake_command.append(bocop_root_path)

        # construct make command
        make_command = [ "cmake", "--build", os.path.join(problem_path, "build"), "--config", cmake_configuration["configurationType"] ]

    # execute cmake command
    status = runCommand(command=cmake_command, verbose=verbose, window=window)
    if status != 0:
        print(f"Error: build step CMAKE failed with return {status}")
        os.chdir(origin_dir)
        return status

    # execute make command
    status = runCommand(command=make_command, verbose=verbose, window=window)
    if status != 0:
        print(f"Error: build step MAKE failed with return {status}")
        os.chdir(origin_dir)
        return status

    # go back to original path and return status
    os.chdir(origin_dir)
    return status


# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
# TODO: pass options as dictionary ?
def run(problem_path = '.', prefix = 'problem', verbose = 1, clean = 1, debug = 1, graph = 0, window = None, separateProcess = 0, warmstart = None):

    # init
    fnull = open(os.devnull, 'w')
    starting_path = os.getcwd()

    # go to problem location
    problem_path = os.path.abspath(problem_path)
    os.chdir(problem_path)
    sys.path.append(problem_path)
    
    # set .def and sol. files names (for non callback runs)
    def_file = prefix + ".def"
    sol_file = prefix + ".sol"

    # set .def and .init for warmstart
    if warmstart is not None:
        setWarmstart(warmstart)

    # clean previous output files +++ single command if possible
    if clean == 1:
        if os.path.exists("result.out"):
            os.remove("result.out")
        if os.path.exists(sol_file):
            os.remove(sol_file)

    # need the debug mode to retrieve files from subfolder Debug/Release ?
    # weird, this should be done in the build instead !
    if (platform.system() == 'Windows'):
        s = ("Release", "Debug")[debug]
        command_copy = ["copy", "/y", f"{s}\\*", "."]
        runCommand(command_copy, verbose=verbose, window=window)

    # when running tests we have to run in separate processes (nb. check this)
    command = f"python -c \"import bocopwrapper; bocopwrapper.solve(None, \'{def_file}\', \'{sol_file}\')\""
    if separateProcess > 0 or graph <= 0:
        # execute with basic iteration text display (use runCommand for display in GUI/notebook instead of shell)
        runCommand(command, verbose=verbose, window=window)
    else:
        # execute with iteration graphical display via callback
        import bocopwrapper
        from IPython.display import clear_output
        from ipywidgets import interact, IntSlider
        callback = bocopwrapper.PyCallback()
        bocopwrapper.solve(callback)
        clear_output()
        interact(callback.update, iteration=IntSlider(min=0,max=len(callback.g_state)-1, value=len(callback.g_state)-1, continuous_update=False))

    # go back to original path
    if verbose > 0:
        print("Done")    
    os.chdir(starting_path)


# -----------------------------------------------------------------------------------
# NB. internal variables for the Runge Kutta formula (ie 'k_i') are not taken into account here... 
# -----------------------------------------------------------------------------------
def setWarmstart(sol_file, verbose = 1):
    
    # read .sol file
    solution = readSolution(filename=sol_file, verbose=verbose)
    
    # generate .init files and set options in .def file for all variables (state, control, parameter)
    for i in range(solution.dim_state):
        init_file= ('state.'+str(i)+'.init')
        np.savetxt(init_file, np.transpose(np.vstack((solution.time_steps, solution.state[i]))))
        setInDef(init_file, init_file)
    for i in range(solution.dim_control):
        init_file= ('control.'+str(i)+'.init')
        np.savetxt(init_file, np.transpose(np.vstack((solution.time_stages, solution.control[i]))))
        setInDef(init_file, init_file)
    for i in range(solution.dim_parameters):
        setInDef('parameter.'+str(i)+'.init', solution.parameters[i])


# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
def readDefFile(deffile = "problem.def"):

	options = {}

	with open(deffile) as f:
		for line in f:
			if line.strip() and line[0] != '#':
				(key, val) = line.split()
				options[key] = val
			
	return options


# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
def setInDef(key, value, deffile = "problem.def"):

    pattern = key + " " + str(value) + "\n"

    with open(deffile) as f:
        lines = f.readlines()

    # replace if present, append if not
    k = 0
    for line in lines:
        if key in line:
            lines[k] = pattern
            break
        k = k + 1

    if k == len(lines):
        lines.append(pattern)

    with open(deffile,'w+') as f:
        f.writelines(lines)


# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
def readSolution(filename='problem.sol', verbose=0):

    if verbose > 0:
        print("Loading solution: ",filename)

    sol = dOCPSolution()

    #+++ probably better to read whole file then extract parts with subfunctions
    #+++ with current getters reads need to be done in the correct order ...
    with open(filename,'r') as solfile:

        # read dimensions and constants
        sol.dim_boundary_conditions = getValue(solfile=solfile,label='dim.boundaryconditions',value_type='int')
        sol.dim_constants = getValue(solfile=solfile,label='dim.constants',value_type='int')
        sol.dim_control = getValue(solfile=solfile,label='dim.control',value_type='int')
        sol.dim_parameters = getValue(solfile=solfile,label='dim.parameters',value_type='int')
        sol.dim_path_constraints = getValue(solfile=solfile,label='dim.pathconstraints',value_type='int')
        sol.dim_state = getValue(solfile=solfile,label='dim.state',value_type='int')
        sol.dim_steps = getValue(solfile=solfile,label='time.steps',value_type='int')
        sol.ode_disc = getValue(solfile=solfile,label='ode.discretization',value_type='string',reset=True)
        # +++ use a function with proper switch case ?
        if sol.ode_disc == 'gauss3':
            sol.dim_stages = 3
        elif sol.ode_disc == 'gauss2' or sol.ode_disc == 'heun':
            sol.dim_stages = 2
        elif sol.ode_disc == 'midpoint_implicit' or sol.ode_disc == 'euler_implicit':
            sol.dim_stages = 1
        else:
            print('ERROR: unrecognized ode discretization method ', sol.ode_disc)
            print('Assume 1-stage method')
            
        for i in range(sol.dim_constants):
            sol.constants.append(getValue(solfile=solfile,label='constant.'+str(i),reset=True))

        # read status, iterations, objective ans constraints
        sol.status = getValue(solfile=solfile,label='Solver status:',value_type='int')
        sol.iterations = getValue(solfile=solfile,label='Solver iterations:',value_type='int')
        sol.objective = getValue(solfile=solfile,label='Objective function:')
        sol.constraints = getValue(solfile=solfile,label='Constraints violation:')

        # read time steps and stages
        sol.time_steps = getBlock1D(solfile=solfile,label='# Time steps',dim1=sol.dim_steps+1)
        sol.time_stages = getBlock1D(solfile=solfile,label='# Time stages',dim1=sol.dim_steps*sol.dim_stages)

        # read state, control, parameters
        sol.state = getBlock2D(solfile=solfile,label='# State',dim1=sol.dim_state,dim2=sol.dim_steps+1)
        sol.control = getBlock2D(solfile=solfile,label='# Control',dim1=sol.dim_control,dim2=sol.dim_steps*sol.dim_stages)
        sol.parameters = getBlock1D(solfile=solfile,label='# Parameters',dim1=sol.dim_parameters)

        # read boundary conditions and path constraints
        sol.boundary_conditions = getBlock1D(solfile=solfile,label='# Boundary Conditions',dim1=sol.dim_boundary_conditions)
        sol.path_constraints = getBlock2D(solfile=solfile,label='# Path constraints',dim1=sol.dim_path_constraints,dim2=sol.dim_steps+1)

        # read boundary conditions multipliers, path constraints multipliers, costate
        sol.boundary_conditions_multipliers = getBlock1D(solfile=solfile,label='# Multipliers associated to the boundary conditions',dim1=sol.dim_boundary_conditions)
        sol.path_constraints_multipliers = getBlock2D(solfile=solfile,label='# Multipliers associated to the path constraints',dim1=sol.dim_path_constraints,dim2=sol.dim_steps+1)
        sol.costate = getBlock2D(solfile=solfile,label='# Adjoint state',dim1=sol.dim_state,dim2=sol.dim_steps)

        # read average control on steps
        sol.average_control = getBlock2D(solfile=solfile,label='# Average control on steps',dim1=sol.dim_control,dim2=sol.dim_steps)        

    return sol


# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
def getValue(solfile, label, value_type = 'float', reset = False):

    # reset position at beginning of file
    if reset:
        solfile.seek(0)
    line = solfile.readline()
    while not label in line:
        line = solfile.readline()
    if value_type == 'float':
        value = float(line.split()[-1])
    elif value_type == 'int':
        value = int(line.split()[-1])
    else:
        value = line.split()[-1]
    return value


# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
def getBlock1D(solfile, label, dim1, reset = False):

    # reset position at beginning of file
    if reset:
        solfile.seek(0)
    block = np.empty([dim1])
    line = solfile.readline()
    while not label in line:
        line = solfile.readline()
    for k in range(0,dim1):
        line = solfile.readline()
        block[k] = float(line)

    return block


# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
def getBlock2D(solfile, label, dim1, dim2, reset = False):

    # reset position at beginning of file
    if reset:
        solfile.seek(0)
    block = np.empty([dim1,dim2])
    line = solfile.readline()
    for i in range(0,dim1):
        while not label in line:
            line = solfile.readline()
        for k in range(0,dim2):
            line = solfile.readline()
            block[i][k] = float(line)

    return block


# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
def hello(message):
    print(message)

# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
# TODO: add build options and bocop options
# +++ check messages refresh (when verbose=0)
def test(examples_root_path=bocop_root_path+'/examples', examples_list_prefix=bocop_root_path+'/test/examples', clean = 1, debug = 0, graph = 0, verbose = 0, benchmark = 0):

    start_time = time.time() # NB. time will include all build times

    # retrieve problem list
    examples_list_file = examples_list_prefix+'.list.csv'
    if verbose > 0:
        print("Bocop root path: {}".format(bocop_root_path))
        print("Problems path: {}".format(examples_root_path))
        print("Problems list: {}".format(examples_list_file))
      
    with open(examples_list_file,'r') as infile:
        blob = infile.readlines()

    # initialisations
    total = len(blob)
    current = 0
    failed = 0
    if benchmark == 1:
        prefix = 'benchmark'
    else:
        prefix = 'problem'

    for line in blob:
        current = current + 1
        ls = line.split()
        problem_path = os.path.normpath(os.path.join(examples_root_path,ls[0])) #NB. join uses / on mingw...
        print("{:2d}/{:2d} {:32s}".format(current,total,os.path.basename(os.path.normpath(problem_path))),end='',flush=True)
        # NB. we must run bocop in separate processes when we run different examples (to avoid reimporting the same lib)
        # +++ check this in more details, is it still needed ?
        solution = solve(problem_path=problem_path, prefix=prefix, clean=clean, debug=debug, graph=graph, verbose=verbose, separateProcess = 1)
        if verbose > 0:
            print("Bocop returns status {} with objective {} and constraint violation {}".format(solution.status,solution.objective,solution.constraints))

    # +++ check status
    #note: several ipopt status correspond to a successful run...
    #+++ btw 'acceptable' is saved as 4 instead of 1, check the save()

        # check objective
        objective_ref = float(ls[1])
        if abs((solution.objective - objective_ref)/objective_ref) < 1e-2:
            print("PASSED")
        else:
            print("FAILED\n with objective {} vs reference {}".format(solution.objective,objective_ref))
            failed = failed + 1

    # final summary
    print("--------------------------------------------")
    print("{:2d}/{:2d} TESTS PASSED           TOTAL TIME {:.2f}".format(total-failed,total,time.time()-start_time))
    return failed


# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
class dOCPDefinition:

    def __init__(self):

        self.dim_state = 0
        self.dim_control = 0
        self.dim_parameters = 0
        self.dim_constants = 0
        self.dim_boundary_conditions = 0
        self.dim_path_constraints = 0
        self.dim_steps = 0
        self.dim_stages = 0
        self.ode_disc = ''
        

# -----------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------
class dOCPSolution:

    def __init__(self):

        self.dim_state = 0
        self.dim_control = 0
        self.dim_parameters = 0
        self.dim_constants = 0
        self.dim_boundary_conditions = 0
        self.dim_path_constraints = 0
        self.dim_steps = 0
        self.dim_stages = 0

        self.objective = 0
        self.constraints = 0
        self.status = 0
        self.iterations = 0

        # np array here ? freaking annoying to initialize without the dims...
        self.time_steps = []
        self.time_stages = []
        self.state = []
        self.control = []
        self.parameters = []
        self.constants = []
        self.boundary_conditions = []
        self.path_constraints = []
        self.boundary_conditions_multipliers = []
        self.path_constraints_multipliers = []
        self.costate = []
        self.average_control = []

# -----------------------------------------------------------------------------------
    def plot(self, graph = 1, blck = True):

        plt.figure()
        nb_plots = len(self.state) * graph + len(self.control)
        index = 0
        for x in self.state:
            plt.subplot(1,nb_plots,index+1)
            plt.plot(self.time_steps, x)
            plt.xlabel("TIME")
            plt.title("STATE {}".format(index))
            index = index + 1
        index = 0
        for u in self.control:
            plt.subplot(1,nb_plots,len(self.state)+index+1)
            plt.plot(self.time_stages, u, 'r')
            plt.xlabel("TIME")
            plt.title("CONTROL {}".format(index))
            index = index + 1
        if graph > 1:
            index = 0
            for p in self.costate:
                plt.subplot(1,nb_plots,len(self.state)+len(self.control)+index+1)
                plt.plot(self.time_stages, p, 'k')
                plt.xlabel("TIME")
                plt.title("COSTATE {}".format(index))
                index = index + 1

        plt.show(block=blck) #block=False when called from PySimpleGUIQt

        # later constructor with filename as argument to fill solution directly


########################################################################
########################################################################
# execute command with output in realtime
def runCommand(command, verbose = 1, window = None):

    # set codec
    if (platform.system() == 'Windows'):
        codec = 'cp850' # this is some old windows dark age encoding.
    else:
        codec = 'utf-8'

    # open subprocess for command and poll output
    p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, encoding=codec)
    if verbose > 0:
        print(f"[EXEC] > {command}")
        while p.poll() == None:
            for line in p.stdout: ## this only works on Windows tho
                sys.stdout.write(f">\t{line}")
            if window:
                window.Refresh()
        print(f"[DONE] > {command}")
    
    p.communicate() # wait for process to finish
    
    return p.returncode
