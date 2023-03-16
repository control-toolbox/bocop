# Bocop julia utils for bocop3
# Pierre Martinon

module bocopj

using Parameters
using Plots

module_src_path = @__DIR__
bocop_root_path = module_src_path * "/../../bocop"

function run_bocopApp(problem_path)

    # execution takes place in problem directory because of input/output files
    module_dir = pwd()
    cd(problem_path)
    Base.run(`./bocopApp`)
    cd(module_dir)

end

function build_bocopApp(problem_path, buildtype = "Release")

    # build takes place in problem directory because of temp files
    module_dir = pwd()
    cd(problem_path)
    if !isdir("build")
        mkdir("build")
    end
    cd("build")

    # linux / macos
    cmake_command = `cmake -DCMAKE_BUILD_TYPE=$buildtype -DPROBLEM_DIR=$problem_path $bocop_root_path`
    make_command = `make -j`

    # execute cmake and make steps
    Base.run(cmake_command)
    Base.run(make_command)
    cd(module_dir)

end

@with_kw mutable struct bocopSolution

    dim_state::Int = 0
    dim_control::Int = 0
    dim_parameters::Int = 0
    dim_constants::Int = 0
    dim_boundary_conditions::Int = 0
    dim_path_constraints::Int = 0
    dim_steps::Int = 0
    dim_stages::Int = 0
    ode_disc::String = " "

    objective = 0
    constraints = 0
    status = 0
    iterations = 0

    time_steps = nothing
    time_stages = nothing
    state = nothing
    control = nothing
    parameters = nothing
    constants = nothing
    boundary_conditions = nothing
    path_constraints = nothing
    boundary_conditions_multipliers = nothing
    path_constraints_multipliers = nothing
    costate = nothing
    average_control = nothing

end

# add EOF test ?
function getValue(solfile, label, value_type, reset)

    if reset
        Base.seek(solfile,0)
    end
    line = readline(solfile)
    while !occursin(label,line)
        line = readline(solfile)
    end
    if value_type == "float"
        value = parse(Float64, last(split(line)))
    elseif value_type == "int"
        value = parse(Int64, last(split(line)))
    else
        value = String(last(split(line)))
    end
    return value

end

function getBlock1D(solfile, label, dim1, reset)

    if reset
        Base.seek(solfile,0)
    end
    block = zeros(dim1)
    line = readline(solfile)
    while !occursin(label,line)
        line = readline(solfile)
    end
    for k in range(1, dim1)
        line = readline(solfile)
        block[k] = parse(Float64,line)
    end
    return block

end

function getBlock2D(solfile, label, dim1, dim2, reset)

    if reset
        Base.seek(solfile,0)
    end
    block = zeros(dim1, dim2)
    line = readline(solfile)

    for i in range(1,dim1)
        while !occursin(label,line)
            line = readline(solfile)
        end
        for k in range(1,dim2)
            line = readline(solfile)
            block[i,k] = parse(Float64,line)
        end    
    end
    return block

end

function read_bocop_sol(solfile_path)

    sol = bocopSolution()
    open(solfile_path) do solfile

        # read dimensions and constants
        sol.dim_boundary_conditions = getValue(solfile,"dim.boundaryconditions","int",false)
        sol.dim_constants = getValue(solfile,"dim.constants","int",false)
        sol.dim_control = getValue(solfile,"dim.control","int",false)
        sol.dim_parameters = getValue(solfile,"dim.parameters","int",false)
        sol.dim_path_constraints = getValue(solfile,"dim.pathconstraints","int",false)
        sol.dim_state = getValue(solfile,"dim.state","int",false)
        sol.dim_steps = getValue(solfile,"time.steps","int",false)
        sol.ode_disc = getValue(solfile,"ode.discretization","string",true)

        if sol.ode_disc == "gauss3"
            sol.dim_stages = 3
        elseif sol.ode_disc == "gauss2" || sol.ode_disc == "heun"
            sol.dim_stages = 2
        elseif sol.ode_disc == "midpoint_implicit" || sol.ode_disc == "euler_implicit"
            sol.dim_stages = 1
        else
            println("ERROR: unrecognized ode discretization method ", sol.ode_disc)
            println("Assume 1-stage method")
            sol.dim_stages = 1
        end

        if sol.dim_constants > 0
            sol.constants = zeros(sol.dim_constants)
            for i in range(1,sol.dim_constants)
                sol.constants[i] = getValue(solfile,"constant."*string(i-1),"float",true)
            end
        end

        # read status, iterations, objective ans constraints
        sol.status = getValue(solfile,"Solver status:","int",false)
        sol.iterations = getValue(solfile,"Solver iterations:","int",false)
        sol.objective = getValue(solfile,"Objective function:","float",false)
        sol.constraints = getValue(solfile,"Constraints violation:","float",false)

        # read time steps and stages
        sol.time_steps = getBlock1D(solfile,"# Time steps",sol.dim_steps+1,false)
        sol.time_stages = getBlock1D(solfile,"# Time stages",sol.dim_steps*sol.dim_stages,false)

        # read state, control, parameters
        sol.state = getBlock2D(solfile,"# State",sol.dim_state,sol.dim_steps+1,false)
        sol.control = getBlock2D(solfile,"# Control",sol.dim_control,sol.dim_steps*sol.dim_stages,false)
        sol.parameters = getBlock1D(solfile,"# Parameters",sol.dim_parameters,false)

        # read boundary conditions and path constraints
        sol.boundary_conditions = getBlock1D(solfile,"# Boundary Conditions",sol.dim_boundary_conditions,false)
        sol.path_constraints = getBlock2D(solfile,"# Path constraints",sol.dim_path_constraints,sol.dim_steps,false)

        # read boundary conditions multipliers, path constraints multipliers, costate
        sol.boundary_conditions_multipliers = getBlock1D(solfile,"# Multipliers associated to the boundary conditions",sol.dim_boundary_conditions,false)
        sol.path_constraints_multipliers = getBlock2D(solfile,"# Multipliers associated to the path constraints",sol.dim_path_constraints,sol.dim_steps,false)
        sol.costate = getBlock2D(solfile,"# Adjoint state",sol.dim_state,sol.dim_steps,false)

        # read average control on steps
        sol.average_control = getBlock2D(solfile,"# Average control on steps",sol.dim_control,sol.dim_steps,false)        

    end
    return sol

end

function plot_bocop_sol(sol)

  println("Objective: ",sol.objective," Constraints: ",sol.constraints," Iterations: ",sol.iterations)

  # reparse solution data
  n = sol.dim_state
  m = sol.dim_control
  T = sol.time_steps
  TS = sol.time_stages
  X = transpose(sol.state)
  U = transpose(sol.control)
  P = transpose(sol.costate)

  # state 
  px = Plots.plot(T, X[:,1:n], layout=(1,n))    
  Plots.plot!(px[1], title="state")
  Plots.plot!(px[n], xlabel="t")
  for i in 1:n
    Plots.plot!(px[i], ylabel=string("x_",i))
  end

  # costate
  pp = Plots.plot(T[1:end-1], P[:,1:n],layout = (1,n))    
  Plots.plot!(pp[1], title="costate")
  Plots.plot!(pp[n], xlabel="t")
  for i in 1:n
    Plots.plot!(pp[i], ylabel=string("p_",i))
  end

  # control
  pu = Plots.plot(TS, U, lc=:red, layout=(1,m))  
  for i in 1:m
    Plots.plot!(pu[i], ylabel=string("u_",i))
  end
  Plots.plot!(pu[1], title="control")
  Plots.plot!(pu[m], xlabel="t")

  # main plot
  Plots.plot(px, pp, pu, layout=(3,1), legend=false) #row layout

end


# exports
export run_bocopApp
export build_bocopApp


end # module bocopj
