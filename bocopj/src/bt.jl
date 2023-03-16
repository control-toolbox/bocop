# bocop module dev/test

using Revise
using bocopj

# test bocop root path
if false
    println("root path: ",bocopj.bocop_root_path)
    println(readdir(bocopj.bocop_root_path))
end

path = "/home/martinon/bocop/bocop3/bocop/examples/vanderpol"

# test build
if false
    println("test: build_bocopApp")
    bocopj.build_bocopApp(path)
    println("end test: build_bocopApp")
end

# test run
if false
    println("test: run_bocopApp")
    bocopj.run_bocopApp(path)
    println("end test: run_bocopApp")
end

solfile = Base.open(path*"/problem.sol")

# test getValue
if false
    label = "time.steps"
    v = bocopj.getValue(solfile, label, "int", true)
    println(label," ", v, " ", typeof(v))
    label = "initial.time"
    v = bocopj.getValue(solfile, label, "float", true)
    println(label," ", v, " ", typeof(v))
    label = "ode.discretization"
    v = bocopj.getValue(solfile, label, "string", true)
    println(label," ", v, " ", typeof(v))
end

# test getBlock1D
if false
    dim = bocopj.getValue(solfile, "time.steps", "int", true)
    label = "# Time steps"
    v = bocopj.getBlock1D(solfile, label, dim+1, true)
    println(label," ", v, " ", typeof(v))
end

# test getBlock2D
if false
    dim1 = bocopj.getValue(solfile, "dim.state", "int", true)    
    dim2 = bocopj.getValue(solfile, "time.steps", "int", true)
    println(dim1,' ', dim2)
    label = "# State"
    v = bocopj.getBlock2D(solfile, label, dim1, dim2+1, true)
    plot(v[1,:])
    plot!(v[2,:])
end
    
# test read_bocop_sol and plot_bocop_sol
if true
    solfile_path = path * "/problem.sol"
    println(solfile_path)
    sol = bocopj.read_bocop_sol(solfile_path)
    bocopj.plot_bocop_sol(sol)
end



