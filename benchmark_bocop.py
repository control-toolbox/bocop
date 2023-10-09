import os
import sys
import bocop

bocop.hello("Launching bocop benchmark...")
opt_dict = {"ipoptNumOption.tol": "1e-8",
            "ipoptIntOption.max_iter": "1000",
            "ipoptStrOption.mu_strategy": "adaptive",
            "ode.discretization": "midpoint_implicit",
            #"ode.discretization": "gauss3",
            "time.steps": "1000",
}
ret = bocop.test(examples_list_prefix='./bocop/test/benchmark', benchmark=1, broadcast_options=opt_dict)
exit(ret)
