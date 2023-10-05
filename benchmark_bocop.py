import os
import sys
import bocop

bocop.hello("Launching bocop benchmark...")
opt_dict = {"time.steps": "1000", 
            "ode.discretization": "midpoint_implicit"}
ret = bocop.test(examples_list_prefix='./bocop/test/benchmark', benchmark=1, broadcast_options=opt_dict)
exit(ret)
