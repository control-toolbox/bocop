import os
import sys
import bocop

bocop.hello("Launching bocop benchmark...")
ret = bocop.test(examples_list_prefix='./bocop/test/benchmark', benchmark=1)
exit(ret)
