import os
import sys
import bocop

if len(sys.argv) > 1:
  verb = int(sys.argv[1])
else:
  verb = 0

bocop.hello("Testing bocop examples...")

#examples_root_path = os.path.join(bocop.bocop_root_path,"examples")
#examples_list_prefix = os.path.join(bocop.bocop_root_path,"test/examples")
#bocop.test(examples_root_path, examples_list_prefix, verbose = 0)
ret = bocop.test(verbose=verb)
exit(ret)
