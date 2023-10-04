import os
import sys
import bocop

if len(sys.argv) > 1:
  verb = int(sys.argv[1])
else:
  verb = 0

bocop.hello("Testing bocop examples...")
ret = bocop.test(verbose=verb)
exit(ret)
