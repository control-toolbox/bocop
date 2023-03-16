from .bocop import *
from .jupyter_widgets import *

with open(bocop_root_path+'/VERSION') as version_file:
  __version__ = version_file.read().strip()

__author__ = 'Pierre Martinon'
__license__ = 'EPL'
