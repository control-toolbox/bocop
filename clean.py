# why is this not a freaking DEFAULT option in setuptools. FFS ?!
import os
import shutil

ls = [f.name for f in os.scandir(os.getcwd()) if f.is_dir()]
for folder in ls:
  if folder == 'build' or folder == 'dist' or ('egg-info' in folder):
    print('removing',folder)
    shutil.rmtree(folder)
