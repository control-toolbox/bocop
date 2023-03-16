from setuptools import setup

def get_version():
  with open('bocop/VERSION') as version_file:
    version = version_file.read().strip()
  return version

setup(name='bocop',
      version=get_version(),
      packages=['bocop'],
      include_package_data=True,              # install files set in MANIFEST.in (only if INSIDE a module -_-)
      zip_safe=False,                         # disable install as an .egg archive (we need the C sources as actual files !)
      entry_points={'console_scripts': ['bocopGUI = bocop.bocopGUI:bocop3_gui']},
     )
     
