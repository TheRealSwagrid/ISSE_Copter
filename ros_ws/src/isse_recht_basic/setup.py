from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['isse_recht_basic'],
    package_dir={'': 'src'}
)
setup(**d)