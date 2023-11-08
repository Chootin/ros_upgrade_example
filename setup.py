from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ros_upgrade_example'],
    package_dir={'': 'ros_upgrade_example'}
)

setup(**d)