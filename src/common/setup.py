from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['my_modules'],
        package_dir={'': 'modules'}
)

setup(**setup_args)
