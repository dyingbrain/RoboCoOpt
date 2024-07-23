from sgi_robocoopt import *
from setuptools import setup, find_packages

NAME = 'sgi_robocoopt'
VERSION = '0.0.1'
DESC = 'RoboCoOpt: Robot Co-optimization using planar linkage mechanisms'

setup(
    name=NAME,
    version=VERSION,
    url='https://github.com/aniketrajnish/RoboCoOpt.git',
    author='Aniket Rajnish',
    author_email='rajnish.ankt@gmail.com',
    description=DESC,
    packages=find_packages(),    
    install_requires=['numpy', 'box2d', 'box2d-py', 'pygme', 'matplotlib', 'scipy', 'pyqt5', 'vapory', 'pygad', 'mcts', 'monte-carlo-tree-search']
)