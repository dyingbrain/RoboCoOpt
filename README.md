
### Robot Co-optimization using planar linkage
The base code implements the planar linkage kinematics in `linkage.py`, and the linkage can be installed on a robot and simulate to test the robot performance using `linkage_physics.py`.  Finally, we implemented a basic linkage optimizer using simulated annealing in `optimizer_anneal.py`.

#### Install

To run the code, you first need to install Anaconda, which can be downloaded from https://docs.anaconda.com/anaconda/install/

Once Anaconda is installed, you can open the anaconda and type the following commands in order:

1. install box2dpy (https://github.com/pybox2d/pybox2d). Fortunately, you can install it via Anaconda.
`conda install conda-forge::box2d-py`

2. now, you can create your conda environment, named "box2d-py"
`conda create -n box2d-py`

3. remember every time you activate the environment, before you use it. 
`conda activate box2d-py`

4. once you activate the environment for the first time, you need to install the following packages.
`pip install pygame`
`pip install numpy`
`pip install vapory`

5. run the Python scripts one by one
`python linkage.py`
`python linkage_physics.py`
`python optimizer_anneal.py`




