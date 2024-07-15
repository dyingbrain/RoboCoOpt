
### Robot Co-optimization using planar linkage
The base code implements the planar linkage kinematics in `linkage.py`, and the linkage can be installed on a robot and simulate to test the robot performance using `linkage_physics.py`.  Finally, we implemented a basic linkage optimizer using simulated annealing in `optimizer_anneal.py`.

#### Install

To run the code, you first need to install Anaconda, which can be downloaded from https://docs.anaconda.com/anaconda/install/

Once Anaconda is installed, you can open the anaconda and type the following commands in order:

1. install box2dpy (https://github.com/pybox2d/pybox2d). Fortunately, you can install it via Anaconda.
   
`conda install conda-forge::box2d-py`

3. now, you can create your conda environment, named "box2d-py"
   
`conda create -n box2d-py`

5. remember every time you activate the environment, before you use it.
   
`conda activate box2d-py`

7. once you activate the environment for the first time, you need to install the following packages.
   
`pip install pygame`

`pip install numpy`

`pip install vapory`

9. run the Python scripts one by one
    
`python linkage.py`

`python linkage_physics.py`

`python optimizer_anneal.py`

### Installing Povray on Windows

On windows, the default povray does not work. We need a special version of povray that is independently compiled called MegaPov.
I have included a version of this MegaPov in the project, i.e., the megapov.rar. To enable povray on windows, unzip this folder to any directory.
Let's say you have a povray.exe located in "C:\Users\Admin\AppData\Roaming\POV-Ray\bin".
After this step, you need to add this to your system path.
Now you can press "LSHIFT" in the window to have the povray render a nice image for you, an example is as follows:
![alt text](https://github.com/dyingbrain/RoboCoOpt/blob/main/frm.png)
