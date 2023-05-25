Robot Properties Go1
---------------------

### What it is

Collection of configuration files for the go1 robot.

### Installation

1. Install Pinocchio if you have not done so already.

   The simplest way to do so is using Conda:

   ```
   conda install -c conda-forge pinocchio
   ```

   Alternatively you can use any of the ways listed here: https://stack-of-tasks.github.io/pinocchio/download.html.

2. Install bullet_utils:

  ```
  git clone git@github.com:machines-in-motion/bullet_utils.git
  cd bullet_utils
  pip3 install .
  ```

3. Install robot_properties_go1:

  ```
  git clone git@github.com:open-dynamic-robot-initiative/robot_properties_go1.git
  cd robot_properties_go1
  pip3 install .
  ```

### Examples

Below are a few examples. You find more in the `demos/` folder.

**Loading Go1 in PyBullet**

```
import pybullet as p
from bullet_utils.env import BulletEnvWithGround
from robot_properties_go1.go1wrapper import Go1Robot

env = BulletEnvWithGround(p.GUI)
robot = env.add_robot(Go1Robot)
```

**Run simulation on Max-Planck Institute cluster**

```
conda create -n go1 python=3.7
source activate go1
conda install -c conda-forge pinocchio 

git clone git@github.com:machines-in-motion/bullet_utils.git
cd bullet_utils
pip3 install .

git clone git@github.com:open-dynamic-robot-initiative/robot_properties_go1.git
cd robot_properties_go1
pip3 install .
```

### License and Copyrights

Copyright(c) 2018-2021 Max Planck Gesellschaft, New York University

BSD 3-Clause License
