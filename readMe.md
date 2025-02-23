# PyBullet

PyBullet is an easy to use Python module for physics simulation, robotics and machine learning. It is built on top of the Bullet Physics SDK.

## Installation

To install PyBullet, you can use pip:

```bash
pip install pybullet
```

## Usage

Here is a simple example of how to use PyBullet:

```python
import pybullet as p
import time

# Connect to the physics server
p.connect(p.GUI)

# Load a plane and a robot
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("r2d2.urdf")

# Set the gravity
p.setGravity(0, 0, -9.8)

# Run the simulation
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

# Disconnect from the server
p.disconnect()
```

## Documentation

For more detailed information, please refer to the [PyBullet documentation](https://pybullet.org/wordpress/).

## Using the package
Run the python code, before that change the path of urdf according to your system. 
Click on start button on gui of pybullet, you will notice the robot to move towards teh box , pick it up and place it at a location, you can change that on the code too easily in the execute_pick_and_place function.


```
Press Ctrl + Shift + V (Windows/Linux) or Cmd + Shift + V (Mac) to open the preview.
```
