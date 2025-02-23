# Robot Dynamics Calculator

A Python-based GUI application for calculating and visualizing joint forces and torques in robotic manipulators. This tool provides real-time force and torque analysis when external forces are applied to any joint in the robot's kinematic chain.

## Features

- Real-time calculation of joint forces and torques
- Interactive GUI for force and torque application
- PyBullet-based 3D visualization
- Support for both revolute and prismatic joints
- Considers gravity, damping, and friction effects
- Dynamic force propagation through the kinematic chain
- Jacobian-based force transformation
- Real-time sensor feedback

## Requirements

### Software Dependencies
```bash
python >= 3.8
numpy
scipy
pybullet
tkinter
```

### Installation

1. Clone the repository:
```bash
git clone https://github.com/yourusername/robot-dynamics-calculator.git
cd robot-dynamics-calculator
```

2. Install required packages:
```bash
pip install numpy scipy pybullet
```

Note: tkinter usually comes pre-installed with Python. If not, install it using your system's package manager:
- Ubuntu/Debian: `sudo apt-get install python3-tk`
- macOS: `brew install python-tk`
- Windows: Included with Python installation

## Usage

### Basic Setup

1. Place your robot's URDF file in the project directory:
   - Name it `robot_Arm.urdf`
   - Ensure the URDF has proper joint definitions and dynamics parameters

2. Run the application:
```bash
python torque_calculator.py
```

### GUI Interface

The application provides an intuitive interface with the following components:

1. **Joint Selection**
   - Dropdown menu to select the joint for force/torque application
   - Shows only movable joints (revolute and prismatic)

2. **Force and Torque Input**
   - Force input field (Newtons)
   - Torque input field (Newton-meters)
   - Currently applies force in Z-direction (vertical)

3. **Control Buttons**
   - Apply: Execute the force/torque application
   - Reset: Return simulation to initial state

4. **Real-time Display**
   - Force magnitude for each joint
   - Torque magnitude for each joint
   - Joint names and types

### Modifying Force Direction

Currently, the application applies forces in the Z-direction. To modify for different directions:

1. Locate the `calculate_dynamics()` method
2. Modify the force vector calculation:
```python
force_vector = [force_x, force_y, force_z]  # Instead of [0, 0, force_value]
```

## Technical Details

### Key Components

1. **Dynamics Calculation**
   - Uses Jacobian matrices for force propagation
   - Includes gravity compensation
   - Considers joint damping and friction
   - Real-time force/torque sensor feedback

2. **Simulation Parameters**
   - Time step: 1/240 seconds
   - Gravity: -9.81 m/s² in Z-direction
   - Fixed-base robot configuration

3. **Joint Analysis**
   - Automatic DOF calculation
   - Joint type detection
   - Dynamic parameter extraction from URDF

### Error Handling

The application includes robust error handling for:
- Invalid input values
- URDF loading issues
- Jacobian calculation errors
- Simulation state inconsistencies

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

### Development Guidelines

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request



## Acknowledgments

- PyBullet for providing the physics simulation engine
- The robotics community for URDF standardization
