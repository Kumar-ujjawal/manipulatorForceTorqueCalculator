import pybullet as p
import pybullet_data
import numpy as np
import time
import tkinter as tk
from tkinter import ttk
import signal
import sys
from scipy.spatial.transform import Rotation

class TorqueCalculator:
    def __init__(self):
        self.robot_id = None
        self.num_joints = None
        self.num_dof = None
        self.root = None
        self.torque_labels = []
        self.force_labels = []
        self.running = True
        self.current_force = 0
        self.current_torque = 0
        self.current_joint = 0
        self.force_applied = False
        self.joint_info = {}
        self.movable_joints = []  # List of joints that contribute to DOF
        
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        print("\nClosing application...")
        self.running = False
        if self.root:
            self.root.quit()
        p.disconnect()
        sys.exit(0)

    def calculate_dof(self):
        """Calculate total DOF based on joint types"""
        self.num_dof = 0
        self.movable_joints = []
        
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_type = joint_info[2]
            
            # p.JOINT_REVOLUTE, p.JOINT_PRISMATIC contribute 1 DOF each
            if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                self.num_dof += 1
                self.movable_joints.append(i)
                
        print(f"Total DOF: {self.num_dof}")
        print(f"Movable joints: {self.movable_joints}")
        return self.num_dof

    def get_joint_info(self):
        """Get and store information about each joint"""
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot_id, i)
            self.joint_info[i] = {
                'name': info[1].decode('utf-8'),
                'type': info[2],
                'damping': info[6],
                'friction': info[7],
                'lower_limit': info[8],
                'upper_limit': info[9],
                'max_force': info[10],
                'max_velocity': info[11],
                'axis': info[13]
            }

    def get_movable_joint_states(self):
        """Get positions and velocities of movable joints"""
        positions = []
        velocities = []
        
        for joint_idx in self.movable_joints:
            state = p.getJointState(self.robot_id, joint_idx)
            positions.append(state[0])
            velocities.append(state[1])
            
        return positions, velocities

    def get_jacobian(self, joint_idx):
        """Calculate the Jacobian for a specific joint"""
        # Get joint positions and velocities for movable joints
        positions, velocities = self.get_movable_joint_states()
        
        # Get the end effector position
        link_state = p.getLinkState(self.robot_id, joint_idx)
        local_pos = [0, 0, 0]  # Local position on the link
        
        # Calculate Jacobian with correct DOF
        zero_vec = [0.0] * self.num_dof  # Zero acceleration
        
        try:
            linear_jac, angular_jac = p.calculateJacobian(
                bodyUniqueId=self.robot_id,
                linkIndex=joint_idx,
                localPosition=local_pos,
                objPositions=positions,
                objVelocities=velocities,
                objAccelerations=zero_vec
            )
            
            return np.array(linear_jac), np.array(angular_jac)
        except p.error as e:
            print(f"Error calculating Jacobian: {e}")
            print(f"Joint index: {joint_idx}")
            print(f"Positions: {positions}")
            print(f"Velocities: {velocities}")
            print(f"Num DOF: {self.num_dof}")
            raise

    def apply_force_and_get_response(self, joint_idx, force_vector, torque_vector):
        """Apply force/torque to joint and measure response"""
        # Apply force and torque in world frame
        p.applyExternalForce(
            objectUniqueId=self.robot_id,
            linkIndex=joint_idx,
            forceObj=force_vector,
            posObj=[0, 0, 0],  # Apply at joint center
            flags=p.WORLD_FRAME
        )
        
        p.applyExternalTorque(
            objectUniqueId=self.robot_id,
            linkIndex=joint_idx,
            torqueObj=torque_vector,
            flags=p.WORLD_FRAME
        )
        
        # Step simulation to see effect
        p.stepSimulation()
        
        # Get resulting joint states
        joint_states = []
        for i in range(self.num_joints):
            state = p.getJointState(self.robot_id, i)
            joint_states.append({
                'position': state[0],
                'velocity': state[1],
                'reaction_forces': state[2],  # 6 values: [Fx, Fy, Fz, Mx, My, Mz]
                'applied_motor_torque': state[3]
            })
        
        return joint_states

    def calculate_dynamics(self, applied_joint, force_value, torque_value):
        """Calculate joint forces and torques"""
        # Convert scalar inputs to vectors (assuming force/torque in z-direction)
        force_vector = [0, 0, force_value]
        torque_vector = [0, 0, torque_value]
        
        # Get joint states before force application
        initial_states = self.get_movable_joint_states()[0]
        
        # Apply force and get response
        joint_states = self.apply_force_and_get_response(applied_joint, force_vector, torque_vector)
        
        # Initialize arrays for storing results
        joint_forces = np.zeros((self.num_joints, 3))
        joint_torques = np.zeros((self.num_joints, 3))
        
        # Get Jacobian for force propagation
        if applied_joint in self.movable_joints:
            linear_jac, angular_jac = self.get_jacobian(applied_joint)
            
            # Calculate force and torque propagation through the kinematic chain
            for i, joint_idx in enumerate(self.movable_joints):
                if joint_idx <= applied_joint:
                    # Get reaction forces and torques
                    reaction = joint_states[joint_idx]['reaction_forces']
                    joint_forces[joint_idx] = reaction[:3]
                    joint_torques[joint_idx] = reaction[3:]
                    
                    # Add effect of applied force/torque
                    if joint_idx == applied_joint:
                        joint_forces[joint_idx] += force_vector
                        joint_torques[joint_idx] += torque_vector
                    else:
                        # Propagate through kinematic chain using Jacobian
                        force_effect = np.dot(linear_jac[i].T, force_vector)
                        torque_effect = np.dot(angular_jac[i].T, torque_vector)
                        joint_forces[joint_idx] += force_effect
                        joint_torques[joint_idx] += torque_effect
                    
                    # Add gravity compensation
                    dynamics_info = p.getDynamicsInfo(self.robot_id, joint_idx)
                    mass = dynamics_info[0]
                    gravity_force = [0, 0, -9.81 * mass]
                    joint_forces[joint_idx] += gravity_force
                    
                    # Add damping and friction effects
                    velocity = joint_states[joint_idx]['velocity']
                    damping = self.joint_info[joint_idx]['damping']
                    friction = self.joint_info[joint_idx]['friction']
                    
                    damping_torque = damping * velocity
                    friction_torque = friction * np.sign(velocity)
                    
                    joint_torques[joint_idx][2] += damping_torque + friction_torque
        
        return joint_forces, joint_torques

    def update_dynamics(self):
        """Update the GUI with new force and torque values"""
        try:
            selected_joint = self.joint_var.get()
            force_value = float(self.force_entry.get())
            torque_value = float(self.torque_entry.get())
            
            # Update current values
            self.current_joint = selected_joint
            self.current_force = force_value
            self.current_torque = torque_value
            self.force_applied = True
            
            # Calculate forces and torques
            joint_forces, joint_torques = self.calculate_dynamics(selected_joint, force_value, torque_value)
            
            # Update GUI
            for i in range(self.num_joints):
                force_mag = np.linalg.norm(joint_forces[i])
                torque_mag = np.linalg.norm(joint_torques[i])
                
                self.force_labels[i].config(
                    text=f"Joint {i} ({self.joint_info[i]['name']}) Force: {force_mag:.4f} N"
                )
                self.torque_labels[i].config(
                    text=f"Joint {i} ({self.joint_info[i]['name']}) Torque: {torque_mag:.4f} Nm"
                )
            
            # Print detailed information
            print(f"\nApplied at Joint {selected_joint} ({self.joint_info[selected_joint]['name']}):")
            print(f"Force: {force_value} N")
            print(f"Torque: {torque_value} Nm")
            print("\nResulting Joint Forces and Torques:")
            for i in range(self.num_joints):
                if i in self.movable_joints:
                    print(f"\nJoint {i} ({self.joint_info[i]['name']}):")
                    print(f"Force vector: {joint_forces[i]}")
                    print(f"Force magnitude: {np.linalg.norm(joint_forces[i]):.4f} N")
                    print(f"Torque vector: {joint_torques[i]}")
                    print(f"Torque magnitude: {np.linalg.norm(joint_torques[i]):.4f} Nm")
            print("-" * 50)
            
        except ValueError as e:
            print(f"Error: {e}")
            print("Please enter valid numeric values for force and torque")

    def reset_simulation(self):
        """Reset the simulation to initial state"""
        p.resetSimulation()
        self.load_robot()
        
        self.force_applied = False
        self.current_force = 0
        self.current_torque = 0
        self.current_joint = 0
        
        self.force_entry.delete(0, tk.END)
        self.force_entry.insert(0, "0")
        self.torque_entry.delete(0, tk.END)
        self.torque_entry.insert(0, "0")
        self.joint_var.set(0)
        
        for i in range(self.num_joints):
            self.force_labels[i].config(
                text=f"Joint {i} ({self.joint_info[i]['name']}) Force: 0.0000 N"
            )
            self.torque_labels[i].config(
                text=f"Joint {i} ({self.joint_info[i]['name']}) Torque: 0.0000 Nm"
            )
        
        print("\nSimulation Reset")
        print("-" * 50)

    def load_robot(self):
        """Load robot and initialize"""
        self.robot_id = p.loadURDF("robot_Arm.urdf", [0, 0, 0], useFixedBase=True)
        self.num_joints = p.getNumJoints(self.robot_id)
        
        # Get joint information and calculate DOF
        self.get_joint_info()
        self.calculate_dof()
        
        print("\nInitializing robot joints:")
        for i in range(self.num_joints):
            print(f"Joint {i}: {self.joint_info[i]['name']}")
            print(f"Type: {self.joint_info[i]['type']}")
            p.enableJointForceTorqueSensor(self.robot_id, i, enableSensor=True)
        print("-" * 50)

    def create_gui(self):
        """Create the GUI interface"""
        self.root = tk.Tk()
        self.root.title("Robot Dynamics Calculator")
        self.root.protocol("WM_DELETE_WINDOW", self.signal_handler)
        
        # Joint selection
        ttk.Label(self.root, text="Select Joint:").grid(row=0, column=0)
        self.joint_var = tk.IntVar()
        joint_selector = ttk.Combobox(self.root, textvariable=self.joint_var, 
                                    values=self.movable_joints)
        joint_selector.grid(row=0, column=1)
        joint_selector.set(self.movable_joints[0] if self.movable_joints else 0)
        
        # Force and torque inputs
        ttk.Label(self.root, text="Force (N):").grid(row=1, column=0)
        self.force_entry = ttk.Entry(self.root)
        self.force_entry.grid(row=1, column=1)
        self.force_entry.insert(0, "0")
        
        ttk.Label(self.root, text="Torque (Nm):").grid(row=2, column=0)
        self.torque_entry = ttk.Entry(self.root)
        self.torque_entry.grid(row=2, column=1)
        self.torque_entry.insert(0, "0")
        
        # Control buttons
        ttk.Button(self.root, text="Apply", command=self.update_dynamics).grid(row=3, column=0, columnspan=2, pady=10)
        ttk.Button(self.root, text="Reset", command=self.reset_simulation).grid(row=4, column=0, columnspan=2)
        
        # Force and torque display labels
        self.force_labels = []
        self.torque_labels = []
        for i in range(self.num_joints):
            force_lbl = ttk.Label(self.root, text=f"Joint {i} ({self.joint_info[i]['name']}) Force: 0.0000 N")
            force_lbl.grid(row=5 + i*2, column=0, columnspan=2)
            self.force_labels.append(force_lbl)
            
            torque_lbl = ttk.Label(self.root, text=f"Joint {i} ({self.joint_info[i]['name']}) Torque: 0.0000 Nm")
            # torque_lbl = ttk.Label(self.root, text=f"Joint {i} ({self.joint_info[i]['name']}) Torque: 0.0000 Nm")
            torque_lbl.grid(row=6 + i*2, column=0, columnspan=2)
            self.torque_labels.append(torque_lbl)

    def run_simulation(self):
        """Initialize and run the simulation"""
        # Connect to PyBullet
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Configure simulation parameters
        p.setTimeStep(1./240.)  # Set simulation timestep
        p.setRealTimeSimulation(0)  # Disable real-time simulation for better accuracy
        
        # Load robot and create GUI
        self.load_robot()
        self.create_gui()
        
        # Start simulation loop
        self.root.after(10, self.simulation_loop)
        self.root.mainloop()

    def simulation_loop(self):
        """Main simulation loop"""
        if self.running:
            if self.force_applied:
                # Only calculate dynamics if force/torque is being applied
                joint_forces, joint_torques = self.calculate_dynamics(
                    self.current_joint, 
                    self.current_force, 
                    self.current_torque
                )
                
                # Update GUI with new values
                for i in range(self.num_joints):
                    force_mag = np.linalg.norm(joint_forces[i])
                    torque_mag = np.linalg.norm(joint_torques[i])
                    
                    self.force_labels[i].config(
                        text=f"Joint {i} ({self.joint_info[i]['name']}) Force: {force_mag:.4f} N"
                    )
                    self.torque_labels[i].config(
                        text=f"Joint {i} ({self.joint_info[i]['name']}) Torque: {torque_mag:.4f} Nm"
                    )
            
            # Step simulation
            p.stepSimulation()
            
            # Schedule next update
            self.root.after(10, self.simulation_loop)

def main():
    """Main function to run the application"""
    try:
        calculator = TorqueCalculator()
        calculator.run_simulation()
    except KeyboardInterrupt:
        print("\nApplication terminated by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        p.disconnect()

if __name__ == "__main__":
    main()