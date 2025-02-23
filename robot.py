import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt
from math import pi
import keyboard

class CylindricalRobot:
    def __init__(self):
        # Initialize PyBullet
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load environment and robot
        self.plane = p.loadURDF("plane.urdf")
        self.robot = p.loadURDF(r"C:\Users\kumar\OneDrive\Desktop\task\robot_Arm.urdf", 
                               basePosition=[0, 0, 0])
        
        # Create table and box (now at the back)
        self.createTable()
        self.createBox()
        
        # Joint indices
        self.base_joint = 0      # Revolute
        self.lift_joint = 1      # Prismatic (vertical)
        self.extend_joint = 2    # Prismatic (horizontal)
        
        # Initialize lists for data collection
        self.torques_data = []
        self.positions_data = []
        
        # Add debug parameters
        self.params = [
            p.addUserDebugParameter("Base Rotation", -pi/2, pi/2, 0),
            p.addUserDebugParameter("Vertical Lift", -2, 0, 0),
            p.addUserDebugParameter("Horizontal Extend", 0, 0.8, 0)
        ]
        
        # Add start button
        self.start_button = p.addUserDebugParameter("Start Task", 1, 0, 0)
        self.last_start = 0

    def createTable(self):
        """Create a table for the box at the back"""
        self.table_height = 1.0
        # Place table behind the robot (negative Y)
        table_position = [0, -1, self.table_height]  # Changed position to back
        
        table_visual = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[0.4, 0.4, 0.02],
            rgbaColor=[0.8, 0.6, 0.4, 1]
        )
        table_collision = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[0.4, 0.4, 0.02]
        )
        self.table = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=table_collision,
            baseVisualShapeIndex=table_visual,
            basePosition=table_position
        )
        
        # Create table legs
        leg_size = [0.02, 0.02, self.table_height/2]
        leg_positions = [
            [table_position[0]-0.35, table_position[1]-0.35, self.table_height/2],
            [table_position[0]+0.35, table_position[1]-0.35, self.table_height/2],
            [table_position[0]-0.35, table_position[1]+0.35, self.table_height/2],
            [table_position[0]+0.35, table_position[1]+0.35, self.table_height/2]
        ]
        
        for pos in leg_positions:
            leg_visual = p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=leg_size,
                rgbaColor=[0.8, 0.6, 0.4, 1]
            )
            leg_collision = p.createCollisionShape(
                shapeType=p.GEOM_BOX,
                halfExtents=leg_size
            )
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=leg_collision,
                baseVisualShapeIndex=leg_visual,
                basePosition=pos
            )
        
    def createBox(self):
        """Create a box to be manipulated"""
        self.box_mass = 20
        self.box_size = 0.3
        # Place box on the table at the back
        box_position = [0, -1, self.table_height + self.box_size/2]  # Aligned with table
        
        visual_shape = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[self.box_size/2]*3,
            rgbaColor=[1, 0, 0, 1]
        )
        collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[self.box_size/2]*3
        )
        self.box = p.createMultiBody(
            baseMass=self.box_mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=box_position
        )
    
    def inverse_kinematics(self, target_pos):
        """Simple analytical IK for cylindrical robot"""
        x, y, z = target_pos
        
        # Calculate joint angles/positions
        base_angle = np.arctan2(y, x)
        extension = np.sqrt(x**2 + y**2) - 0.5  # Subtract base radius
        lift = z - 0.5  # Subtract base height
        
        return [base_angle, lift, extension]
    
    def move_to_position(self, target_pos, steps=50, force_multiplier=1.0):
        """Move to position with linear interpolation"""
        current_joints = [p.getJointState(self.robot, i)[0] for i in range(3)]
        target_joints = self.inverse_kinematics(target_pos)
        
        # Generate trajectory
        trajectory = []
        for i in range(steps):
            t = i / (steps - 1)
            interp_joints = [current_joints[j] + t * (target_joints[j] - current_joints[j]) 
                           for j in range(3)]
            trajectory.append(interp_joints)
        
        # Execute trajectory
        base_forces = [400, 500, 300]
        forces = [f * force_multiplier for f in base_forces]
        
        for joints in trajectory:
            p.setJointMotorControlArray(
                bodyUniqueId=self.robot,
                jointIndices=[self.base_joint, self.lift_joint, self.extend_joint],
                controlMode=p.POSITION_CONTROL,
                targetPositions=joints,
                forces=forces
            )
            
            p.stepSimulation()
            
            # Collect data
            joint_states = p.getJointStates(self.robot, [0, 1, 2])
            self.torques_data.append([state[3] for state in joint_states])
            self.positions_data.append(joints)
            
            time.sleep(0.02)
    
    def attach_box(self):
        """Create constraint to attach box to gripper"""
        gripper_pos, _ = p.getBasePositionAndOrientation(self.box)
        self.constraint = p.createConstraint(
            parentBodyUniqueId=self.robot,
            parentLinkIndex=3,  # gripper link
            childBodyUniqueId=self.box,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0]
        )
    
    def detach_box(self):
        """Remove constraint to release box"""
        p.removeConstraint(self.constraint)
    
    def execute_pick_and_place(self):
        """Execute complete pick and place sequence"""
        # Clear previous data
        self.torques_data = []
        self.positions_data = []
        
        try:
            # Pick sequence (from back table)
            print("Moving to initial position...")
            self.move_to_position([0, -0.8, 1.5])  # Move to safe position
            
            print("Approaching box...")
            # Move above box
            self.move_to_position([0, -1, self.table_height + self.box_size + 0.1])
            # Lower to box
            self.move_to_position([0, -1, self.table_height + self.box_size/2 - 0.1], steps=50)
            
            print("Gripping box...")
            self.attach_box()
            
            print("Lifting box...")
            # Lift box with increased force for heavy load
            self.move_to_position([0, -1, self.table_height + self.box_size + 0.3], steps=50, force_multiplier=1.5)
            
            # Move to front placement position
            print("Moving to front...")
            placement_x = 0.0
            placement_y = 1.0  # Front position
            
            # Move to position above placement spot
            self.move_to_position([placement_x, placement_y, 1.0], steps=70)
            
            # Lower all the way down very slowly
            print("Lowering to ground...")
            # First lower to intermediate position
            self.move_to_position([placement_x, placement_y, 0.5], steps=100)
            
            # Then go all the way down to ground
            self.move_to_position([placement_x, placement_y, self.box_size/2], steps=150, force_multiplier=0.7)
            
            print("Placing box...")
            # Release box
            self.detach_box()
            
            # Retract carefully
            print("Retracting...")
            # First move slightly up
            self.move_to_position([placement_x, placement_y, 0.3], steps=50)
            # Then move to safe position
            self.move_to_position([0, 0, 1.0], steps=70)
            
            print("Task completed successfully")
            
            # Plot results
            self.plot_data()
            
        except Exception as e:
            print(f"Error during pick and place operation: {str(e)}")
            try:
                self.detach_box()
            except:
                pass
    
    def plot_data(self):
        """Plot collected torque and position data"""
        torques = np.array(self.torques_data)
        positions = np.array(self.positions_data)
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
        
        # Plot torques
        ax1.plot(torques[:, 0], label="Base Torque (Nm)")
        ax1.plot(torques[:, 1], label="Lift Force (N)")
        ax1.plot(torques[:, 2], label="Extend Force (N)")
        ax1.set_title("Joint Torques/Forces During Motion")
        ax1.set_xlabel("Time Steps")
        ax1.set_ylabel("Torque/Force")
        ax1.grid(True)
        ax1.legend()
        
        # Plot positions
        ax2.plot(positions[:, 0], label="Base Angle (rad)")
        ax2.plot(positions[:, 1], label="Lift Position (m)")
        ax2.plot(positions[:, 2], label="Extension (m)")
        ax2.set_title("Joint Positions During Motion")
        ax2.set_xlabel("Time Steps")
        ax2.set_ylabel("Position")
        ax2.grid(True)
        ax2.legend()
        
        plt.tight_layout()
        plt.show()
    
    def run(self):
        """Main run loop"""
        print("Press 'Start Task' button to begin pick and place operation")
        try:
            while True:
                # Check if start button pressed
                start_val = p.readUserDebugParameter(self.start_button)
                if start_val != self.last_start:
                    self.last_start = start_val
                    self.execute_pick_and_place()
                
                # Manual joint control through sliders
                for i, param in enumerate(self.params):
                    p.setJointMotorControl2(
                        bodyUniqueId=self.robot,
                        jointIndex=i,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=p.readUserDebugParameter(param)
                    )
                
                p.stepSimulation()
                time.sleep(0.02)
                
        except KeyboardInterrupt:
            print("\nShutting down gracefully...")
        except Exception as e:
            print(f"Error in main loop: {str(e)}")

def main():
    """Main entry point"""
    try:
        robot = CylindricalRobot()
        robot.run()
    except Exception as e:
        print(f"Fatal error: {str(e)}")
    finally:
        p.disconnect()
        print("PyBullet disconnected")

if __name__ == "__main__":
    main()