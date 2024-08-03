import pybullet as p
import time
import pybullet_data
import math

# Connect to PyBullet and set up the simulation
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Load the ground plane and the quadruped robot URDF
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 0.5]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("quadruped.urdf", startPos, startOrientation)

# Define the joint indices for the four legs
leg_joint_indices = [0, 1, 2, 3]  # Assuming joint indices are in sequence

# Simulation parameters
timestep = 1. / 240.  # Simulation timestep in seconds
max_time = 10.0  # Maximum simulation time in seconds
start_time = time.time()

# Simulate the forward motion
while time.time() - start_time < max_time:
    sim_time = time.time() - start_time  # Current simulation time
    angle = math.sin(sim_time * 4.0) * math.radians(30.0)
    joint_angles = [angle, -angle, angle, -angle]
    for i in range(4):
        # Calculate joint angle based on sine function for cyclic motion
        angle = joint_angles[i]

        p.setJointMotorControl2(robotId, leg_joint_indices[i], p.POSITION_CONTROL, targetPosition=angle)

    p.stepSimulation()
    time.sleep(timestep)

# Disconnect from the simulation
p.disconnect()
