import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Adjusted ground position
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

# Create sensor manager
manager = sens.ChSensorManager(system)
system.Add(manager)

# Configure and add lidar sensor
lidar = sens.ChLidarSensor(
    system,
    100,  # scanning rate
    chrono.ChFrameD(chrono.ChVector3d(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)),  # offset pose
    0.1,  # max distance
    0.05,  # min distance
    0.01,  # accuracy
    0.1,  # horizontal FOV
    0.1  # vertical FOV
)
manager.AddSensor(lidar)

# Add filters to lidar sensor
lidar.AddFilter(sens.ChFilterShadows())
lidar.AddFilter(sens.ChFilterNoise(0.05))

# Create randomly placed boxes
for _ in range(5):
    box_mat = chrono.ChContactMaterialNSC()
    box = chrono.ChBodyEasyBox(
        np.random.uniform(0.5, 2),  # length
        np.random.uniform(0.5, 2),  # width
        np.random.uniform(0.5, 2),  # height
        100,  # density
        True,  # collide
        True,  # visualize
        box_mat
    )
    box.SetPos(chrono.ChVector3d(
        np.random.uniform(-5, 5),
        np.random.uniform(-5, 5),
        np.random.uniform(0, 2)
    ))
    system.Add(box)

# Define motion control function for Turtlebot
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(2, 0)
        robot.SetMotorSpeed(2, 1)
    elif mode == 'left':
        robot.SetMotorSpeed(-math.pi, 0)
        robot.SetMotorSpeed(math.pi, 1)
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi, 0)
        robot.SetMotorSpeed(-math.pi, 1)

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Enable shadows (commented out to improve performance)
# vis.EnableShadows()

# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
while vis.Run():
    # Move Turtlebot
    move('straight')

    # Update sensor manager
    manager.Update()

    # Increment time counter
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)