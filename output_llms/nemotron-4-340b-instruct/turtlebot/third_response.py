import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Position the ground slightly below the origin
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

# Create sensor manager and configure lidar sensor
sensor_manager = sens.ChSensorManager(system)
lidar_sensor = sens.ChLidarSensor(
    robot.GetChassisBody(),  # Body to attach the sensor to
    chrono.ChFrameD(chrono.ChVector3d(0, 0, 0.2), chrono.ChQuaterniond(1, 0, 0, 0)),  # Sensor position and orientation
    0.1,  # Time step
    1000,  # Number of points
    0.01,  # Min range
    10,  # Max range
    0.01,  # Angular resolution
    0.01,  # Noise std deviation
    True,  # Use intensity
    True,  # Use shadowing
)
lidar_sensor.SetName("lidar")
lidar_sensor.SetLag(0.0)
lidar_sensor.SetFilter(sens.ChFilterPCfromRaycast(0.01))  # Set filter to remove points closer than 0.01 m
sensor_manager.AddSensor(lidar_sensor)

# Create randomly placed boxes
num_boxes = 5
box_size = chrono.ChVector3d(0.5, 0.5, 0.5)
box_mass = 10
for i in range(num_boxes):
    box_pos = chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0.5)
    box = chrono.ChBodyEasyBox(box_size.x, box_size.y, box_size.z, box_mass, True, True, ground_mat)
    box.SetPos(box_pos)
    system.Add(box)

# Motion control function for Turtlebot
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi / 2, turtlebot.TurtleBot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi / 2, turtlebot.TurtleBot.RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, turtlebot.TurtleBot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, turtlebot.TurtleBot.RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-math.pi, turtlebot.TurtleBot.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, turtlebot.TurtleBot.RIGHT_DRIVE_WHEEL)

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
    # Move the Turtlebot
    move('straight')

    # Update sensor manager
    sensor_manager.Update()

    # Increment time counter
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)