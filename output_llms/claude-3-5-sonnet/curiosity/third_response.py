import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens  # Added sensor module import

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body with contact material and add it to the system
ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.5))  # Position the ground slightly below the origin
ground.SetBodyFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 20, 20)
system.Add(ground)

# create a long box for rover to cross
box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, ground_mat)
box.SetPos(chrono.ChVectorD(0, 0, 0.0))
box.SetBodyFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)

# Create Curiosity rover and add it to the system
rover = robot.Curiosity(system)

# Create driver for rover
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

# Initialize rover position and orientation
init_pos = chrono.ChVectorD(-5, 0.0, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 3), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0), 3, 4, 10, 40, 512)

# Create a sensor manager
manager = sens.ChSensorManager(system)

# Create a lidar sensor
lidar_update_rate = 5.0
lidar_horizontal_samples = 360
lidar_vertical_samples = 16
lidar_min_vert_angle = -0.26  # In radians
lidar_max_vert_angle = 0.26   # In radians
lidar_max_distance = 100.0
lidar_noise = 0.01  # Standard deviation of noise [m]

offset_pose = chrono.ChFrameD(chrono.ChVectorD(0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0))

lidar = sens.ChLidarSensor(
    rover.GetChassisBody(),  # body lidar is attached to
    lidar_update_rate,       # scanning rate in Hz
    offset_pose,             # offset pose relative to body
    lidar_horizontal_samples,  # number of horizontal samples
    lidar_vertical_samples,  # number of vertical channels
    chrono.CH_C_2PI,         # horizontal field of view
    lidar_min_vert_angle,    # vertical field of view bottom
    lidar_max_vert_angle,    # vertical field of view top
    lidar_max_distance       # max lidar range
)

lidar.SetName("Lidar Sensor")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.0)

# Add noise to the lidar
lidar_noise_model = sens.ChLidarNoiseNormal(lidar_noise)
lidar.SetNoiseModel(lidar_noise_model)

# Add lidar to the sensor manager
manager.AddSensor(lidar)

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    time += time_step

    # ask rover to move forward
    driver.SetThrottle(0.5)
    driver.SetSteering(0.0)

    # Update rover dynamics
    rover.Update()

    # Update sensor manager
    manager.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)