import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.irrlicht as chronoirr
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
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Position the ground slightly below the origin
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create sensor manager
sensor_manager = chrono.ChSensorManager()
system.AddSensorManager(sensor_manager)

# Configure lidar sensor
lidar_sensor = sens.ChLidarSensor()
lidar_sensor.SetRange(10)
lidar_sensor.SetResolution(0.1)
lidar_sensor.SetFrequency(10)
lidar_sensor.SetPosition(chrono.ChVector3d(0.2, 0, 0.1))
lidar_sensor.SetOrientation(chrono.Q_from_AngX(math.pi / 2))
lidar_sensor.SetRangeFilter(sens.ChLidarSensor.RangeFilter_LOG)
lidar_sensor.SetReturnMode(sens.ChLidarSensor.ReturnMode_ALL)
lidar_sensor.SetNoise(0.001)
sensor_manager.AddSensor(lidar_sensor)

# Create 5 randomly placed boxes
for i in range(5):
    box_shape = chrono.ChBox(0.5, 0.5, 0.5)
    box_mat = chrono.ChMaterialSurfaceNSC()
    box_body = chrono.ChBody(box_shape, box_mat, chrono.ChVectorD(0, 0, 0))
    box_body.SetPos(chrono.ChVectorD(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0))
    system.AddBody(box_body)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

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


def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, 0)
        robot.SetMotorSpeed(math.pi, 1)
    elif mode == 'left':
        robot.SetMotorSpeed(math.pi, 0)
        robot.SetMotorSpeed(0, 1)
    elif mode == 'right':
        robot.SetMotorSpeed(0, 0)
        robot.SetMotorSpeed(math.pi, 1)


# Simulation loop
time = 0
while vis.Run():
    move('straight')
    sensor_manager.DoStepDynamics(time_step)

    # Increment time counter
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)