"""
Modified PyChrono script for Viper rover simulation with SCM deformable terrain
"""

import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in the negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create SCM deformable terrain
terrain_length = 20
terrain_width = 20
terrain_mesh_resolution = 0.1
soil_density = 1500
soil_youngs_modulus = 1e7
soil_damping_ratio = 0.1
soil_friction_angle = math.radians(30)

terrain = chrono.ChBodyEasyBox(system, terrain_length, terrain_width, 1, soil_density, True, True)
terrain.SetPos(chrono.ChVector3d(0, 0, -1))  # Position the terrain slightly below the origin
terrain.SetFixed(True)  # Fix the terrain in place

# Create Viper rover
rover = viper.Viper(system)  # Instantiate the Viper rover
driver = viper.ViperDCMotorControl()  # Create a driver for the rover
rover.SetDriver(driver)  # Assign the driver to the rover

# Initialize rover position and orientation
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - SCM deformable terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Enable shadows (commented out to improve performance)
# vis.EnableShadows()

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    time += time_step  # Increment the simulation time
    steering = 0.0  # Set steering to a constant value of 0.0

    driver.SetSteering(steering)  # Set the steering for the rover

    rover.Update()  # Update the rover's state

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)