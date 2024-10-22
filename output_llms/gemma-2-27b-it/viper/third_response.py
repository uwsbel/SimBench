import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
from pychrono import irrlicht as chronoirr
import pychrono.vehicle as veh

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create SCM deformable terrain
terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChPlane(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 1)))
terrain.SetLength(20)
terrain.SetWidth(20)
terrain.SetMeshResolution(100)
terrain.SetSoilParameters(
    veh.SCMDeformableTerrain.SoilParameters(
        friction=0.5,
        restitution=0.2,
        young_modulus=1e7,
        poisson_ratio=0.3,
        density=1500,
        yield_stress=1e5,
        plastic_strain_limit=0.1,
    )
)

# Create Viper rover
rover = viper.Viper(system)
driver = viper.ViperDCMotorControl()
rover.SetDriver(driver)

# Initialize rover position and orientation
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - Deformable terrain')
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
    time += time_step
    steering = 0.0  # Set steering to a constant value

    driver.SetSteering(steering)

    rover.Update()

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)