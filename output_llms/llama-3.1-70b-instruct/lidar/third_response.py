"""
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

import numpy as np
import time
import math

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # Create the vehicle
    vehicle = veh.ChVehicle(mphysicalSystem)

    # Set the vehicle parameters
    vehicle.SetVehicleFile("path/to/vehicle/file")
    vehicle.SetInitPosition(chrono.ChVector3d(0, 0, 0))
    vehicle.SetInitOrientation(chrono.ChQuaternion(1, 0, 0, 0))

    # Initialize the vehicle system
    vehicle.Initialize()

    # Create a driver for the vehicle
    driver = veh.ChIrrGuiDriver(vehicle)
    driver.SetSteeringMode(veh.SteeringMode_TANK)

    # Create a rigid terrain
    terrain = chrono.ChBodyEasyBox(mphysicalSystem, 100, 100, 10, 1000)
    terrain.SetPos(chrono.ChVector3d(0, -10, 0))
    terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
    terrain.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/floor.jpg"))
    terrain.GetVisualModel().GetShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    mphysicalSystem.Add(terrain)

    # Create a sensor manager
    manager = sens.ChSensorManager(mphysicalSystem)

    # Create a lidar sensor
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # Body lidar is attached to
        5.0,            # Scanning rate in Hz
        offset_pose,            # Offset pose
        800,     # Number of horizontal samples
        300,       # Number of vertical channels
        2 * chrono.CH_PI,         # Horizontal field of view
        chrono.CH_PI / 12,         # Maximum vertical field of view
        -chrono.CH_PI / 6,         # Minimum vertical field of view
        100.0,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        2,          # Sample radius
        0.003,       # Divergence angle
        0.003,       # Divergence angle (again, typically same value)
        sens.LidarReturnMode_STRONGEST_RETURN             # Return mode for the lidar
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(0)
    lidar.SetCollectionWindow(1. / 5.0)  # typically 1/update rate

    # Create a filter graph for post-processing the data from the lidar
    lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Raw Lidar Depth Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    # Create a 2D lidar sensor
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # Body lidar is attached to
        5.0,            # Scanning rate in Hz
        offset_pose,            # Offset pose
        800,     # Number of horizontal samples
        1,       # Number of vertical channels
        2 * chrono.CH_PI,         # Horizontal field of view
        0.0,         # Maximum vertical field of view
        0.0,         # Minimum vertical field of view
        100.0,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        2,          # Sample radius
        0.003,       # Divergence angle
        0.003,       # Divergence angle (again, typically same value)
        sens.LidarReturnMode_STRONGEST_RETURN             # Return mode for the lidar
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(0)
    lidar_2d.SetCollectionWindow(1. / 5.0)  # typically 1/update rate
    lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    lidar_2d.PushFilter(sens.ChFilterVisualize(800, 1, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    # Create a third person camera
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  # Body camera is attached to
        30.0,            # Frame rate in Hz
        chrono.ChFramed(
            chrono.ChVector3d(0, -5, 2), chrono.QuatFromAngleAxis(chrono.CH_PI / 4, chrono.ChVector3d(0, 1, 0))
        ),            # Offset pose
        640,     # Horizontal resolution
        480,       # Vertical resolution
        60.0,         # Field of view
        0.1,         # Near plane
        100.0,                  # Far plane
        sens.CameraSensorMode_RGB             # Camera mode
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(0)
    camera.SetCollectionWindow(1. / 30.0)  # typically 1/frame rate
    manager.AddSensor(camera)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < 40.0:
        # Synchronize the modules
        vehicle.Synchronize(chrono.ChTime(ch_time))
        driver.Synchronize(chrono.ChTime(ch_time))
        terrain.Synchronize(chrono.ChTime(ch_time))

        # Advance the modules
        vehicle.Advance(chrono.ChTime(ch_time))
        driver.Advance(chrono.ChTime(ch_time))
        terrain.Advance(chrono.ChTime(ch_time))

        # Update the sensor manager
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(1e-3)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", 40.0, "Wall time:", time.time() - t1)

main()