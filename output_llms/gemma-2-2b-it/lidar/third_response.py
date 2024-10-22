import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math

# --- System Initialization ---
# Create ARTcar vehicle system
mphysicalSystem = chrono.ChSystemNSC()

# --- Vehicle Parameters ---
car_mass = 1000.0  # kg
car_length = 4.0  # m
car_width = 1.5  # m
car_height = 1.0  # m
car_center_of_mass = chrono.ChVector3d(0.0, 0.5, 0.0)  # m
car_wheel_base = 2.0  # m

# --- Terrain ---
terrain_material = chrono.ChMaterial("terrain", 
                                     density=1000.0, 
                                     friction=0.5, 
                                     restitution=0.0)
terrain_texture = chrono.GetChronoDataFile("textures/terrain.png")
terrain = chrono.ChRigidBody(chrono.ChVector3d(0, 0, 0), terrain_material, terrain_texture)
terrain.SetPos(chrono.ChVector3d(0, 0, 0))
terrain.SetFixed(True)

# --- Lidar Sensors ---
# 3D Lidar
lidar_3d = sens.ChLidarSensor(
    terrain,  # Body lidar is attached to
    update_rate,            # Scanning rate in Hz
    chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    ),
    horizontal_samples,     # Number of horizontal samples
    vertical_samples,       # Number of vertical channels
    horizontal_fov,         # Horizontal field of view
    max_vert_angle,         # Maximum vertical field of view
    min_vert_angle,         # Minimum vertical field of view
    100.0,                  # Maximum lidar range
    sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
    sample_radius,          # Sample radius
    divergence_angle,       # Divergence angle
    divergence_angle,       # Divergence angle (again, typically same value)
    return_mode             # Return mode for the lidar
)
lidar_3d.SetName("3D Lidar Sensor")
lidar_3d.SetLag(lag)
lidar_3d.SetCollectionWindow(collection_time)

# 2D Lidar
lidar_2d = sens.ChLidarSensor(
    terrain,  # Body lidar is attached to
    update_rate,            # Scanning rate in Hz
    chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    ),
    1,                      # only 1 vertical channel for 2D lidar
    horizontal_fov,         # Horizontal field of view
    0.0,                    # Maximum vertical field of view
    0.0,                    # Minimum vertical field of view
    100.0,                  # Maximum lidar range
    sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
    sample_radius,          # Sample radius
    divergence_angle,       # Divergence angle
    divergence_angle,       # Divergence angle (again, typically same value)
    return_mode             # Return mode for the lidar
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(lag)
lidar_2d.SetCollectionWindow(collection_time)

# --- Vehicle Driver ---
driver = chrono.ChVehicleDriver(mphysicalSystem, car_mass, car_length, car_width, car_height, car_center_of_mass, car_wheel_base)
driver.SetSpeed(10.0)  # m/s

# --- Third Person Camera ---
camera = sens.ChCameraSensor(
    mphysicalSystem,
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)
camera.SetName("Third Person Camera")
camera.SetLag(lag)
camera.SetCollectionWindow(collection_time)

# --- Simulation Loop ---
t1 = time.time()
ch_time = 0.0
while ch_time < end_time:
    # Update vehicle and driver
    driver.Update()
    mphysicalSystem.DoStepDynamics(step_size)

    # Update sensor manager
    manager.Update()

    # Update camera
    camera.Update()

    # Update lidar sensors
    lidar_3d.Update()
    lidar_2d.Update()

    # Access the XYZI buffer from lidar
    xyzi_buffer = lidar_3d.GetMostRecentXYZIBuffer()
    if xyzi_buffer.HasData():
        xyzi_data = xyzi_buffer.GetXYZIData()
        print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
        print('Max Value: {0}'.format(np.max(xyzi_data)))

    # Render camera images
    if vis:
        camera.Render()

    # Perform step of dynamics
    mphysicalSystem.DoStepDynamics(step_size)

    # Get the current time of the simulation
    ch_time = mphysicalSystem.GetChTime()

    # Print simulation time
    print("Sim time:", end_time, "Wall time:", time.time() - t1)