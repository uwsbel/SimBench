import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as vehicle

import numpy as np
import time
import math


def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # ----------------------------------
    # Create and initialize an ARTcar vehicle
    # ----------------------------------
    artcar = vehicle.ChVehicle("ARTcar")
    artcar.Initialize(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))
    mphysicalSystem.Add(artcar)

    # ----------------------------------
    # Add terrain
    # ----------------------------------
    terrain = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True)
    terrain.SetPos(chrono.ChVectorD(0, -0.5, 0))
    terrain.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
    terrain.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/ground_texture.png"))
    mphysicalSystem.Add(terrain)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        artcar.GetChassis(),  # Body lidar is attached to the vehicle chassis
        update_rate,          # Scanning rate in Hz
        offset_pose,          # Offset pose
        horizontal_samples,   # Number of horizontal samples
        vertical_samples,     # Number of vertical channels
        horizontal_fov,       # Horizontal field of view
        max_vert_angle,       # Maximum vertical field of view
        min_vert_angle,       # Minimum vertical field of view
        100.0,                # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,        # Sample radius
        divergence_angle,     # Divergence angle
        divergence_angle,     # Divergence angle (again, typically same value)
        return_mode           # Return mode for the lidar
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the lidar
    # -----------------------------------------------------------------
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        # Don't add any noise models
        pass
    if vis:
        # Visualize the raw lidar data
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    # Provides the host access to the Depth, Intensity data
    lidar.PushFilter(sens.ChFilterDIAccess())
    # Convert Depth, Intensity data to XYZI point cloud data
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        # Visualize the point cloud
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    # Provides the host access to the XYZI data
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    # Add the lidar to the sensor manager
    manager.AddSensor(lidar)
    
    # Create 2D lidar sensor
    lidar_2d = sens.ChLidarSensor(
        artcar.GetChassis(),  # Body lidar is attached to the vehicle chassis
        update_rate,          # Scanning rate in Hz
        offset_pose,          # Offset pose
        horizontal_samples,   # Number of horizontal samples
        1,                    # only 1 vertical channel for 2D lidar
        horizontal_fov,       # Horizontal field of view
        0.0,                  # Maximum vertical field of view
        0.0,                  # Minimum vertical field of view
        100.0,                # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,        # Sample radius
        divergence_angle,     # Divergence angle
        divergence_angle,     # Divergence angle (again, typically same value)
        return_mode           # Return mode for the lidar
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        # Don't add any noise models
        pass
    if vis:
        # Visualize the raw lidar data
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 2D Lidar Depth Data"))
    # Provides the host access to the Depth, Intensity data
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    # Convert Depth, Intensity data to XYZI point cloud data
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    # Provides the host access to the XYZI data
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    # Add the lidar to the sensor manager
    manager.AddSensor(lidar_2d)

    # Add a third-person camera sensor
    camera_offset = chrono.ChVectorD(5, 2, 1)
    camera = sens.ChCameraSensor(
        artcar.GetChassis(),
        update_rate,
        camera_offset,
        640,  # Width
        480,  # Height
        0.1,  # Near plane
        100.0 # Far plane
    )
    camera.SetName("Third Person Camera")
    manager.AddSensor(camera)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Synchronize vehicle and driver
        artcar.Update(ch_time)
        
        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Update sensor manager (will render/save/filter automatically)
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


# -----------------
# Lidar parameters
# -----------------

# Noise model attached to the sensor
noise_model = "NONE"  # No noise model

# Lidar return mode
return_mode = sens.LidarReturnMode_STRONGEST_RETURN

# Update rate in Hz
update_rate = 5.0

# Number of horizontal and vertical samples
horizontal_samples = 800
vertical_samples = 300

# Horizontal and vertical field of view (radians)
horizontal_fov = 2 * chrono.CH_PI  # 360 degrees
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6

# Lag time
lag = 0

# Collection window for the lidar
collection_time = 1. / update_rate  # typically 1/update rate

# Radius of samples to use, 1->1 sample, 2->9 samples, 3->25 samples...
sample_radius = 2

# 3mm radius (as cited by velodyne)
divergence_angle = 0.003

# ---------------------
# Simulation parameters
# ---------------------

# Simulation step size
step_size = 1e-3

# Simulation end time
end_time = 40.0

# Save camera images
save = False

# Render camera images
vis = True

# Output directory
out_dir = "SENSOR_OUTPUT/"

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with:
# chrono.SetChronoDataPath('path/to/data')

main()