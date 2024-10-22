import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math


def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # ----------------------------------
    # Create a box to be sensed by a lidar
    # ----------------------------------
    side = 1.0
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000, True, True)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetPose(chrono.ChQuaternion(1, 0, 0, 0))
    box_body.SetCollide(True)
    box_body.SetMaterialSurface(chrono.ChMaterialSurface.BoxMat)
    mphysicalSystem.Add(box_body)

    # Create a visual representation of the box
    box_shape = chrono.ChBoxShape()
    box_shape.SetBox(chrono.ChVectorD(side / 2, side / 2, side / 2))
    box_shape.SetTexture(chrono.GetChronoDataFile("textures/checker.png"))
    box_body.AddVisualShape(box_shape)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-side / 2 - 0.1, 0, 0.1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        box_body,              # Body lidar is attached to
        5.0,                   # Scanning rate in Hz
        offset_pose,            # Offset pose
        800,                  # Number of horizontal samples
        1,                    # Number of vertical channels
        2 * chrono.CH_PI,      # Horizontal field of view
        0,                    # Maximum vertical angle
        0,                    # Minimum vertical angle
        100.0,                # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        0.01,                 # Sample radius
        0.003,               # Divergence angle
        0.003,               # Divergence angle (again, typically same value)
        sens.LidarReturnMode_STRONGEST_RETURN
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(0)
    lidar.SetCollectionWindow(1 / 5.0)  # typically 1/update rate

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
        lidar.PushFilter(sens.ChFilterVisualize(800, 1, "Raw Lidar Depth Data"))

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

    # ------------------------------------------------
    # Create a 2D lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose_2d = chrono.ChFramed(
        chrono.ChVector3d(0, -side / 2 - 0.1, 0.1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar_2d = sens.ChLidarSensor2D(
        box_body,              # Body lidar is attached to
        5.0,                   # Scanning rate in Hz
        offset_pose_2d,            # Offset pose
        800,                  # Number of horizontal samples
        1,                    # Number of vertical channels
        2 * chrono.CH_PI,      # Horizontal field of view
        0,                    # Maximum vertical angle
        0,                    # Minimum vertical angle
        100.0,                # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        0.01,                 # Sample radius
        0.003,               # Divergence angle
        0.003,               # Divergence angle (again, typically same value)
        sens.LidarReturnMode_STRONGEST_RETURN
    )
    lidar_2d.SetName("Lidar Sensor 2D")
    lidar_2d.SetLag(0)
    lidar_2d.SetCollectionWindow(1 / 5.0)  # typically 1/update rate

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the 2D lidar
    # -----------------------------------------------------------------
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        # Don't add any noise models
        pass

    if vis:
        # Visualize the raw 2D lidar data
        lidar_2d.PushFilter(sens.ChFilterVisualize(800, 1, "Raw 2D Lidar Depth Data"))

    # Provides the host access to the Depth, Intensity data
    lidar_2d.PushFilter(sens.ChFilterDIAccess())

    # Convert Depth, Intensity data to XYZI point cloud data
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        # Visualize the point cloud
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))

    # Provides the host access to the XYZI data
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    # Add the 2D lidar to the sensor manager
    manager.AddSensor(lidar_2d)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Set lidar to orbit around the box
        lidar.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    0.1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )

        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Set 2D lidar to orbit around the box
        lidar_2d.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    0,
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate) + 0.1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )

        # Access the XYZI buffer from 2D lidar
        xyzi_buffer_2d = lidar_2d.GetMostRecentXYZIBuffer()
        if xyzi_buffer_2d.HasData():
            xyzi_data_2d = xyzi_buffer_2d.GetXYZIData()
            print('XYZI buffer received from 2D lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer_2d.Width, xyzi_buffer_2d.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data_2d)))

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


main()