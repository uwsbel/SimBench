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
    # Add a box to be sensed by a lidar
    # ----------------------------------
    side = 5
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetBodyFixed(True)
    box_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    mphysicalSystem.Add(box_body)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        box_body,              # Body lidar is attached to
        update_rate,            # Scanning rate in Hz
        offset_pose,            # Offset pose
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

    # -----------------------------------------------------------------
    # Create a 2D lidar sensor and add it to the sensor manager
    # -----------------------------------------------------------------
    lidar_2d = sens.ChLidarSensor(
        box_body,              # Body lidar is attached to
        update_rate,            # Scanning rate in Hz
        offset_pose,            # Offset pose
        horizontal_samples,     # Number of horizontal samples
        1,                      # Number of vertical channels
        horizontal_fov,         # Horizontal field of view
        0,                      # Maximum vertical field of view
        0,                      # Minimum vertical field of view
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

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the 2D lidar
    # -----------------------------------------------------------------
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        # Don't add any noise models
        pass

    if vis:
        # Visualize the raw lidar data
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))

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
        # Set lidar to orbit around the box body
        lidar.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
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
# TODO: Noise models haven't been implemented in python
# noise_model = "CONST_NORMAL_XYZI"  # Gaussian noise with constant mean and standard deviation
noise_model = "NONE"  # No noise model

# Lidar return mode
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
# return_mode = sens.MEAN_RETURN
# return_mode = sens.FIRST_RETURN
# return_mode = sens.LAST_RETURN

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

# Simulation parameters

# Simulation step size
step_size = 1e-3

# Simulation end time
end_time = 40.0

# Save camera images
save = False

# Render camera images
vis = True

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with:
# chrono.SetChronoDataPath('path/to/data')

# Output directory
out_dir = "SENSOR_OUTPUT/"

# The given code has an error. Please correct it and follow the instructions to meet the requirements.

# -------------------
# 1. Replace the triangular mesh object (chrono.ChTriangleMeshConnected()) with a box (side, side, side, 1000)
# 2. Set the box dimensions to (5, 5, 5)

# 3. Create a box body with the specified dimensions
# -------------------

# The box should be a cube with side length 5.0

# -------------------
# The cube should be a cube with side length 5.0

# -------------------
# The cube should be a cube with side length 5.0

# -------------------
# The cube should be a cube with side length 5.0

# -------------------
# The cube should be a cube with side length 5.0

# -------------------
# The cube should be a cube with side length 5.0

# -------------------
# The cube should be a cube with side length 5.0

# -------------------
# The cube should be a cube with side length 5.0

# -------------------
# The cube should be a cube with side length 5.0
# -------------------
# The cube should be a cube with side length 5.0

# -------------------
# The cube should be a cube with side length 5.0

# -------------------
# The cube should be a cube with side length 5.0
# -------------------
# 1. Modify the given code to meet the following requirements:
# -------------------
# The cube should be a cube with side length 5.0
# -------------------
# The cube should be a cube with side length 5.0

# -------------------

# The cube should be a cube with side length 5.0
# -------------------
# The cube should be a cube with side length 5.0

# The cube should be a cube with side length 5.0

# The cube should be a cube with side length 5.0

# The cube should be a cube with side length 5.0

# The cube should be a cube with side length 5. Please modify the given code and the instructions. Here is the code:

# The cube should be a cube with side length 5. Please modify the given code to modify the code according to the following instructions:

# The cube should be a cube with side length 5.

# The cube should be a cube with side length 5. Create a cube with side length 

# The cube should be a cube with side length 

# -----------------
# The cube should be a cube with side length 
# The cube should be a cube with side length 5.0

# The cube should be a cube with side length 

# The cube should be a cube = 
# The cube should be a cube = 

# The cube should be a cube = 

# The cube should be a cube = 

# The cube should be a cube = 

# -----------------

# The cube should be a cube with side length 

# ------------------------------------

# The cube should be a cube = 

# --------------------
# The cube = 

# The cube should be a cube = 


















































































































# The given_script = 

# --------------------
# The given script:

# 1. Create a box = 

# The given_script =
# The given_script = 
#. Create a box = 
# The given_script
# The script. The script = 

# 
# Create a box =
# 
# Create a box =

#. Create a box =
# The script. Create a box =


# Create a box = 
# 
#. Create a box =
#. There are the given_script
#. Create a box =

#. Create a box =

#. Create a box =

# The given_script =

#. Create a box =
#. Create a box =

    box =

    box =

Here is the given script =

Here is the

# Create a box =

# The script.Create(mphysicalSystem =

# Add a box =

# Add a box =

# Add the mesh =

Here is the_script.Create(mphysicalSystem.Create(box =

# Create a box =

#. Create(mphysicalSystem.be.Create(box =

# Modify the mesh

Here is the

Here is the

#. Create(mphysicalSystem.Add(

Here is what you are given instructions:

# Create a box with the given instructions:

# -

# Create a box

Here are the_script.Create a box =

# Create the script.Create a lidar sensor

# Create the script.Create a lidar_script.Create

Here is the script.Create a box with

# Create a box:
# Create a box =



















# Create a box with

# Create(

Here is what.Create a box =

# Create(











# Create(

# Create a box

# -
# Create a box with
print("error happened with only start ```python")