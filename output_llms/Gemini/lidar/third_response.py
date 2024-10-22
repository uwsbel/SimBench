import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.assets as ass
import pychrono.irr as irr

import numpy as np
import time
import math

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

def main():
    # ----------------------
    # Create the ARTcar vehicle
    # ----------------------
    system = chrono.ChSystemNSC()
    vehicle = veh.Vehicle(system, "ARTcar", "vehicle/artcar/json/artcar.json")
    vehicle.Initialize(chrono.ChCoordsD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))

    # ----------------
    # Create a driver
    # ----------------
    driver = veh.ChDriver(vehicle)
    driver.Initialize()

    # ---------------------------------
    # Create the terrain and its assets
    # ---------------------------------
    terrain = veh.RigidTerrain(system)
    patch = terrain.AddPatch(
        ass.ChBoxShape(500, 1, 500), chrono.ChVectorD(0, 0, -5), chrono.QUNIT, 10000
    )
    patch.SetContactFrictionCoefficient(0.9)
    patch.SetContactRestitutionCoefficient(0.01)
    patch.SetContactRollingFrictionCoefficient(0.01)
    patch.GetGroundBody().SetName("ground")
    patch.GetGroundBody().GetVisualModel().GetShape(0).SetTexture(
        chrono.GetChronoDataFile("textures/concrete.jpg")
    )
    patch.GetGroundBody().GetVisualModel().GetShape(0).SetTextureScale(200, 200)
    terrain.Initialize()

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(system)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    # Attach lidar to the vehicle chassis
    chassis_body = vehicle.GetChassisBody()
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVectorD(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        chassis_body,           # Body lidar is attached to
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

    # Create 2D lidar sensor
    lidar_2d = sens.ChLidarSensor(
        chassis_body,           # Body lidar is attached to
        update_rate,            # Scanning rate in Hz
        offset_pose,            # Offset pose
        horizontal_samples,     # Number of horizontal samples
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
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        # Don't add any noise models
        pass
    if vis:
        # Visualize the raw lidar data
        lidar_2d.PushFilter(
            sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data")
        )
    # Provides the host access to the Depth, Intensity data
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    # Convert Depth, Intensity data to XYZI point cloud data
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    # Provides the host access to the XYZI data
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    # Add the lidar to the sensor manager
    manager.AddSensor(lidar_2d)

    # -------------------------------------------------------
    # Create a camera and add it to the sensor manager
    # -------------------------------------------------------
    # Create the camera
    camera = sens.ChCameraSensor(
        chassis_body,
        update_rate,
        chrono.ChFrameD(chrono.ChVectorD(-3, 0, 3), chrono.Q_from_AngAxis(0.2, chrono.ChVectorD(0, 1, 0))),
        640,
        480,
    )
    camera.SetName("Camera Sensor")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)
    # Provides the host access to the RGB data
    camera.PushFilter(sens.ChFilterRGBAccess())
    # Add the camera to the sensor manager
    manager.AddSensor(camera)

    # ---------------
    # Simulate system
    # ---------------
    myapplication = irr.ChIrrApp(
        system, "Sensor Demonstration", irr.dimension2du(1280, 720), False, True
    )
    myapplication.SetTimestep(step_size)
    myapplication.SetTryRealtime(True)
    myapplication.SetBackground(
        chrono.ChColor(0.1, 0.1, 0.1), chrono.ChColor(0.9, 0.9, 0.9), True
    )
    myapplication.AssetBindAll()
    myapplication.AssetUpdateAll()
    myapplication.AddTypicalLights()
    myapplication.AddTypicalCamera(
        irr.vector3df(0.0, 4.0, -6.0), irr.vector3df(0, 1, 0)
    )

    ch_time = 0.0
    render_time = 0
    t1 = time.time()
    while ch_time < end_time:
        # Collect output data from sensors
        manager.Update(system.GetChTime())

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules
        driver.Synchronize(ch_time)
        terrain.Synchronize(ch_time)
        vehicle.Synchronize(ch_time, driver_inputs, terrain)
        # Advance driver state
        driver.Advance(step_size)
        # Advance system state
        system.DoStepDynamics(step_size)

        # Render scene
        myapplication.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        myapplication.DrawAll()
        myapplication.EndScene()

        # Get the current time of the simulation
        ch_time = system.GetChTime()

        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print(
                "XYZI buffer received from lidar. Lidar resolution: {0}x{1}".format(
                    xyzi_buffer.Width, xyzi_buffer.Height
                )
            )
            print("Max Value: {0}".format(np.max(xyzi_data)))

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


if __name__ == "__main__":
    main()