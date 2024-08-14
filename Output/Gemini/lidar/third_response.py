import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh

import numpy as np
import time
import math

# -----------------
# Global parameters
# -----------------

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with:
# chrono.SetChronoDataPath('path/to/data')
chrono.SetChronoDataPath(veh.GetDataPath())

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


def main():
    # -----------------
    # Create the system
    # -----------------
    # Create an ARTcar vehicle
    vehicle = veh.ARTcar()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5)))
    vehicle.SetTireType(veh.Tire.Type_RIGID)
    vehicle.SetTireStepSize(step_size)
    vehicle.Initialize()

    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Use the vehicle system as the main physical system
    mphysicalSystem = vehicle.GetSystem()

    # ----------------------------------
    # Add a mesh to be sensed by a lidar
    # ----------------------------------
    #create box for camera to sense
    side = 4
    box = chrono.ChBodyEasyBox(side, side, side, 1000)
    box.SetPos(chrono.ChVectorD(0, 0, 0))
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    box.SetBodyFixed(True)
    mphysicalSystem.Add(box)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------

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

    # Changed lidar offset pose
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(1.0, 0, 1), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # Body lidar is attached to (Changed to vehicle chassis)
        update_rate,  # Scanning rate in Hz
        offset_pose,  # Offset pose
        horizontal_samples,  # Number of horizontal samples
        vertical_samples,  # Number of vertical channels
        horizontal_fov,  # Horizontal field of view
        max_vert_angle,  # Maximum vertical field of view
        min_vert_angle,  # Minimum vertical field of view
        100.0,  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,  # Sample radius
        divergence_angle,  # Divergence angle
        divergence_angle,  # Divergence angle (again, typically same value)
        return_mode  # Return mode for the lidar
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
        vehicle.GetChassisBody(),  # Body lidar is attached to (Changed to vehicle chassis)
        update_rate,  # Scanning rate in Hz
        offset_pose,  # Offset pose
        horizontal_samples,  # Number of horizontal samples
        1,  # only 1 vertical channel for 2D lidar
        horizontal_fov,  # Horizontal field of view
        0.0,  # Maximum vertical field of view
        0.0,  # Minimum vertical field of view
        100.0,  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,  # Sample radius
        divergence_angle,  # Divergence angle
        divergence_angle,  # Divergence angle (again, typically same value)
        return_mode  # Return mode for the lidar
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

    # ----------------------------------
    # Create a third person view camera
    # ----------------------------------
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  # Body the camera is attached to
        10,  # Update rate in Hz
        chrono.ChFrameD(chrono.ChVectorD(-10, 0, 2), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),  # Offset pose
        1280,  # Image width
        720,  # Image height
        chrono.CH_C_PI / 3  # Field of view
    )
    camera.SetName("Third Person View Camera")
    camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person View"))  # Visualization filter
    if save:
        camera.PushFilter(sens.ChFilterSavePNG(out_dir + "camera/"))  # Save PNG images filter
    manager.AddSensor(camera)

    # -------------------------------
    # Create the driver system
    # -------------------------------
    driver = veh.ChDriver(vehicle)
    # Driver inputs are always generated inside this function
    driver.Initialize()

    # -------------------------
    # Create the terrain
    # -------------------------
    terrain = veh.RigidTerrain(mphysicalSystem)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.1)
    patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(-50, -50, 0), chrono.ChVectorD(50, 50, 0), 0.01)
    patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"), 20, 20)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # ---------------
    # Simulate system
    # ---------------
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # ----------------------
        # Update the vehicle
        # ----------------------
        vehicle.Update(ch_time)

        # ----------------------
        # Update the driver
        # ----------------------
        driver.Synchronize(ch_time)
        driver_inputs = driver.GetInputs()

        # -----------------
        # Update the terrain
        # -----------------
        terrain.Synchronize(ch_time)

        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width,
                                                                                     xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Update sensor manager (will render/save/filter automatically)
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


if __name__ == "__main__":
    main()