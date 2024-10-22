"""
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import matplotlib.pyplot as plt

# Set the Chrono data directory
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0      # Terrain height
terrainLength = 100.0  # Size in X direction
terrainWidth = 100.0   # Size in Y direction

# Point on the chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50
log_step_size = 1.0 / 20    # Frequency of data logging

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full() # veh.HMMWV_Reduced() could be another choice
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Set visualization types for vehicle parts
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system type
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs
steering_time = 1.0  # Time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # Time to go from 0 to +1
braking_time = 0.3   # Time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Initialize sensor manager
manager = sens.ChSensorManager(vehicle.GetSystem())

# Create an IMU sensor and add it to the manager
offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
imu = sens.ChAccelerometerSensor(vehicle.GetChassisBody(),                     # Body IMU is attached to
                                 10,        # Update rate in Hz
                                 offset_pose,          # Offset pose
                                 sens.ChNoiseNone())   # Noise model
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
# Provides the host access to the IMU data
imu.PushFilter(sens.ChFilterAccelAccess())
# Add the IMU to the sensor manager
manager.AddSensor(imu)

# Create a GPS sensor and add it to the manager
gps = sens.ChGPSSensor(vehicle.GetChassisBody(),                     # Body GPS is attached to
                       10,        # Update rate in Hz
                       offset_pose,          # Offset pose
                       chrono.ChVector3d(-89.400, 43.070, 260.0),  # GPS reference point
                       sens.ChNoiseNone())   # Noise model
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
# Provides the host access to the GPS data
gps.PushFilter(sens.ChFilterGPSAccess())
# Add the GPS to the sensor manager
manager.AddSensor(gps)

# ---------------
# Simulation loop
# ---------------

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)
log_steps = math.ceil(log_step_size / step_size)
# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

gps_data = []
# Simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    if step_number % log_steps == 0:
        # get most recent GPS data
        gps_coor = gps.GetMostRecentGPSBuffer().GetGPSData()
        gps_data.append([gps_coor[0], gps_coor[1], gps_coor[2]])
    # Set driver inputs
    driver.SetThrottle(0.5)
    driver.SetSteering(0.6)
    driver_inputs = driver.GetInputs()
    
    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Update sensor manager in each step
    manager.Update()
    
    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)
print("GPS Data: ", gps_data)

# Plot GPS data
plt.figure(figsize=(10, 10))
plt.plot([x[0] for x in gps_data], [x[1] for x in gps_data])
plt.xlabel('Latitude')
plt.ylabel('Longitude')
plt.title('GPS Trajectory')
plt.show()