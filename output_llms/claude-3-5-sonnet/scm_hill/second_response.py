import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import numpy as np

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(-15, 0, 1.2)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Rigid terrain
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

# Point tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 20  # FPS = 50

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Create the SCM deformable terrain patch
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6,   # Bekker Kphi
                          0,     # Bekker Kc
                          1.1,   # Bekker n exponent
                          0,     # Mohr cohesive limit (Pa)
                          30,    # Mohr friction limit (degrees)
                          0.01,  # Janosi shear coefficient (m)
                          2e8,   # Elastic stiffness (Pa/m), before plastic yield
                          3e4    # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

# Optionally, enable moving patch feature (single patch around vehicle chassis)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(5, 3, 1))

# Set plot type for SCM (false color plotting)
terrain.SetPlotType(veh.SCMTerrain.PlotType_SINKAGE, 0, 0.1)

# Initialize the SCM terrain
terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"), 40, 40, -1, 1, 0.02)

terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

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

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Create obstacles
for _ in range(5):
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
    box.SetPos(chrono.ChVectorD(np.random.uniform(-20, 20), np.random.uniform(-20, 20), 0.5))
    vehicle.GetSystem().Add(box)

# Create the sensor manager
manager = sens.ChSensorManager(vehicle.GetSystem())

# Create a lidar sensor
offset_pose = chrono.ChFrameD(chrono.ChVectorD(0, 0, 2), chrono.ChQuaternionD(1, 0, 0, 0))
lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),  # body lidar is attached to
    20,                        # scanning rate in Hz
    offset_pose,               # offset pose
    1000,                      # number of horizontal samples
    16,                        # number of vertical channels
    chrono.CH_C_PI,            # horizontal field of view
    chrono.CH_C_PI / 12,       # vertical field of view
    100                        # max distance
)

# Add filters to the lidar
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterVisualize(640, 480, "Lidar"))

# Add the lidar to the sensor manager
manager.AddSensor(lidar)

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if step_number % math.ceil(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    manager.Update()

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)