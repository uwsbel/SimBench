"""
Import necessary modules
"""
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import math

"""
Set the Chrono data path
"""
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Rigid terrain
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full() # veh.HMMWV_Reduced()  could be another choice here
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

# Ensure the vehicle is not initialized within the vehicle
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Add objects to the scene
for i in range(5):
    box = chrono.ChBodyEasyBox(2, 2, 2, 1000, True, True, chrono.ChContactMaterialSMC())
    box.SetPos(chrono.ChVector3d(np.random.uniform(-5, 10), np.random.uniform(2, 10), 1))
    box.SetFixed(True)
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    # Ensure the box is not initialized within the vehicle
    if box.GetPos().Distance(vehicle.GetChassisBody().GetPos()) > 5:
        vehicle.GetSystem().Add(box)

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
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))

# Set plot type for SCM (false color plotting)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

# Initialize the SCM terrain (length, width, mesh resolution), specifying the initial mesh grid
terrain.Initialize(20, 20, 0.02)

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

# Create a sensor manager
manager = sens.ChSensorManager(vehicle.GetSystem())
intensity = 0.1
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)

# Create a camera sensor attached to the vehicle chassis
offset_pose = chrono.ChFramed(chrono.ChVector3d(.7, 0, 1.2), chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)))
update_rate = 30
image_width = 1280
image_height = 720
fov = 1.047
cam = sens.ChCameraSensor(
    vehicle.GetChassisBody(),
    update_rate,
    offset_pose,
    image_width,
    image_height,
    fov
)
cam.SetName("First Person POV")

# Renders the image at current point in the filter graph
if vis:
    cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale Filter"))

# Add the camera sensor to the sensor manager
manager.AddSensor(cam)

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Simulation loop
# output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
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