import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 200.0  # size in X direction (increased from 100.0)
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
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

# Create the circular path
path_radius = 10.0
path_center = chrono.ChVector3d(0, 0, 0)

# Visualize the path using two balls
path_vis_sphere1 = chrono.ChSphereShape(0.2)
path_vis_sphere2 = chrono.ChSphereShape(0.2)
path_vis_body1 = chrono.ChBodyEasySphere(0.2, 1000, False, True)
path_vis_body2 = chrono.ChBodyEasySphere(0.2, 1000, False, True)
path_vis_body1.SetPos(path_center + chrono.ChVector3d(path_radius, 0, 0.2))
path_vis_body2.SetPos(path_center - chrono.ChVector3d(path_radius, 0, 0.2))
path_vis_body1.SetBodyFixed(True)
path_vis_body2.SetBodyFixed(True)
path_vis_body1.GetCollisionModel().ClearModel()
path_vis_body2.GetCollisionModel().ClearModel()
path_vis_body1.AddAsset(path_vis_sphere1)
path_vis_body2.AddAsset(path_vis_sphere2)
vehicle.GetSystem().Add(path_vis_body1)
vehicle.GetSystem().Add(path_vis_body2)

# Create the path follower
path_follower = veh.ChPathFollower(vehicle.GetVehicle(), path_center, path_radius)
path_follower.Initialize()

# Set the constant throttle value and PID controller gains for steering control
throttle_value = 0.3
steering_gain = 0.5
proportional_gain = 1.0
integral_gain = 0.1
derivative_gain = 0.05
path_follower.GetSteeringController().SetGains(proportional_gain, integral_gain, derivative_gain)
path_follower.GetSteeringController().SetSteeringFunction(veh.ChPathFollower.STEERING_PID)
path_follower.GetSpeedController().SetDesiredSpeed(10.0)

# output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
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

    # Update path follower
    path_follower.Advance(step_size)

    # Update modules (process inputs from other modules)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, path_follower.GetInputs(), terrain)
    vis.Synchronize(time, path_follower.GetInputs())

    # Advance simulation for one timestep for all modules
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)