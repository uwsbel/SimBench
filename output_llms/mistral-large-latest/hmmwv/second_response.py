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
terrainLength = 200.0  # increased size in X direction
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
vehicle = veh.HMMWV_Full()  # veh.HMMWV_Reduced() could be another choice here
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

# Path and Controller Implementation
radius = 10.0  # radius of the circular path
center = chrono.ChVector3d(0, 0, 0)

# Function to calculate the point on the circular path
def circular_path(angle):
    return chrono.ChVector3d(center.x + radius * math.cos(angle),
                             center.y + radius * math.sin(angle),
                             center.z)

# PID Controller for steering
pid = veh.ChPIDController()
pid.Set_Kp(0.1)
pid.Set_Ki(0.0)
pid.Set_Kd(0.0)

# Visualization of the path using two balls
ball1 = chrono.ChBodyEasySphere(0.1, 1000, True, True, chrono.ChMaterialSurfaceNSC())
ball1.SetPos(circular_path(0))
ball1.SetBodyFixed(True)
vehicle.GetSystem().Add(ball1)

ball2 = chrono.ChBodyEasySphere(0.1, 1000, True, True, chrono.ChMaterialSurfaceNSC())
ball2.SetPos(circular_path(math.pi))
ball2.SetBodyFixed(True)
vehicle.GetSystem().Add(ball2)

# Sentinel and Target points for visualization
sentinel = chrono.ChBodyEasySphere(0.1, 1000, True, True, chrono.ChMaterialSurfaceNSC())
sentinel.SetPos(initLoc)
sentinel.SetBodyFixed(True)
vehicle.GetSystem().Add(sentinel)

target = chrono.ChBodyEasySphere(0.1, 1000, True, True, chrono.ChMaterialSurfaceNSC())
target.SetPos(circular_path(0))
target.SetBodyFixed(True)
vehicle.GetSystem().Add(target)

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0
angle = 0.0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update the target point position
    angle += step_size / (2 * math.pi * radius)
    target_pos = circular_path(angle)
    target.SetPos(target_pos)

    # Calculate steering using PID controller
    vehicle_pos = vehicle.GetVehicle().GetPos()
    vehicle_dir = vehicle.GetVehicle().GetRot().GetYaxis()
    error = (target_pos - vehicle_pos).Cross(vehicle_dir).z
    steering = pid.Compute(error, step_size)

    # Set constant throttle and steering
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = 0.3
    driver_inputs.m_steering = steering

    # Update modules (process inputs from other modules)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)