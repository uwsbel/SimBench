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
terrainHeight = 0      
terrainLength = 200.0  
terrainWidth = 100.0   

# Path parameters
path_radius = 50.0
path_center = chrono.ChVector3d(0, 0, 0)

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
vehicle = veh.HMMWV_Full() # veh.HMMWV_Reduced()  could be another choice here
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
patch = terrain.AddPatch(patch_mat,chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),terrainLength, terrainWidth)
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

# PID controller parameters
kp = 1.0
ki = 0.1
kd = 0.01

# Initialize PID controller
pid = chrono.ChPID()
pid.SetKp(kp)
pid.SetKi(ki)
pid.SetKd(kd)

# Path follower
class PathFollower:
    def __init__(self, vehicle, path_radius, path_center):
        self.vehicle = vehicle
        self.path_radius = path_radius
        self.path_center = path_center
        self.target_angle = 0.0

    def update(self, time):
        # Calculate target angle based on vehicle position
        vehicle_pos = self.vehicle.GetChassis().GetPos()
        distance_to_center = (vehicle_pos - self.path_center).Length()
        target_angle = math.atan2(vehicle_pos.y - self.path_center.y, vehicle_pos.x - self.path_center.x)

        # Apply PID control to steering
        error = target_angle - self.vehicle.GetSteeringAngle()
        pid.Compute(error, time)
        steering_angle = pid.GetOutput()

        # Set throttle and steering
        self.vehicle.SetThrottle(0.3)
        self.vehicle.SetSteeringAngle(steering_angle)

# Create path follower instance
path_follower = PathFollower(vehicle, path_radius, path_center)

# Visualization of path points
sentinel_sphere = chrono.ChBodyEasySphere(0.1)
sentinel_sphere.SetPos(path_center)
sentinel_sphere.SetBodyFixed(True)
vehicle.GetSystem().Add(sentinel_sphere)

target_sphere = chrono.ChBodyEasySphere(0.1)
target_sphere.SetPos(path_center + chrono.ChVector3d(path_radius, 0, 0))
target_sphere.SetBodyFixed(True)
vehicle.GetSystem().Add(target_sphere)

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update path follower
    path_follower.update(time)

    # Advance simulation for one timestep for all modules
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)