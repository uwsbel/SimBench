import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# ---------------------------------------------------------------------
# Modify the following parameters to customize the vehicle simulation -
# ---------------------------------------------------------------------

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
terrainLength = 200.0  # size in X direction (Increased as per instructions)
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

# ---------------------------------------------------------------------
#  End of parameter customization
# ---------------------------------------------------------------------


print( "Copyright (c) 2017 projectchrono.org\n")

# --------------
# Create systems
# --------------

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

# Create the terrain
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
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

# ------------------------
# Create the driver system
# ------------------------

# Create a circular path
path_radius = 40
path_center = chrono.ChVector3d(0, path_radius, terrainHeight + 1)

# Visualization for the path
path_asset_1 = chrono.ChSphereShape()
path_asset_1.GetSphereGeometry().rad = 0.2
path_asset_2 = chrono.ChSphereShape()
path_asset_2.GetSphereGeometry().rad = 0.2
ball_1 = chrono.ChAsset()
ball_1.AddShape(path_asset_1)
ball_2 = chrono.ChAsset()
ball_2.AddShape(path_asset_2)
vis.BindItem(vis.AddAsset(ball_1, chrono.ChFrameD(path_center + chrono.ChVector3d(path_radius, 0, 0))))
vis.BindItem(vis.AddAsset(ball_2, chrono.ChFrameD(path_center + chrono.ChVector3d(-path_radius, 0, 0))))

class PathFollowerDriver:
    def __init__(self, vehicle, path_center, path_radius, target_speed, look_ahead_distance=5.0):
        self.vehicle = vehicle
        self.path_center = path_center
        self.path_radius = path_radius
        self.target_speed = target_speed
        self.look_ahead_distance = look_ahead_distance
        self.throttle = 0.3  # Constant throttle value (as per instructions)
        self.steering = 0.0
        self.Kp = 0.5  # PID gain for steering
        self.Ki = 0.05
        self.Kd = 0.1
        self.error_integral = 0
        self.previous_error = 0
        self.target_point = chrono.ChVector3d(0, 0, 0)
        self.sentinel_point = chrono.ChVector3d(0, 0, 0)

        # Visualization for the sentinel and target points
        self.target_asset = chrono.ChSphereShape()
        self.target_asset.GetSphereGeometry().rad = 0.2
        self.target_ball = chrono.ChAsset()
        self.target_ball.AddShape(self.target_asset)
        self.target_vis_id = vis.BindItem(vis.AddAsset(self.target_ball, chrono.ChFrameD(self.target_point)))

        self.sentinel_asset = chrono.ChSphereShape()
        self.sentinel_asset.GetSphereGeometry().rad = 0.2
        self.sentinel_ball = chrono.ChAsset()
        self.sentinel_ball.AddShape(self.sentinel_asset)
        self.sentinel_vis_id = vis.BindItem(vis.AddAsset(self.sentinel_ball, chrono.ChFrameD(self.sentinel_point)))

    def GetInputs(self):
        # Calculate steering angle using PID control
        self.calculate_steering()

        return veh.DriverInputs(self.throttle, self.steering, 0.0)  # Brake is always 0 for now

    def Synchronize(self, time):
        # Calculate target point on the path
        vehicle_pos = self.vehicle.GetVehicle().GetPos()
        closest_point = self.path_center + (vehicle_pos - self.path_center).GetNormalized() * self.path_radius
        self.sentinel_point = closest_point + (closest_point - self.path_center).GetNormalized() * self.look_ahead_distance

        # Update visualization
        vis.UpdateAssetPosition(self.target_vis_id, chrono.ChFrameD(self.target_point))
        vis.UpdateAssetPosition(self.sentinel_vis_id, chrono.ChFrameD(self.sentinel_point))

    def Advance(self, step):
        pass  # Not used in this implementation

    def calculate_steering(self):
        # PID control for steering
        vehicle_pos = self.vehicle.GetVehicle().GetPos()
        vehicle_dir = self.vehicle.GetVehicle().GetRot().Rotate(chrono.ChVector3d(1, 0, 0))
        error = (self.sentinel_point - vehicle_pos).Cross(vehicle_dir).z()
        self.error_integral += error * step_size
        error_derivative = (error - self.previous_error) / step_size
        self.steering = - (self.Kp * error + self.Ki * self.error_integral + self.Kd * error_derivative)
        self.previous_error = error

driver = PathFollowerDriver(vehicle, path_center, path_radius, 5)

# ---------------------------------------------------------------------

# ---------------
# Simulation loop
# ---------------

# output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
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

    # Get driver inputs
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

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)