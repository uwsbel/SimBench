import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# ---------------------------------------------------------------------
#  Create the simulation system and set the default
#  material properties for rigid bodies
# ---------------------------------------------------------------------

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-20, 0, 1.5) # Modified: Changed initial location
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# ---------------------------------------------------------------------
#  Create the MAN vehicle, set parameters, and initialize
# ---------------------------------------------------------------------

vehicle = veh.MAN_5t() # Modified: Changed vehicle type to MAN_5t
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

# ---------------------------------------------------------------------
#  Create the terrain
# ---------------------------------------------------------------------

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(patch_mat, 
                        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
                        200, 200)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200) # Modified: Changed terrain texture to "grass.jpg"
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# Create the terrain with hills
height_map = chrono.ChHeightField()
height_map.SetFileName(chrono.GetChronoDataPath() + 'height_maps/test64.bmp')

# Set the size of the terrain patch
terrain_size_x = 100.0
terrain_size_y = 100.0

# Calculate the height scaling factor based on the height map range
height_range = height_map.GetHeightRange()
height_scale = 5.0 / height_range

# Create the terrain patch
terrain_patch = terrain.AddPatch(patch_mat, 
                                chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                                terrain_size_x, terrain_size_y,
                                0.1, height_scale) 

# Set the height map for the terrain patch
terrain_patch.SetHeightField(height_map)
terrain_patch.SetTexture(chrono.GetChronoDataPath() + "textures/grass.jpg")

terrain.Initialize()

# ---------------------------------------------------------------------
#  Create the vehicle Irrlicht interface
# ---------------------------------------------------------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 5t Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(-3.0, 0.0, 1.7), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


# ---------------------------------------------------------------------
#  Create the driver system
# ---------------------------------------------------------------------

driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# ---------------------------------------------------------------------
#  Simulation loop
# ---------------------------------------------------------------------

# output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
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