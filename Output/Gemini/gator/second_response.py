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
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
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

# Create the Gator vehicle, set parameters, and initialize
vehicle = veh.Gator()
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
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Define a function to add a terrain patch with options for bump and height map
def add_terrain_patch(terrain, material, position, length, width, texture_file=None, bump_position=None, bump_radius=None, bump_height=None, height_map_file=None):
    patch = terrain.AddPatch(material, 
        chrono.ChCoordsysd(position, chrono.QUNIT), 
        length, width)
    if texture_file:
        patch.SetTexture(veh.GetDataFile(texture_file), 200, 200)
    if bump_position:
        patch.AddBump(chrono.ChCoordsysd(bump_position, chrono.QUNIT), bump_radius, bump_height)
    if height_map_file:
        patch.SetHeightMap(veh.GetDataFile(height_map_file), length, width, 0, 0.2)  # Example height range
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    return patch

# Shared material properties
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Patch 1: Flat with texture
patch1 = add_terrain_patch(terrain, patch_mat, chrono.ChVector3d(0, 0, terrainHeight), terrainLength/2, terrainWidth/2, texture_file="terrain/textures/tile4.jpg")

# Patch 2: Flat with different texture
patch2 = add_terrain_patch(terrain, patch_mat, chrono.ChVector3d(terrainLength/2, 0, terrainHeight), terrainLength/2, terrainWidth/2, texture_file="terrain/textures/grass.jpg")

# Patch 3: Flat with another different texture
patch3 = add_terrain_patch(terrain, patch_mat, chrono.ChVector3d(0, terrainWidth/2, terrainHeight), terrainLength/2, terrainWidth/2, texture_file="terrain/textures/stone.jpg")

# Patch 4: Height map with bump
patch4 = add_terrain_patch(terrain, patch_mat, chrono.ChVector3d(terrainLength/2, terrainWidth/2, terrainHeight), terrainLength/2, terrainWidth/2, height_map_file="terrain/height_maps/test64.bmp", bump_position=chrono.ChVector3d(terrainLength/4, terrainWidth/4, 0), bump_radius=5, bump_height=0.3)

terrain.Initialize()

# -------------------------------------
# Create the vehicle Irrlicht interface
# Create the driver system
# -------------------------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator vehicle')
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
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# ---------------
# Simulation loop
# ---------------

# output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
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

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)