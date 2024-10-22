import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
import numpy as np
import math

# Initial vehicle location and orientation
initLoc = ch.ChVector3d(0, 0, 0.5)
initRot = ch.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = ch.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = ch.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# ---------------
# Create systems
# ---------------

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()  # veh.HMMWV_Reduced() could also be used here
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(ch.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.SetShaftLock(True)
vehicle.SetSteeringLock(steering_lock)
vehicle.SetTireStepSize(tire_step_size)
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type, vis_type)

vehicle.GetSystem().SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = ch.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    ch.ChCoordsysd(ch.ChVector3d(0, 0, 0), ch.QUNIT), 
    terrainLength, terrainWidth)

patch.SetTexture("", 200, 200)
patch.SetColor(ch.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle ROS manager
ros_manager = chros.ChROSVehicleManager(vehicle.GetSystem())
ros_manager.RegisterHandler(chros.ChROSVehicleManager.CLOCK, chros.ChROSVehicleClockHandler())
ros_manager.RegisterHandler(chros.ChROSVehicleManager.DRIVER_INPUTS, chros.ChROSVehicleDriverInputsHandler())
ros_manager.RegisterHandler(chros.ChROSVehicleManager.VEHICLE_STATE, chros.ChROSVehicleVehicleStateHandler())
ros_manager.Initialize()

# Create the driver system
driver = veh.ChInteractiveDriverIRB(vehicle.GetChassisBody())
driver.Initialize()

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.SetLookAheadDistance(3.0)

# ---------------
# Simulation loop
# ---------------

# output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between two render frames
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
realtime_timer = ch.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while (ros_manager.Run()) :
    time = vehicle.GetSystem().GetChTime()

    # Render the scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        # Update vehicle ROS manager
        ros_manager.Update()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    ros_manager.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    ros_manager.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin the real-time loop
    realtime_timer.Spin(step_size)