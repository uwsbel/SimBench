import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle locations and orientations
initLoc1 = chrono.ChVector3d(0, 0, 0.5)
initRot1 = chrono.ChQuaternionD(1, 0, 0, 0)
initLoc2 = chrono.ChVector3d(5, 0, 0.5)  # Second vehicle position
initRot2 = chrono.ChQuaternionD(1, 0, 0, 0)  # Second vehicle orientation

vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

contact_method = chrono.ChContactMethod_NSC
contact_vis = False

step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50

# Create the first Sedan vehicle
vehicle1 = veh.BMW_E90()
vehicle1.SetContactMethod(contact_method)
vehicle1.SetChassisCollisionType(chassis_collision_type)
vehicle1.SetChassisFixed(False)
vehicle1.SetInitPosition(chrono.ChCoordsysD(initLoc1, initRot1))
vehicle1.SetTireType(tire_model)
vehicle1.SetTireStepSize(tire_step_size)
vehicle1.Initialize()

vehicle1.SetChassisVisualizationType(vis_type)
vehicle1.SetSuspensionVisualizationType(vis_type)
vehicle1.SetSteeringVisualizationType(vis_type)
vehicle1.SetWheelVisualizationType(vis_type)
vehicle1.SetTireVisualizationType(vis_type)

# Create the second Sedan vehicle
vehicle2 = veh.BMW_E90()
vehicle2.SetContactMethod(contact_method)
vehicle2.SetChassisCollisionType(chassis_collision_type)
vehicle2.SetChassisFixed(False)
vehicle2.SetInitPosition(chrono.ChCoordsysD(initLoc2, initRot2))
vehicle2.SetTireType(tire_model)
vehicle2.SetTireStepSize(tire_step_size)
vehicle2.Initialize()

vehicle2.SetChassisVisualizationType(vis_type)
vehicle2.SetSuspensionVisualizationType(vis_type)
vehicle2.SetSteeringVisualizationType(vis_type)
vehicle2.SetWheelVisualizationType(vis_type)
vehicle2.SetTireVisualizationType(vis_type)

# Create the terrain
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle1.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle1.GetVehicle())

# Create the driver systems
driver1 = veh.ChInteractiveDriverIRR(vis)
driver2 = veh.ChDriver(vehicle2.GetVehicle())

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver1.SetSteeringDelta(render_step_size / steering_time)
driver1.SetThrottleDelta(render_step_size / throttle_time)
driver1.SetBrakingDelta(render_step_size / braking_time)

driver1.Initialize()
driver2.Initialize()

# Simulation loop
render_steps = math.ceil(render_step_size / step_size)

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle1.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs1 = driver1.GetInputs()

    # Sinusoidal steering input for both vehicles
    steering_input = 0.5 * math.sin(0.5 * time)
    driver_inputs2 = veh.DriverInputs()
    driver_inputs2.m_steering = steering_input
    driver_inputs2.m_throttle = 0.5
    driver_inputs2.m_braking = 0

    # Update modules
    driver1.Synchronize(time)
    driver2.Synchronize(time)
    terrain.Synchronize(time)
    vehicle1.Synchronize(time, driver_inputs1, terrain)
    vehicle2.Synchronize(time, driver_inputs2, terrain)
    vis.Synchronize(time, driver_inputs1)

    # Advance simulation
    driver1.Advance(step_size)
    driver2.Advance(step_size)
    terrain.Advance(step_size)
    vehicle1.Advance(step_size)
    vehicle2.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)