import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initTruckLoc = chrono.ChVector3d(0, 0, 0.5)
initTruckRot = chrono.ChQuaterniond(1, 0, 0, 0)
initSedanLoc = chrono.ChVector3d(10, 0, 0.5)
initSedanRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


truck_tire_model = veh.TireModelType_RIGID
sedan_tire_model = veh.TireModelType_TMEASY


terrain_model = veh.RigidTerrain.MESH
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0
terrainFile = "terrain/highway.obj"  


trackPoint = chrono.ChVector3d(0, 0, 2.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(initTruckLoc, initTruckRot))
truck.Initialize()

truck.SetChassisVisualizationType(vis_type, vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)

truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(initSedanLoc, initSedanRot))
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type, vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)

sedan.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())
patch = terrain.AddPatchFromMesh(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainFile, terrainLength, terrainWidth)

patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())
vis.AttachVehicle(sedan.GetTractor())


truck_driver = veh.ChInteractiveDriverIRR(vis)
sedan_driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
truck_driver.SetSteeringDelta(render_step_size / steering_time)
truck_driver.SetThrottleDelta(render_step_size / throttle_time)
truck_driver.SetBrakingDelta(render_step_size / braking_time)

sedan_driver.SetSteeringDelta(render_step_size / steering_time)
sedan_driver.SetThrottleDelta(render_step_size / throttle_time)
sedan_driver.SetBrakingDelta(render_step_size / braking_time)

truck_driver.Initialize()
sedan_driver.Initialize()


truck_state = {}


sedan_driver.SetThrottle(1.0)
sedan_driver.SetSteering(0.0)


print( "VEHICLE MASS: ",  truck.GetTractor().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = truck.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    truck_inputs = truck_driver.GetInputs()
    sedan_inputs = sedan_driver.GetInputs()

    
    truck_driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, truck_inputs, terrain)
    sedan_driver.Synchronize(time)
    terrain.Synchronize(time)
    sedan.Synchronize(time, sedan_inputs, terrain)
    vis.Synchronize(time, truck_inputs, sedan_inputs)

    
    truck_driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan_driver.Advance(step_size)
    terrain.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)