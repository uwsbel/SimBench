import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


truck_initLoc = chrono.ChVector3d(0, 0, 0.5)
truck_initRot = chrono.ChQuaterniond(1, 0, 0, 0)
sedan_initLoc = chrono.ChVector3d(10, 0, 0.5)
sedan_initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


truck_tire_model = veh.TireModelType_RIGID
sedan_tire_model = veh.TireModelType_TMEASY


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   
terrain_mesh = veh.GetDataFile("terrain/meshes/highway.obj")


truck_trackPoint = chrono.ChVector3d(0,0, 2.1)
sedan_trackPoint = chrono.ChVector3d(10,0, 2.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(truck_initLoc, truck_initRot))
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
sedan.SetInitPosition(chrono.ChCoordsysd(sedan_initLoc, sedan_initRot))
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
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth, terrain_mesh)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


truck_vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
truck_vis.SetWindowTitle('Truck Demo')
truck_vis.SetWindowSize(1280, 1024)
truck_vis.SetChaseCamera(truck_trackPoint, 25.0, 1.5)
truck_vis.Initialize()
truck_vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
truck_vis.AddLightDirectional()
truck_vis.AddSkyBox()
truck_vis.AttachVehicle(truck.GetTractor())

sedan_vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
sedan_vis.SetWindowTitle('Sedan Demo')
sedan_vis.SetWindowSize(1280, 1024)
sedan_vis.SetChaseCamera(sedan_trackPoint, 25.0, 1.5)
sedan_vis.Initialize()
sedan_vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
sedan_vis.AddLightDirectional()
sedan_vis.AddSkyBox()
sedan_vis.AttachVehicle(sedan.GetTractor())


truck_driver = veh.ChInteractiveDriverIRR(truck_vis)
sedan_driver = veh.ChInteractiveDriverIRR(sedan_vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
truck_driver.SetSteeringDelta(render_step_size / steering_time)
truck_driver.SetThrottleDelta(render_step_size / throttle_time)
truck_driver.SetBrakingDelta(render_step_size / braking_time)


sedan_driver.SetThrottle(0.5)
sedan_driver.SetSteering(0.2)

truck_driver.Initialize()
sedan_driver.Initialize()


print( "TRUCK MASS: ",  truck.GetTractor().GetMass())
print( "SEDAN MASS: ",  sedan.GetTractor().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


truck_initial_state = truck.GetTractor().GetPos()
truck_initial_rot = truck.GetTractor().GetRot()

while truck_vis.Run() and sedan_vis.Run() :
    time = truck.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        truck_vis.BeginScene()
        truck_vis.Render()
        truck_vis.EndScene()
        sedan_vis.BeginScene()
        sedan_vis.Render()
        sedan_vis.EndScene()
        render_frame += 1

    
    truck_driver_inputs = truck_driver.GetInputs()
    sedan_driver_inputs = sedan_driver.GetInputs()

    
    truck_driver.Synchronize(time)
    sedan_driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, truck_driver_inputs, terrain)
    sedan.Synchronize(time, sedan_driver_inputs, terrain)
    truck_vis.Synchronize(time, truck_driver_inputs)
    sedan_vis.Synchronize(time, sedan_driver_inputs)

    
    truck_driver.Advance(step_size)
    sedan_driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    truck_vis.Advance(step_size)
    sedan_vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)


print("TRUCK FINAL POSITION: ", truck.GetTractor().GetPos())
print("TRUCK FINAL ROTATION: ", truck.GetTractor().GetRot())