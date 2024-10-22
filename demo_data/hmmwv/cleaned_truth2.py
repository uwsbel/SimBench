import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import numpy as np

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_PRIMITIVES


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY



terrainHeight = 0      
terrainLength = 200.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  



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


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()



vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

ballS = vis.GetSceneManager().addSphereSceneNode(0.1)
ballT = vis.GetSceneManager().addSphereSceneNode(0.1)
ballS.getMaterial(0).EmissiveColor = irr.SColor(0, 255, 0, 0)
ballT.getMaterial(0).EmissiveColor = irr.SColor(0, 0, 255, 0)




path = veh.CirclePath(initLoc, 20, 40, True, 10)
npoints = path.GetNumPoints()

path_asset = chrono.ChVisualShapeLine()
path_asset.SetLineGeometry(chrono.ChLineBezier(path))
path_asset.SetName("test path")
path_asset.SetNumRenderPoints(max(2 * npoints, 400))
patch.GetGroundBody().AddVisualShape(path_asset)


steeringPID = veh.ChPathSteeringController(path)
steeringPID.SetLookAheadDistance(5)
steeringPID.SetGains(0.8, 0, 0)
steeringPID_output = 0
throttle_value = 0.3


print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_steering = np.clip(steeringPID_output, -1.0, +1.0)
    driver_inputs.m_throttle = throttle_value
    driver_inputs.m_braking = 0.0

    pS = steeringPID.GetSentinelLocation()
    pT = steeringPID.GetTargetLocation()
    ballS.setPosition(irr.vector3df(pS.x, pS.y, pS.z))
    ballT.setPosition(irr.vector3df(pT.x, pT.y, pT.z))


    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    steeringPID_output = steeringPID.Advance(vehicle.GetRefFrame(), time, step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)