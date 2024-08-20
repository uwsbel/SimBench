import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import random

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID



terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)


contact_method = chrono.ChContactMethod_SMC
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


terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6,   
                            0,     
                            1.1,   
                            0,     
                            30,    
                            0.01,  
                            2e8,   
                            3e4    
)


terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))


terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)


terrain.Initialize(20, 20, 0.02)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()


num_boxes = 10
box_size = chrono.ChVectorD(1, 1, 1)
for _ in range(num_boxes):
    x = random.uniform(-terrainLength / 2 + 5, terrainLength / 2 - 5)
    y = random.uniform(-terrainWidth / 2 + 5, terrainWidth / 2 - 5)
    z = terrainHeight + box_size.z / 2
    box_loc = chrono.ChVectorD(x, y, z)
    while (box_loc - initLoc).Length() < 5:  
        x = random.uniform(-terrainLength / 2 + 5, terrainLength / 2 - 5)
        y = random.uniform(-terrainWidth / 2 + 5, terrainWidth / 2 - 5)
        box_loc = chrono.ChVectorD(x, y, z)
    box_body = chrono.ChBodyEasyBox(box_size, 1000, True, True, box_loc)
    vehicle.GetSystem().Add(box_body)


sensor_manager = sens.ChSensorManager(vehicle.GetSystem())


light_positions = [chrono.ChVectorD(0, 0, 10), chrono.ChVectorD(10, 0, 0), chrono.ChVectorD(-10, 0, 0)]
for pos in light_positions:
    light = sens.ChPointLight(pos, chrono.ChColor(1, 1, 1), 100)
    sensor_manager.AddLight(light)


camera_pos = chrono.ChVectorD(0, 0, 1.71) + initLoc
camera_rot = chrono.ChQuaternionD(1, 0, 0, 0)
camera_res = chrono.ChVector2I(640, 480)
camera_fov = chrono.CH_C_PI / 3
camera_sensor = sens.ChCameraSensor(vehicle.GetChassisBody(), camera_pos, camera_rot, camera_res, camera_fov)
sensor_manager.AddSensor(camera_sensor)


filter = sens.ChFilterVisualize(camera_sensor)
sensor_manager.AddFilter(filter)


sensor_manager.Initialize()






print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    sensor_manager.Synchronize(time)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    sensor_manager.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)