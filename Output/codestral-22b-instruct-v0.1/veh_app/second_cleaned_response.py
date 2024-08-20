import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os


veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

print(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, -5, 0.4)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_NONE
tire_vis_type = veh.VisualizationType_MESH


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


step_size = 1e-3
tire_step_size = step_size


tend = 1000


render_step_size = 1.0 / 50  


noise_model = "NONE"  


update_rate = 10


image_width = 1280
image_height = 720


fov = 1.408


lag = 0


exposure_time = 0


vis = True


gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetBrakeType(veh.BrakeType_SHAFTS)
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(tire_step_size)
gator.SetInitFwdVel(0.0)
gator.Initialize()

gator.SetChassisVisualizationType(chassis_vis_type)
gator.SetSuspensionVisualizationType(suspension_vis_type)
gator.SetSteeringVisualizationType(steering_vis_type)
gator.SetWheelVisualizationType(wheel_vis_type)
gator.SetTireVisualizationType(tire_vis_type)


print("Vehicle mass:   " + str(gator.GetVehicle().GetMass()))
print("Driveline type: " + gator.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type:     " + gator.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type:      " + gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print("\n")


gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)




terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50, 50)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)
terrain.Initialize()




box_body = chrono.ChBody()
box_body.SetPos(chrono.ChVector3d(0, 0, 0.5))
box_body.SetMass(1)
box_body.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
box_body.SetBodyFixed(False)
gator.GetSystem().Add(box_body)

box_shape = chrono.ChBoxShape()
box_shape.GetBoxGeometry().SetLengths(chrono.ChVector3d(1, 1, 1))
box_material = chrono.ChMaterialSurfaceNSC()
box_material.SetFriction(0.9)
box_material.SetRestitution(0.01)
box_body.AddAsset(box_shape)
box_body.SetMaterialSurface(box_material)


box_texture = chrono.ChTexture()
box_texture.SetTextureFilename(veh.GetDataFile("textures/blue.png"))
box_body.AddAsset(box_texture)




cylinder_body = chrono.ChBody()
cylinder_body.SetPos(chrono.ChVector3d(0, 0, 1.5))
cylinder_body.SetMass(1)
cylinder_body.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
cylinder_body.SetBodyFixed(False)
gator.GetSystem().Add(cylinder_body)

cylinder_shape = chrono.ChCylinderShape()
cylinder_shape.GetCylinderGeometry().p1 = chrono.ChVector3d(0, 0, 0)
cylinder_shape.GetCylinderGeometry().p2 = chrono.ChVector3d(0, 0, 1)
cylinder_shape.GetCylinderGeometry().rad = 0.5
cylinder_material = chrono.ChMaterialSurfaceNSC()
cylinder_material.SetFriction(0.9)
cylinder_material.SetRestitution(0.01)
cylinder_body.AddAsset(cylinder_shape)
cylinder_body.SetMaterialSurface(cylinder_material)


cylinder_texture = chrono.ChTexture()
cylinder_texture.SetTextureFilename(veh.GetDataFile("textures/blue.png"))
cylinder_body.AddAsset(cylinder_texture)


driver = veh.ChDriver(gator.GetVehicle())

driver.Initialize()




manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3d(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)


offset_pose = chrono.ChFrame(chrono.ChVector3d(-8.0, 0, 1.45), chrono.Q_from_AngAxis(.2, chrono.ChVector3d(0, 1, 0)))
cam = sens.ChCameraSensor(
    gator.GetChassisBody(),
    update_rate,
    offset_pose,
    image_width,
    image_height,
    fov
)
cam.SetName("Third Person POV")

cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Gator Camera"))
manager.AddSensor(cam)




lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),
    update_rate,
    chrono.ChFrame(chrono.ChVector3d(0.0, 0, 2)),
    noise_model,
    lag,
    exposure_time,
    vis
)
lidar.SetName("Lidar")
lidar.SetScanParameters(800, 300, 2 * chrono.CH_PI, chrono.CH_PI / 12, -chrono.CH_PI / 6, 100.0)
lidar.SetBeamShape(sens.ChLidarSensor.BeamShape_RECTANGULAR)
lidar.SetSampleRadius(2)
lidar.SetDivergenceAngle(0.003)
lidar.SetReturnMode(sens.ChLidarSensor.ReturnMode_STRONGEST)
lidar.PushFilter(sens.ChFilterLidarDistance())
lidar.PushFilter(sens.ChFilterLidarIntensity())
lidar.PushFilter(sens.ChFilterLidarXYZI())
lidar.PushFilter(sens.ChFilterLidarVisualize())
manager.AddSensor(lidar)





realtime_timer = chrono.ChRealtimeStepTimer()
time = 0
end_time = 30
while time < end_time:
    time = gator.GetSystem().GetChTime()
    
    driver.SetSteering(0.5)
    driver.SetThrottle(0.2)
    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)

    manager.Update()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)

    
    realtime_timer.Spin(step_size)