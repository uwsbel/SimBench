import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as irr
import numpy as np


chrono.SetChronoDataPath(chrono.GetChronoDataPath())


system = chrono.ChSystemNSC()


gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
gator.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
gator.SetDriveType(veh.DrivelineTypeWV_AWD)
gator.SetTireType(veh.TireModelType_TMEASY)
gator.Initialize()


gator.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                         chrono.ChVectorD(200, 200, 10), True, True, True)
patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetContactMaterialProperties(2e7, 0.3)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(chrono.GetChronoDataFile('terrain/textures/tile4.jpg'), 200, 200)
terrain.Initialize()


driver = veh.ChIrrGuiDriver(gator.GetVehicle())
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)


manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVectorF(0, 0, 10), chrono.ChColor(1, 1, 1), 500)


camera = sens.ChCameraSensor(
    gator.GetChassisBody(),
    30,  
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 1.5), chrono.Q_from_AngAxis(np.pi, chrono.ChVectorD(0, 1, 0))),
    1280,  
    720,   
    chrono.CH_CameraLens(chrono.CH_CameraLens.Perspective, 1.4, 1.4, 0.1, 100)
)
camera.SetName("Camera Sensor")
camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera View"))
camera.PushFilter(sens.ChFilterRGBA8Access())
manager.AddSensor(camera)


app = veh.ChWheeledVehicleIrrApp(gator.GetVehicle(), 'Gator Vehicle Demo')
app.SetSkyBox()
app.AddTypicalLights()
app.AddTypicalLogo()
app.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
app.SetTimestep(0.01)
app.AssetBindAll()
app.AssetUpdateAll()


timestep = 0.01
while app.GetDevice().run():
    time = system.GetChTime()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver.GetInputs(), terrain)
    app.Synchronize("", driver.GetInputs())

    
    driver.Advance(timestep)
    terrain.Advance(timestep)
    gator.Advance(timestep)
    app.Advance(timestep)
    manager.Update()

    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()