import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath('./data/')


mysystem = chrono.ChSystemNSC()
chrono.ChCollisionSystem.SetDefaultSuggestedEnvelope(0.001)


kraz = veh.ChKraz()
kraz.SetChassisVisualizationType(veh.VisualizationType_MESH)
kraz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
kraz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
kraz.SetWheelVisualizationType(veh.VisualizationType_MESH)
kraz.SetChassisMaterial(chrono.ChMaterialSurfaceNSC())
kraz.SetChassisMass(1500)
kraz.SetChassisInertiaXX(chrono.ChVectorD(1, 1, 1))
kraz.SetChassisInertiaXY(chrono.ChVectorD(0, 0, 0))
kraz.SetChassisInertiaXZ(chrono.ChVectorD(0, 0, 0))
kraz.SetChassisInertiaYY(chrono.ChVectorD(1, 1, 1))
kraz.SetChassisInertiaYZ(chrono.ChVectorD(0, 0, 0))
kraz.SetChassisInertiaZZ(chrono.ChVectorD(1, 1, 1))
kraz.SetChassisPosition(chrono.ChVectorD(0, 0.5, 0))
kraz.SetChassisOrientation(chrono.ChQuaternionD(1, 0, 0, 0))
mysystem.Add(kraz.GetChassisBody())


terrain = chrono.ChBodyEasyBox(mysystem, 10, 10, 1, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -0.5, 0))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
terrain.SetFriction(0.9)
terrain.SetRestitution(0.1)
mysystem.Add(terrain)


driver = veh.ChIrrGuiDriver()
driver.SetVehicle(kraz)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.01)
driver.SetBrakingDelta(0.1)
driver.Initialize()


application = chronoirr.ChVisualSystemIrrlicht()
application.SetWindowSize(800, 600)
application.SetWindowTitle('Kraz Vehicle Simulation')
application.AddSkyBox()
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 1, -3), chrono.ChVectorD(0, 1, 0))
application.SetSymbolicCameraSpeed(0.5)
application.SetShowLogo(False)
application.SetCameraVerticalRotationSpeed(0.5)
application.SetCameraHorizontalRotationSpeed(0.5)
application.SetCameraZoomSpeed(0.5)
application.SetCameraPanSpeed(0.5)
application.SetCameraTiltSpeed(0.5)
application.SetCameraDistance(3)
application.SetCameraAzimuth(45)
application.SetCameraElevation(30)
application.SetCameraRoll(0)
application.SetShowInfo(True)
application.SetShowFrame(True)
application.SetShowLogo(True)
application.SetShowInfoCamera(True)
application.SetShowInfoLights(True)
application.SetShowInfoFps(True)
application.SetShowInfoMemory(True)
application.SetShowInfoOs(True)
application.SetShowInfoCpu(True)
application.SetShowInfoGpu(True)
application.SetShowInfoScreen(True)
application.SetShowInfoView(True)
application.SetShowInfoCameraPosition(True)
application.SetShowInfoCameraTarget(True)
application.SetShowInfoCameraUp(True)
application.SetShowInfoCameraRight(True)
application.SetShowInfoCameraForward(True)
application.SetShowInfoCameraNear(True)
application.SetShowInfoCameraFar(True)
application.SetShowInfoCameraFov(True)
application.SetShowInfoCameraAspect(True)
application.SetShowInfoCameraNearPlane(True)
application.SetShowInfoCameraFarPlane(True)
application.SetShowInfoCameraFovAngle(True)
application.SetShowInfoCameraAspectAngle(True)
application.SetShowInfoCameraNearPlaneAngle(True)
application.SetShowInfoCameraFarPlaneAngle(True)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    mysystem.DoStepDynamics(0.01)
    driver.Synchronize(0.01)
    application.Synchronize(0.01)
    application.DrawAll()