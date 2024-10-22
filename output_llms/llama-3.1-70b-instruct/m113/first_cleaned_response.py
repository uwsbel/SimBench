import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath('./data/')
system = chrono.ChSystemNSC()



m113 = veh.M113()
m113.SetChassisVisualizationType(veh.VisualizationType_MESH)
m113.SetChassisMaterial(chrono.ChMaterialSurfaceNSC())
m113.SetChassisCollisionType(chrono.ChCollisionType_MESH)
m113.SetChassisSuggestedEnvelope(chrono.ChVectorD(1, 1, 1))
m113.SetChassisSuggestedMargin(chrono.ChVectorD(0.1, 0.1, 0.1))
m113.SetChassisMass(5000)
m113.SetChassisInertiaXX(chrono.ChVectorD(1000, 1000, 1000))
m113.SetChassisInertiaXY(chrono.ChVectorD(0, 0, 0))
m113.SetChassisInertiaXZ(chrono.ChVectorD(0, 0, 0))
m113.SetChassisInertiaYY(chrono.ChVectorD(1000, 1000, 1000))
m113.SetChassisInertiaYZ(chrono.ChVectorD(0, 0, 0))
m113.SetChassisInertiaZZ(chrono.ChVectorD(1000, 1000, 1000))
m113.SetChassisPosition(chrono.ChVectorD(0, 0, 0))
m113.SetChassisRotation(chrono.ChQuaternionD(1, 0, 0, 0))
m113.SetChassisVelocity(chrono.ChVectorD(0, 0, 0))
m113.SetChassisAcceleration(chrono.ChVectorD(0, 0, 0))
m113.SetChassisAngularVelocity(chrono.ChVectorD(0, 0, 0))
m113.SetChassisAngularAcceleration(chrono.ChVectorD(0, 0, 0))
system.Add(m113.GetChassisBody())


terrain = chrono.ChBodyEasyBox(system, 100, 100, 10, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -10, 0))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
terrain.SetFriction(0.8)
terrain.SetRestitution(0.1)
system.Add(terrain)


driver = veh.ChIrrlichtDriver()
driver.SetVehicle(m113)
driver.SetSteering(0)
driver.SetThrottle(0)
driver.SetBraking(0)


irrApp = chronoirr.ChVisualSystemIrrlicht()
irrApp.SetWindowSize(1024, 768)
irrApp.SetWindowTitle('M113 Simulation')
irrApp.SetCamera(chrono.ChVectorD(0, 0, 10), chrono.ChVectorD(0, 0, 0))
irrApp.AddTypicalLights()
irrApp.AddSkyBox()
irrApp.AddLogo()
irrApp.SetShowInfos(True)
irrApp.SetShowCollisionModels(True)
irrApp.SetShowContactForces(True)
irrApp.SetShowFEMeshes(True)
irrApp.SetShowMarkers(True)
irrApp.SetShowShadow(True)
irrApp.SetShowSkyBox(True)
irrApp.SetShowLogo(True)
irrApp.SetShowInfos(True)
irrApp.SetShowCollisionModels(True)
irrApp.SetShowContactForces(True)
irrApp.SetShowFEMeshes(True)
irrApp.SetShowMarkers(True)
irrApp.SetShowShadow(True)
irrApp.SetShowSkyBox(True)
irrApp.SetShowLogo(True)


while (irrApp.Run()):
    
    driver.Synchronize(chrono.ChTime(0.01))
    system.DoStepDynamics(chrono.ChTime(0.01))
    irrApp.BeginScene()
    irrApp.DrawAll()
    irrApp.EndScene()