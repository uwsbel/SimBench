import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.ChSystemNSC.SetNumThreads(1)
system = chrono.ChSystemNSC()


application = chronoirr.ChIrrApp(system, "Crank-Slider Mechanism", chronoirr.dimension2du(800, 600))
application.AddLogo(chrono.GetChronoDataPath() + "logo_pychrono_alpha.png")
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, 1, -5), chrono.ChVectorD(0, 0, 0))
application.AddLight(chrono.ChVectorD(10, 10, 10), chrono.ChVectorD(1, 1, 1))


ground = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.05, 0))
ground.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
system.Add(ground)


crankshaft = chrono.ChBodyEasyCylinder(0.05, 1.0, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
crankshaft.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
system.Add(crankshaft)


connecting_rod = chrono.ChBodyEasyBox(0.02, 0.5, 0.1, 1000, True, True)
connecting_rod.SetPos(chrono.ChVectorD(0.5, 0.25, 0))
connecting_rod.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
system.Add(connecting_rod)


piston = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True)
piston.SetPos(chrono.ChVectorD(1.0, 0.1, 0))
piston.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
system.Add(piston)


crank_joint = chrono.ChLinkLockRevolute()
crank_joint.Initialize(crankshaft, ground, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
system.Add(crank_joint)

rod_joint = chrono.ChLinkLockSpherical()
rod_joint.Initialize(connecting_rod, crankshaft, chrono.ChVectorD(0.5, 0.25, 0))
system.Add(rod_joint)

piston_joint = chrono.ChLinkLockSpherical()
piston_joint.Initialize(piston, connecting_rod, chrono.ChVectorD(1.0, 0.1, 0))
system.Add(piston_joint)


motor = chrono.ChLinkMotorRotationTorque()
motor.Initialize(crankshaft, ground, chrono.ChVectorD(0, 0, 0))
motor.SetMotorFunction(chrono.ChFunction_Const(10))  
system.Add(motor)


crankshaft.SetRot(chrono.Q_from_AngZ(0))  
connecting_rod.SetPos(chrono.ChVectorD(0.5, 0.25, 0))
piston.SetPos(chrono.ChVectorD(1.0, 0.1, 0))


application.SetTimestep(0.01)
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(application.GetTimestep())


application.GetDevice().drop()