import pychrono as chrono
import pychrono.irrlicht as irr


chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')
my_system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
my_system.Add(ground)


mass = chrono.ChBodyEasySphere(1, 1000, True, True)
mass.SetPos(chrono.ChVectorD(0, 5, 0))
my_system.Add(mass)


spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(mass, ground, chrono.ChCoordsysD(chrono.ChVectorD(0, 5, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(1, 0, 0))))
my_system.Add(spring_damper)


spring_damper.SetSpringCoefficient(100)
spring_damper.SetDampingCoefficient(10)


my_application = irr.ChIrrApp(my_system, 'Mass-Spring-Damper System', irr.dimension2du(1024, 768))


ground_shape = irr.ChVisualShapeBox(20, 1, 20)
ground_shape.SetColor(irr.ChColor(0.3, 0.3, 0.3))
ground.AddVisualShape(ground_shape)

mass_shape = irr.ChVisualShapeSphere(1)
mass_shape.SetColor(irr.ChColor(0.5, 0.5, 1))
mass.AddVisualShape(mass_shape)

spring_shape = irr.ChVisualShapeSpring(0.5, 100, 10)
spring_shape.SetColor(irr.ChColor(1, 0.5, 0.5))
spring_damper.AddVisualShape(spring_shape)


my_application.AddTypicalLights()
my_application.AddCamera(irr.vector3df(0, 10, -20))
my_application.AddSkyBox()
my_application.AddLogo()
my_application.SetTimestep(0.01)


my_application.AssetBindAll()
my_application.AssetUpdateAll()
my_application.Run()