import pychrono as chrono
import pychrono.irrlicht as chronoirr

rest_length = 1.5
spring_coef = 50
damping_coef = 1

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, 0))

ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.SetCollide(False)

# Add visual shape for ground
sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFrameD(chrono.ChVectorD(-1, 0, 0)))

# Create body_1
body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVectorD(-1, -3, 0))
body_1.SetFixed(False)
body_1.SetCollide(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVectorD(1, 1, 1))

# Attach a visualization asset to body_1
box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# Create the spring between body_1 and ground
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)

# Attach a visualization asset to spring_1
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 15, 10))

# Create body_2
body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVectorD(1, -3, 0))
body_2.SetFixed(False)
body_2.SetCollide(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVectorD(1, 1, 1))

# Attach a visualization asset to body_2
box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0.6, 0))
body_2.AddVisualShape(box_2)

# Create the spring between body_1 and body_2
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_1, body_2, True, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
spring_2.SetRestLength(rest_length)
spring_2.SetSpringCoefficient(spring_coef)
spring_2.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_2)

# Attach a visualization asset to spring_2
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 15, 10))

# Create body_3
body_3 = chrono.ChBody()
sys.AddBody(body_3)
body_3.SetPos(chrono.ChVectorD(3, -3, 0))
body_3.SetFixed(False)
body_3.SetCollide(False)
body_3.SetMass(1)
body_3.SetInertiaXX(chrono.ChVectorD(1, 1, 1))

# Attach a visualization asset to body_3
box_3 = chrono.ChVisualShapeBox(1, 1, 1)
box_3.SetColor(chrono.ChColor(0, 0, 0.6))
body_3.AddVisualShape(box_3)

# Create the spring between body_2 and body_3
spring_3 = chrono.ChLinkTSDA()
spring_3.Initialize(body_2, body_3, True, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
spring_3.SetRestLength(rest_length)
spring_3.SetSpringCoefficient(spring_coef)
spring_3.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_3)

# Attach a visualization asset to spring_3
spring_3.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 15, 10))

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 6))
vis.AddTypicalLights()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)