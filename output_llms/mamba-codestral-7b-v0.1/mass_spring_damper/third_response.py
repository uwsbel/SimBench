import pychrono as chrono
import pychrono.irrlicht as chronoirr

rest_length_1 = 1.5
spring_coef_1 = 50
damping_coef_1 = 1

rest_length_2 = 2.0
spring_coef_2 = 75
damping_coef_2 = 1.5

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))

body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Attach a visualization asset.
box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# Create the spring between body_1 and ground.
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length_1)
spring_1.SetSpringCoefficient(spring_coef_1)
spring_1.SetDampingCoefficient(damping_coef_1)
sys.AddLink(spring_1)

# Attach a visualization asset.
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(1, -3, 0))
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Attach a visualization asset.
box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0.0, 0.6, 0))
body_2.AddVisualShape(box_2)

# Create the spring between body_1 and body_2.
spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_1, body_2, True, chrono.ChVector3d(1, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_2.SetRestLength(rest_length_2)
spring_2.SetSpringCoefficient(spring_coef_2)
spring_2.SetDampingCoefficient(damping_coef_2)
sys.AddLink(spring_2)

# Attach a visualization asset.
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

body_3 = chrono.ChBody()
sys.AddBody(body_3)
body_3.SetPos(chrono.ChVector3d(3, -3, 0))
body_3.SetFixed(False)
body_3.EnableCollision(False)
body_3.SetMass(1)
body_3.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Attach a visualization asset.
box_3 = chrono.ChVisualShapeBox(1, 1, 1)
box_3.SetColor(chrono.ChColor(0.0, 0.0, 0.6))
body_3.AddVisualShape(box_3)

# Create the spring between body_2 and body_3.
spring_3 = chrono.ChLinkTSDA()
spring_3.Initialize(body_2, body_3, True, chrono.ChVector3d(-1, 0, 0), chrono.ChVector3d(1, 0, 0))
spring_3.SetRestLength(rest_length_2)
spring_3.SetSpringCoefficient(spring_coef_2)
spring_3.SetDampingCoefficient(damping_coef_2)
sys.AddLink(spring_3)

# Attach a visualization asset.
spring_3.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 6))
vis.AddTypicalLights()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)