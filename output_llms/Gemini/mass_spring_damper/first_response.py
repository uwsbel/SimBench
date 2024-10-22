import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# -------------------------------------------------------------------
# Create the simulation system and add items
# -------------------------------------------------------------------
system = chrono.ChSystemNSC()

# Set gravitational acceleration
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create a body to represent the mass
mass = 1.0  # kg
mass_body = chrono.ChBodyEasySphere(0.2,  # radius
                                    mass,   # mass
                                    True,   # collide enable?
                                    True)  # visualize?
mass_body.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(mass_body)

# Create the ground body
ground = chrono.ChBodyEasyBox(1, 0.1, 1,  # size
                              1000,     # density
                              True,     # collide enable?
                              True)     # visualize?
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)  # Fix the ground to the reference frame
system.Add(ground)

# Connect the mass to the ground with a spring-damper
spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(mass_body,               # First body connected
                         ground,                  # Second body connected
                         False,                   # Actuator or spring-damper?
                         chrono.ChVectorD(0, 0, 0),  # Point on the first body
                         chrono.ChVectorD(0, 0, 0)) # Point on the second body
system.Add(spring_damper)

# -------------------------------------------------------------------
# Create the visualization window and assets
# -------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()

# Set the background color of the scene
vis.GetGUIEnvironment().AddSkyBox(
    chrono.GetChronoDataFile("skybox/sky_06.jpg"),
    chrono.GetChronoDataFile("skybox/sky_02.jpg"),
    chrono.GetChronoDataFile("skybox/sky_01.jpg"),
    chrono.GetChronoDataFile("skybox/sky_04.jpg"),
    chrono.GetChronoDataFile("skybox/sky_05.jpg"),
    chrono.GetChronoDataFile("skybox/sky_03.jpg"), 1000)

# -------------------------------------------------------------------
# Simulate the system
# -------------------------------------------------------------------
while vis.Run():
    # Get time from the simulation
    time = system.GetChTime()

    # Advance the simulation by a time step
    system.DoStepDynamics(0.01)

    # Update the visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()