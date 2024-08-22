import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the system (in m/s^2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # g = 9.81 m/s^2

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  # The ground is fixed and does not move
ground.EnableCollision(False)  # Disable collision detection for the ground

# Add a visualization cylinder to the ground
cyl_1 = chrono.ChVisualShapeCylinder(0.2, 0.4)  # Cylinder with radius 0.2 and height 0.4
ground.AddVisualShape(cyl_1, chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))

# Create the first pendulum body and add it to the simulation
pend_1 = chrono.ChBodyAuxRef(chrono.ChVector3d(1, 0, 1), chrono.ChBody())
sys.AddBody(pend_1)
pend_1.SetFixed(False)  # The first pendulum can move
pend_1.EnableCollision(False)  # Disable collision detection for the first pendulum
pend_1.SetMass(1)  # Set the mass of the first pendulum (in kg)
pend_1.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  # Set the inertia tensor (in kg·m^2)

# Add a visualization cylinder to the first pendulum
cyl_1 = chrono.ChVisualShapeCylinder(0.2, 2)  # Cylinder with radius 0.2 and height 2
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  # Set the color of the cylinder (RGB)
pend_1.AddVisualShape(cyl_1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Create the second pendulum body and add it to the simulation
pend_2 = chrono.ChBodyAuxRef(chrono.ChVector3d(1, 0, 1), chrono.ChBody())
sys.AddBody(pend_2)
pend_2.SetFixed(False)  # The second pendulum can move
pend_2.EnableCollision(False)  # Disable collision detection for the second pendulum
pend_2.SetMass(1)  # Set the mass of the second pendulum (in kg)
pend_2.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  # Set the inertia tensor (in kg·m^2)

# Add a visualization cylinder to the second pendulum
cyl_2 = chrono.ChVisualShapeCylinder(0.2, 2)  # Cylinder with radius 0.2 and height 2
cyl_2.SetColor(chrono.ChColor(0, 0.6, 0))  # Set the color of the cylinder (RGB)
pend_2.AddVisualShape(cyl_2, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Create a revolute joint to connect the first pendulum to the ground
rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFrameD::Identity())
sys.AddLink(rev_1)

# Create a revolute joint to connect the first pendulum to the second pendulum
rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChFrameD::Identity())
sys.AddLink(rev_2)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('Double Pendulum demo')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for better visual appearance
vis.AddCamera(chrono.ChVector3d(0, 3, 6))  # Add a camera to the visualization
vis.AddTypicalLights()  # Add typical lights for better visualization

# Simulation loop
log_info = True  # Flag to control logging
while vis.Run():
    vis.BeginScene()  # Begin the scene for rendering
    vis.Render()  # Render the scene
    vis.EndScene()  # End the scene
    sys.DoStepDynamics(1e-3)  # Advance the simulation by a time step of 0.001 seconds

    # Log information after 1 second of simulation time
    if log_info and sys.GetChTime() > 1:
        pos_1 = pend_1.GetPos()  # Get the position of the first pendulum
        print("t = ", sys.GetChTime())
        print("     ", pos_1.x, "  ", pos_1.y)
        lin_vel_1 = pend_1.GetPosDt()  # Get the linear velocity of the first pendulum
        print("     ", lin_vel_1.x, "  ", lin_vel_1.y)
        log_info = False  # Disable further logging