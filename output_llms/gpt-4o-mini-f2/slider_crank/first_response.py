import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# Create the Chrono simulation system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # No gravity in this simulation

# Create the floor truss as a box
floor = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True)
floor.SetPos(chrono.ChVector3d(0, 0, -0.5))  # Position the floor slightly below the origin
floor.SetFixed(True)  # Fix the floor in place
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # Set texture for the floor
system.Add(floor)  # Add the floor to the simulation

# Create the crankshaft as a cylinder
crankshaft = chrono.ChBodyEasyCylinder(0.25, 5, 1000, True, True)
crankshaft.SetPos(chrono.ChVector3d(-2.5, 0, 0))  # Position the crankshaft to the left of the origin
crankshaft.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  # Set texture for the crankshaft
system.Add(crankshaft)  # Add the crankshaft to the simulation

# Create the connecting rod as a box
connecting_rod = chrono.ChBodyEasyBox(1, 3, 1, 1000, True, True)
connecting_rod.SetPos(chrono.ChVector3d(-1.25, 0, 0))  # Position the connecting rod between the crankshaft and piston
connecting_rod.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/green.png"))  # Set texture for the connecting rod
system.Add(connecting_rod)  # Add the connecting rod to the simulation

# Create the piston as a cylinder
piston = chrono.ChBodyEasyCylinder(0.5, 1, 1000, True, True)
piston.SetPos(chrono.ChVector3d(1, 0, 0))  # Position the piston to the right of the connecting rod
piston.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/red.png"))  # Set texture for the piston
system.Add(piston)  # Add the piston to the simulation

# Create a motor to spin the crankshaft
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, floor, chrono.ChFramed(chrono.ChVector3d(-2.5, 0, 0)))  # Initialize the motor at the crankshaft's position
motor.SetSpeedFunction(chrono.ChFunctionConst(chrono.CH_PI))  # Set the motor speed to π rad/s (180°/s)
system.Add(motor)  # Add the motor to the simulation

# Create a revolute joint to connect the crankshaft and connecting rod
joint1 = chrono.ChLinkLockRevolute()
joint1.Initialize(connecting_rod, crankshaft, chrono.ChFramed(chrono.ChVector3d(-1.25, 0, 0)))  # Initialize the joint at the connecting rod's position
system.Add(joint1)  # Add the joint to the simulation

# Create a revolute joint to connect the connecting rod and piston
joint2 = chrono.ChLinkLockRevolute()
joint2.Initialize(piston, connecting_rod, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))  # Initialize the joint at the piston’s position
system.Add(joint2)  # Add the joint to the simulation

# Create a prismatic joint to connect the piston and floor, allowing linear motion along the x-axis
joint3 = chrono.ChLinkLockPrismatic()
joint3.Initialize(piston, floor, chrono.ChFramed(chrono.ChVector3d(1, 0, 0), chrono.QuatFromAngleY(chrono.CH_PI_2)))  # Initialize the joint at the piston’s position, rotated 90° to align with the x-axis
system.Add(joint3)  # Add the joint to the simulation

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('Crank slider')  # Set the window title
vis.Initialize()  # Initialize the visualization window
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for better aesthetics
vis.AddCamera(chrono.ChVector3d(0, 3, 5), chrono.ChVector3d(0, 0, 0))  # Add a camera to the visualization
vis.AddTypicalLights()  # Add typical lights for better visibility

# Set the simulation time step
time_step = 1e-2

# Run the simulation loop
while vis.Run():
    vis.BeginScene()  # Begin the visualization scene
    vis.Render()  # Render the scene
    vis.EndScene()  # End the visualization scene
    system.DoStepDynamics(time_step)  # Advance the simulation by one time step