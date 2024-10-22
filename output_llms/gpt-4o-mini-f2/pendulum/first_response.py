import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# Create the Chrono simulation system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Set gravity in the negative Y direction

# Create the ground body and add it to the system
ground = chrono.ChBody()
system.AddBody(ground)
ground.SetFixed(True)  # The ground is fixed and does not move
ground.EnableCollision(False)  # Disable collision for the ground

# Create a visualization shape for the ground
box = chrono.ChVisualShapeBox(4, 0.5, 4)  # Create a box of size 4x0.5x4
ground.AddVisualShape(box, chrono.ChFramed(chrono.ChVector3d(0, -0.25, 0)))  # Add the box to the ground at position (0, -0.25, 0)

# Create the pendulum body and add it to the system
pendulum = chrono.ChBody()
system.AddBody(pendulum)
pendulum.SetFixed(False)  # The pendulum can move
pendulum.EnableCollision(False)  # Disable collision for the pendulum
pendulum.SetMass(1)  # Set the mass of the pendulum
pendulum.SetInertiaXX(chrono.ChVector3d(0.0833, 0.6667, 0.6667))  # Set the inertia of the pendulum (in kg*m^2)

# Create a visualization shape for the pendulum
box = chrono.ChVisualShapeBox(1, 0.5, 0.5)  # Create a box of size 1x0.5x0.5
box.SetColor(chrono.ChColor(0.8, 0, 0))  # Set the color of the box to red
pendulum.AddVisualShape(box, chrono.ChFramed(chrono.ChVector3d(-0.5, 0, 0)))  # Add the box to the pendulum at position (-0.5, 0, 0)

# Set the initial position and orientation of the pendulum
pendulum.SetPos(chrono.ChVector3d(-1, 0, 0))  # Set the position of the pendulum to (-1, 0, 0)
pendulum.SetRot(chrono.QuatFromAngleY(0))  # Set the orientation of the pendulum to 0 radians around the Y-axis

# Create a revolute joint to connect the pendulum to the ground
joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleY(0)))  # Initialize the joint at the origin with no rotation
system.AddLink(joint)  # Add the joint to the system

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('Simple pendulum')  # Set the window title
vis.Initialize()  # Initialize the visualization
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for better aesthetics
vis.AddCamera(chrono.ChVector3d(0, 2, 6), chrono.ChVector3d(0, 1, 0))  # Add a camera to the visualization
vis.AddTypicalLights()  # Add typical lights to the visualization

# Set the simulation time step
time_step = 0.01

# Simulation loop
while vis.Run():
    vis.BeginScene()  # Begin the scene for rendering
    vis.Render()  # Render the scene
    vis.EndScene()  # End the scene

    # Log the position and velocity of the pendulum
    pos = pendulum.GetPos()  # Get the position of the pendulum
    print(f'Pendulum position: {pos.x}, {pos.y}, {pos.z}')  # Log the position
    print(f'Pendulum velocity: {pendulum.GetPosDt().x}, {pendulum.GetPosDt().y}, {pendulum.GetPosDt().z}')  # Log the velocity

    system.DoStepDynamics(time_step)  # Advance the simulation by one time step