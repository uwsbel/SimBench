"""
Double Pendulum Simulation using PyChrono
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the system (in m/s^2)
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))  # g = 9.81 m/s^2

# Create the ground body and add it to the simulation
ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
sys.Add(ground)

# Create the first pendulum body and add it to the simulation
pend_1 = chrono.ChBodyEasyCylinder(sys, 0.2, 2, 1000, True, True)
sys.Add(pend_1)
pend_1.SetPos(chrono.ChVectorD(1, 0, 1))

# Create the second pendulum body and add it to the simulation
pend_2 = chrono.ChBodyEasyCylinder(sys, 0.2, 2, 1000, True, True)
sys.Add(pend_2)
pend_2.SetPos(chrono.ChVectorD(2, 0, 1))

# Create a revolute joint to connect the first pendulum to the ground
rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1), chrono.Q_from_AngY(math.pi/2)))
sys.AddLink(rev_1)

# Create a revolute joint to connect the second pendulum to the first pendulum
rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChFrameD(chrono.ChVectorD(1, 0, 1), chrono.Q_from_AngY(math.pi/2)))
sys.AddLink(rev_2)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('Double Pendulum Simulation')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for better visual appearance
vis.AddCamera(chrono.ChVectorD(0, 3, 6))  # Add a camera to the visualization
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
        lin_vel_1 = pend_1.GetPos_dt()  # Get the linear velocity of the first pendulum
        print("     ", lin_vel_1.x, "  ", lin_vel_1.y)

        pos_2 = pend_2.GetPos()  # Get the position of the second pendulum
        print("     ", pos_2.x, "  ", pos_2.y)
        lin_vel_2 = pend_2.GetPos_dt()  # Get the linear velocity of the second pendulum
        print("     ", lin_vel_2.x, "  ", lin_vel_2.y)

        log_info = False  # Disable further logging