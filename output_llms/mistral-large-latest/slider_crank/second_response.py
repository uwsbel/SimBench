import pychrono.core as chrono  # Import the core Chrono library
import pychrono.irrlicht as chronoirr  # Import the Irrlicht visualization library for Chrono
import matplotlib.pyplot as plt  # Import matplotlib for plotting
import numpy as np  # Import numpy for numerical operations

# Initialize the Chrono simulation system with non-smooth contact (NSC) method
sys = chrono.ChSystemNSC()

# Define common parameters for the simulation
crank_center = chrono.ChVector3d(-1, 0.5, 0)  # Center of the crankshaft (x=-1, y=0.5, z=0)
crank_rad = 0.4  # Radius of the crankshaft (in meters)
crank_thick = 0.1  # Thickness of the crankshaft (in meters)
rod_length = 1.5  # Length of the connecting rod (in meters)

# Create the floor (truss) body, which is a box
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)  # Create a box with dimensions 3x1x3 meters and density 1000 kg/m^3
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))  # Position the floor at (x=0, y=-0.5, z=0)
mfloor.SetFixed(True)  # Fix the floor so it doesn't move
sys.Add(mfloor)  # Add the floor to the simulation system

# Create the crank body, which is a cylinder
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)  # Create a cylinder along the Y-axis with radius 0.4 meters and thickness 0.1 meters
mcrank.SetPos(crank_center)  # Position the crank at (x=-1, y=0.5, z=0)
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  # Rotate the crank to align it along the Z-axis
sys.Add(mcrank)  # Add the crank to the simulation system

# Create the connecting rod, which is a box
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  # Create a box with dimensions 1.5x0.1x0.1 meters and density 1000 kg/m^3
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad, 0, 0))  # Position the rod at (x=-0.6, y=0.5, z=0)
sys.Add(mrod)  # Add the rod to the simulation system

# Create the piston, which is a cylinder
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)  # Create a cylinder along the Y-axis with radius 0.2 meters and height 0.3 meters
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))  # Position the piston at (x=0.9, y=0.5, z=0)
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)  # Rotate the piston to align it along the X-axis
sys.Add(mpiston)  # Add the piston to the simulation system

# Create a motor to spin the crankshaft
my_motor = chrono.ChLinkMotorRotationSpeed()  # Create a motor that controls rotational speed
my_motor.Initialize(mcrank, mfloor, chrono.ChFrameD(crank_center))  # Initialize the motor at the crank center, connecting the crank and the floor
my_angularspeed = chrono.ChFunction_Const(chrono.CH_C_PI)  # Set the angular speed of the motor to π rad/s (180°/s)
my_motor.SetMotorFunction(my_angularspeed)  # Assign the angular speed function to the motor
sys.Add(my_motor)  # Add the motor to the simulation system

# Create a revolute joint to connect the crank to the rod
mjointA = chrono.ChLinkLockRevolute()  # Create a revolute (hinge) joint
mjointA.Initialize(mrod, mcrank, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))  # Initialize the joint at (x=-0.6, y=0.5, z=0)
sys.Add(mjointA)  # Add the joint to the simulation system

# Create a revolute joint to connect the rod to the piston
mjointB = chrono.ChLinkLockRevolute()  # Create a revolute (hinge) joint
mjointB.Initialize(mpiston, mrod, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))  # Initialize the joint at (x=0.9, y=0.5, z=0)
sys.Add(mjointB)  # Add the joint to the simulation system

# Create a prismatic joint to connect the piston to the floor, allowing linear motion along the X-axis
mjointC = chrono.ChLinkLockPrismatic()  # Create a prismatic (slider) joint
mjointC.Initialize(mpiston, mfloor, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), chrono.Q_ROTATE_Z_TO_X))  # Initialize the joint at (x=0.9, y=0.5, z=0) with rotation aligning the Z-axis to the X-axis
sys.Add(mjointC)  # Add the joint to the simulation system

# Initialize arrays for plotting
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Set up the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()  # Create the Irrlicht visualization system
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization (1024x768 pixels)
vis.SetWindowTitle('Crank demo')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for better visual appearance
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))  # Add a camera to the visualization, positioned at (x=1, y=1, z=3) and looking at (x=0, y=1, z=0)
vis.AddTypicalLights()  # Add typical lights for better visualization

# Run the interactive simulation loop
simulation_time = 0
step_size = 1e-3  # Time step size in seconds
max_sim_time = 20  # Maximum simulation time in seconds

while vis.Run() and simulation_time < max_sim_time:
    # Visualization and time step integration
    vis.BeginScene()  # Begin the visualization scene
    vis.Render()  # Render the scene
    vis.EndScene()  # End the visualization scene

    # Collect data
    time = sys.GetChTime()
    angle = my_motor.GetMotorRot()
    pos = mpiston.GetPos().x
    speed = mpiston.GetPos_dt().x

    array_time.append(time)
    array_angle.append(angle)
    array_pos.append(pos)
    array_speed.append(speed)

    # Advance the simulation by a time step
    sys.DoStepDynamics(step_size)
    simulation_time += step_size

# Matplotlib Plotting
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# First subplot: Position [m] vs. Crank Angle [rad]
ax1.plot(array_angle, array_pos)
ax1.set_ylabel('Position [m]')
ax1.set_title('Position vs. Crank Angle')
ax1.xaxis.set_major_locator(plt.MultipleLocator(np.pi / 2))
ax1.xaxis.set_minor_locator(plt.MultipleLocator(np.pi / 4))
ax1.xaxis.set_major_formatter(plt.FuncFormatter(lambda val, pos: '{:.0f}$\pi$'.format(val / np.pi) if val != 0 else '0'))

# Second subplot: Speed [m/s] vs. Crank Angle [rad]
ax2.plot(array_angle, array_speed)
ax2.set_ylabel('Speed [m/s]')
ax2.set_xlabel('Crank Angle [rad]')
ax2.set_title('Speed vs. Crank Angle')
ax2.xaxis.set_major_locator(plt.MultipleLocator(np.pi / 2))
ax2.xaxis.set_minor_locator(plt.MultipleLocator(np.pi / 4))
ax2.xaxis.set_major_formatter(plt.FuncFormatter(lambda val, pos: '{:.0f}$\pi$'.format(val / np.pi) if val != 0 else '0'))

plt.show()