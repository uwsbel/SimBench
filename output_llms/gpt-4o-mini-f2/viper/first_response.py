import pychrono as chrono
import pychrono.robot as viper
import math

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in negative Z direction

# Create ground body
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetFixed(True)  # Fix the ground in place
ground.SetName("ground")
system.Add(ground)

# Create Viper rover
rover = viper.Viper(system)  # Instantiate the Viper rover
rover.Initialize()  # Initialize the rover

# Create rover driver
driver = viper.ViperDCMotorControl()
rover.SetDriver(driver)  # Set the driver for the rover

# Create run-time visualization
vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40)

# Enable shadows (commented out to improve performance)
# vis.EnableShadows()

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update the rover's steering over time
    if time < 2.0:
        driver.SetSteering(0.5 * math.pi * time / 2.0)  # Gradually steer left
    elif time < 4.0:
        driver.SetSteering(0.5 * math.pi * (4.0 - time) / 2.0)  # Gradually steer right
    else:
        driver.SetSteering(0.5 * math.pi)  # Set steering to maximum right

    rover.Update()  # Update the rover's position and state

    # Increment the simulation time
    time += time_step

    # Set the time step for the simulation
    system.DoStepDynamics(time_step)