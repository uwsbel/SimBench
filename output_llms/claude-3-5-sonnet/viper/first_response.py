import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import math

# Initialize the Chrono system
chrono.SetChronoDataPath("C:/Program Files/Chrono/data/")
system = chrono.ChSystemNSC()

# Set gravity
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the ground body
ground = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True)
ground.SetBodyFixed(True)
system.Add(ground)

# Initialize the Viper rover
viper = veh.Viper()
viper.SetContactMethod(chrono.ChContactMethod_NSC)
viper.SetChassisFixed(False)
viper.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
viper.SetTireType(veh.TireModelType_RIGID)
viper.Initialize()

# Create and initialize the powertrain
powertrain = veh.SimpleMapPowertrain(chrono.GetChronoDataFile("vehicle/powertrain/powertrain_map.json"))
viper.InitializePowertrain(powertrain)

# Create the driver
driver = veh.ViperDriver(viper)

# Add Viper rover to the system
viper.GetSystem().Add(ground)

# Create the visualization application
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 5, -8), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()

# Simulation parameters
step_size = 1e-3
simulation_time = 10.0
steering_time = 5.0
max_steering = 0.5

# Simulation loop
time = 0
while time < simulation_time:
    # Calculate steering input
    if time < steering_time:
        steering = (time / steering_time) * max_steering
    else:
        steering = max_steering
    
    # Update driver inputs
    throttle = 0.5  # Constant throttle
    braking = 0.0   # No braking
    
    # Update the rover
    driver.SetInputs(steering, throttle, braking)
    viper.Synchronize(time, driver.GetInputs(), system.GetContactContainer())
    
    # Advance the simulation
    system.DoStepDynamics(step_size)
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    time += step_size

# Clean up
vis.GetDevice().closeDevice()