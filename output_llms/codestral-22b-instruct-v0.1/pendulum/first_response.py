import pychrono as chrono
import pychrono.irrlicht as irr

# Initialize PyChrono system
chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')
my_system = chrono.ChSystemNSC()

# Create ground body
ground = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
my_system.Add(ground)

# Create pendulum body
pendulum = chrono.ChBodyEasySphere(0.5, 1000, True, True)
pendulum.SetPos(chrono.ChVectorD(0, 5, 0))
my_system.Add(pendulum)

# Create revolute joint
joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 5, 0)))
my_system.Add(joint)

# Set initial conditions
pendulum.SetWvel_par(chrono.ChVectorD(0, 0, 3))

# Create Irrlicht application
my_application = irr.ChIrrApp(my_system, 'Pendulum Simulation', irr.dimension2du(1024, 768))

# Set up visualization
my_application.AddTypicalLights()
my_application.AddCamera(irr.vector3df(0, 10, -20))
my_application.AssetBindAll()
my_application.AssetUpdateAll()

# Run the simulation
while my_application.GetDevice().run():
    my_application.BeginScene()
    my_application.DrawAll()
    my_application.DoStep()
    my_application.EndScene()

    # Log pendulum's position and velocity
    print("Position: ", pendulum.GetPos())
    print("Velocity: ", pendulum.GetWvel_par())

    # Limit the simulation frame rate for visualization
    chrono.ChTime.Sleep(0.01)