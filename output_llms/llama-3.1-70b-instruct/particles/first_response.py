import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Initialize the PyChrono environment
chrono.SetChronoDataPath('./')

# Create the simulation system
mysystem = chrono.ChSystemNSC()

# Set the gravity
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the particle emitter
class ParticleEmitter:
    def __init__(self, system):
        self.system = system
        self.particle_count = 0

    def emit_particle(self):
        # Random shape (sphere or box)
        shape_type = np.random.choice(['sphere', 'box'])

        if shape_type == 'sphere':
            # Random radius
            radius = np.random.uniform(0.1, 0.5)
            # Create a sphere
            particle = chrono.ChBodyEasySphere(self.system, radius, 1000, True, True)
        else:
            # Random dimensions
            length = np.random.uniform(0.1, 0.5)
            width = np.random.uniform(0.1, 0.5)
            height = np.random.uniform(0.1, 0.5)
            # Create a box
            particle = chrono.ChBodyEasyBox(self.system, length, width, height, 1000, True, True)

        # Random position
        x = np.random.uniform(-5, 5)
        y = np.random.uniform(5, 10)
        z = np.random.uniform(-5, 5)
        particle.SetPos(chrono.ChVectorD(x, y, z))

        # Random velocity
        vx = np.random.uniform(-1, 1)
        vy = np.random.uniform(-1, 1)
        vz = np.random.uniform(-1, 1)
        particle.SetVel(chrono.ChVectorD(vx, vy, vz))

        # Random orientation
        angle = np.random.uniform(0, 2 * np.pi)
        axis = chrono.ChVectorD(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1))
        axis.Normalize()
        particle.SetRot(chrono.Q_from_AngAxis(angle, axis))

        # Add the particle to the system
        self.system.Add(particle)
        self.particle_count += 1

# Create the particle emitter
emitter = ParticleEmitter(mysystem)

# Emit particles
for _ in range(100):
    emitter.emit_particle()

# Create the Irrlicht visualization
myapplication = chronoirr.ChVisualSystemIrrlicht()

# Set up the camera
myapplication.AddCamera(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(0, 1, 0))

# Set up the Irrlicht driver
myapplication.SetWindowSize(800, 600)
myapplication.SetWindowTitle('PyChrono Simulation')
myapplication.SetCameraDistance(10)
myapplication.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))

# Create the custom gravitational force
class CustomGravitationalForce(chrono.ChForce):
    def __init__(self, system):
        super().__init__()
        self.system = system

    def ComputeF(self, time):
        # Iterate over all particles
        for particle in self.system.Get_bodies():
            # Calculate the gravitational force
            force = chrono.ChVectorD(0, 0, 0)
            for other_particle in self.system.Get_bodies():
                if particle != other_particle:
                    distance = (particle.GetPos() - other_particle.GetPos()).Length()
                    force += (other_particle.GetPos() - particle.GetPos()) / distance * 0.1 / distance**2
            # Apply the force
            particle.AddForce(force)

# Add the custom gravitational force
custom_force = CustomGravitationalForce(mysystem)
mysystem.Add(custom_force)

# Run the simulation
while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.EndScene()
    mysystem.DoStepDynamics(0.01)
    myapplication.Synchronize(True)