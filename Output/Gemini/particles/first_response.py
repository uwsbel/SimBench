import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random

# ---------------------------------------------------------------------
# Create the simulation system and set default parameters
# ---------------------------------------------------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Initially disable default gravity

# Set simulation time step
system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)
system.SetStep(0.01)

# ---------------------------------------------------------------------
# Create a particle emitter
# ---------------------------------------------------------------------
class ParticleEmitter:
    def __init__(self, system, flow_rate, position, velocity_range, shape_range):
        self.system = system
        self.flow_rate = flow_rate
        self.position = position
        self.velocity_range = velocity_range
        self.shape_range = shape_range
        self.time_since_last_emission = 0

    def Update(self, time_step):
        # Calculate number of particles to emit based on flow rate
        num_particles = int(self.flow_rate * time_step)

        # Emit particles
        for i in range(num_particles):
            self.EmitParticle()

        # Update time since last emission
        self.time_since_last_emission += time_step

    def EmitParticle(self):
        # Randomize particle properties
        mass = random.uniform(0.1, 1.0)
        radius = random.uniform(self.shape_range[0], self.shape_range[1])
        position = self.position + chrono.ChVectorD(random.uniform(-0.5, 0.5), 
                                                    random.uniform(-0.5, 0.5), 
                                                    random.uniform(-0.5, 0.5))
        velocity = chrono.ChVectorD(random.uniform(self.velocity_range[0], self.velocity_range[1]),
                                   random.uniform(self.velocity_range[0], self.velocity_range[1]),
                                   random.uniform(self.velocity_range[0], self.velocity_range[1]))

        # Create particle body
        particle = chrono.ChBodyEasySphere(radius, 1000)  # Adjust density as needed
        particle.SetPos(position)
        particle.SetPos_dt(velocity)

        # Add particle to the system
        self.system.Add(particle)

# Initialize the particle emitter
emitter = ParticleEmitter(system,
                          flow_rate=50,  # Emit 50 particles per second
                          position=chrono.ChVectorD(0, 0, 0),
                          velocity_range=(-2, 2),
                          shape_range=(0.05, 0.1))

# ---------------------------------------------------------------------
# Custom gravitational force function
# ---------------------------------------------------------------------
def CustomGravity(body1, body2, G):
    """Calculates and applies custom gravitational force between two bodies.

    Args:
        body1 (chrono.ChBody): The first body.
        body2 (chrono.ChBody): The second body.
        G (float): Gravitational constant.
    """
    # Calculate the distance between the bodies
    dir = body2.GetPos() - body1.GetPos()
    dist = dir.Length()
    if dist < 1e-3:  # Avoid division by zero
        dist = 1e-3

    # Calculate the gravitational force magnitude
    force_mag = G * body1.GetMass() * body2.GetMass() / (dist**2)

    # Apply the force to both bodies
    force_vec = force_mag * dir / dist  # Normalize direction vector
    body1.AddForce(force_vec)
    body2.AddForce(-force_vec)

# ---------------------------------------------------------------------
# Create the Irrlicht visualization
# ---------------------------------------------------------------------
application = chronoirr.ChIrrApp(system, 'Particle Attraction', chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(1, 1, 3))
application.AssetBindAll()
application.AssetUpdateAll()

# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    # Update the particle emitter
    emitter.Update(system.GetStep())

    # Apply custom gravity between all particle pairs
    for i in range(system.Get_bodylist().size()):
        for j in range(i + 1, system.Get_bodylist().size()):
            CustomGravity(system.Get_bodylist().at(i), system.Get_bodylist().at(j), 1)

    system.DoStepDynamics(system.GetStep())
    application.EndScene()