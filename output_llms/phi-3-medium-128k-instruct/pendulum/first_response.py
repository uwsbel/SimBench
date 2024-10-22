import pychrono

import pychrono.Irrlicht

import pychrono.utils as utils


# Initialize the PyChrono environment

chrono = pychrono.ChSystemNSC()


# Add the ground body

ground = pychrono.ChBodyAO(pychrono.Vec3(0, 0, 0), pychrono.ChVectorD(0, 0, 0), pychrono.ChBodyTypes.STATIC_SPHERE)

ground.SetMass(1e6)  # Set a large mass to simulate an infinite ground

ground.SetInertia(pychrono.ChVectorD(0, 0, 0))

chrono.AddBody(ground)


# Create the pendulum body

pendulum = pychrono.ChBodyAO(pychrono.ChVectorD(0, 0, 0), pychrono.ChVectorD(0, 0, 0), pychrono.ChBodyTypes.DYNAMIC_SPHERE)

pendulum.SetMass(1)  # Set the mass of the pendulum

pendulum.SetInertia(pychrono.ChVectorD(0, 0, 0))

pendulum.SetPosition(pychrono.ChVectorD(0, 0, 1))  # Set the initial position

pendulum.SetVelocity(pychrono.ChVectorD(0, 0, 0))  # Set the initial velocity

chrono.AddBody(pendulum)


# Create a revolute joint to connect the pendulum to the ground

joint = pychrono.ChRevoluteJoint(pendulum, ground, pychrono.ChVectorD(0, 0, 1), pychrono.ChVectorD(0, 0, 0))

joint.SetMaxForce(1e5)  # Set the maximum force allowed by the joint

chrono.AddJoint(joint)


# Set up the visualization system

irrlicht = pychrono.IrrlichtVisualizer(chrono)


# Set up the logging system

log = pychrono.ChLog(pychrono.ChLogLevel.LOG_ALL)

log.AddSystem(chrono)


# Main simulation loop

while chrono.GetChTime() < 10:  # Run for 10 seconds

    chrono.DoStepDynamics(0.01)  # Time step

    irrlicht.Draw()  # Render the simulation

    log.Print(chrono.GetChTime())  # Log the current time

    # Log the pendulum's position and velocity

    position = pendulum.GetPos()

    velocity = pendulum.GetLinearVel()

    print(f"Time: {chrono.GetChTime():.2f}, Position: {position}, Velocity: {velocity}")


# Clean up

irrlicht.End()

chrono.Destroy()