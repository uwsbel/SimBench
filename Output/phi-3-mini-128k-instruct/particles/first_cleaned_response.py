import pychrono as chrono

import pychrono.openvr as vr

import pychrono.visual as vis

import pychrono.analytic_motion as am

import pychrono.analytic_force as af

import numpy as np

import random

import irrlicht as ir

import irrlicht.keyboard as keyboard




chrono.ChSystemContext().SetGravity(chrono.ChVectorD(0, -9.81, 0))




particle_emitter = chrono.ChParticleEmitter()

particle_emitter.SetShape(chrono.ChSphereShape(1.0))

particle_emitter.SetPosition(chrono.ChVectorD(0, 100, 0))

particle_emitter.SetVelocity(chrono.ChVectorD(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10)))

particle_emitter.SetOrientation(chrono.ChQuaternionD(random.uniform(0, 2*np.pi), random.uniform(0, np.pi), random.uniform(0, 2*np.pi)))




system = chrono.ChSystemContext()

system.AddParticleEmitter(particle_emitter)




def custom_gravitational_force(particle, system):

    for other_particle in system.GetParticles():

        if particle is not other_particle:

            distance = particle.GetPosition() - other_particle.GetPosition()

            force_magnitude = chrono.ChVectorD(distance.x, distance.y, distance.z) * chrono.ChVectorD(0, -1, 0).Length()

            force = force_magnitude * chrono.ChVectorD(0, 1, 0)

            particle.AddForce(force)

            other_particle.AddForce(-force)




scene = vis.ChVisualScene()

scene.AddActor(vis.ChActor(system.GetParticles()[0].GetShape()))




application = vr.ChApplicationVR_Full(chrono.ChSystemContext())

application.SetWindowTitle("Particle Simulation")

application.SetWindowSize(1024, 768)

application.AddScene(scene)

application.AddTimer(1/60.0)




while application.GetDevice().IsOpened():

    application.BeginScene()

    application.DrawAll()

    application.EndScene()

    application.Update()


    

    for particle in system.GetParticles():

        custom_gravitational_force(particle, system)


    

    keyboard.wait_for_key(application)

    if keyboard.is_pressed('q'):

        application.Close()