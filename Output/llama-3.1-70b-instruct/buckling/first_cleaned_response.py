import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import numpy as np


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)  
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)  


beam_length = 10
beam_width = 0.1
beam_height = 0.1
beam_density = 7850
beam_young_modulus = 2e11
beam_poisson_ratio = 0.3

beam = fea.ChBeamSectionEuler()
beam.SetDensity(beam_density)
beam.SetYoungModulus(beam_young_modulus)
beam.SetPoissonRatio(beam_poisson_ratio)
beam.SetWidth(beam_width)
beam.SetHeight(beam_height)


beam_mesh = fea.ChBeamMesh()
beam_mesh.SetSection(beam)
beam_mesh.SetLength(beam_length)
beam_mesh.SetNumElements(100)


beam_body = fea.ChBodyEasyBeam(beam_mesh)
beam_body.SetMass(beam_density * beam_length * beam_width * beam_height)
beam_body.SetInertiaXX(chrono.ChVectorD(1/12 * beam_density * beam_length * beam_width * beam_height, 1/12 * beam_density * beam_length * beam_width * beam_height, 1/12 * beam_density * beam_length * beam_width * beam_height))
beam_body.SetInertiaXY(chrono.ChVectorD(0, 0, 0))
beam_body.SetInertiaXZ(chrono.ChVectorD(0, 0, 0))
beam_body.SetInertiaYY(chrono.ChVectorD(1/12 * beam_density * beam_length * beam_width * beam_height, 1/12 * beam_density * beam_length * beam_width * beam_height, 1/12 * beam_density * beam_length * beam_width * beam_height))
beam_body.SetInertiaYZ(chrono.ChVectorD(0, 0, 0))
beam_body.SetInertiaZZ(chrono.ChVectorD(1/12 * beam_density * beam_length * beam_width * beam_height, 1/12 * beam_density * beam_length * beam_width * beam_height, 1/12 * beam_density * beam_length * beam_width * beam_height))


system.Add(beam_body)


def motor_function(time):
    return 0.1 * np.sin(time)


motor = chrono.ChFunction_MotorLinear()
motor.SetFunction(motor_function)
beam_body.AddMotor(motor)


constraint = chrono.ChLinkLockLock()
constraint.Initialize(beam_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
system.Add(constraint)


irrlicht_module = chronoirr.ChVisualSystemIrrlicht()
irrlicht_module.SetWindowSize(800, 600)
irrlicht_module.SetWindowTitle("Beam Buckling Simulation")
irrlicht_module.SetCamera(chrono.ChVectorD(0, 0, 10), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))


solver = chrono.ChSolverNSC()
solver.SetMaxItersPerStep(100)
solver.SetTol(1e-6)
timestepper = chrono.ChTimestepperHHT(0.7)


system.SetSolver(solver)
system.SetTimestepper(timestepper)


irrlicht_module.AddSkyBox()
irrlicht_module.AddTypicalLights()
irrlicht_module.AddCameraVerticalAll()
irrlicht_module.AddLightWithShadow(chrono.ChVectorD(1, 1, 2), chrono.ChVectorD(0, 0, 0), 10, 10, 10, 100, 100, 100)
irrlicht_module.SetShowInfos(True)
irrlicht_module.SetShowCollisionModels(True)

while irrlicht_module.Run():
    system.DoStepDynamics(timestepper.GetTimeStep())
    irrlicht_module.BeginScene()
    irrlicht_module.DrawAll()
    irrlicht_module.EndScene()