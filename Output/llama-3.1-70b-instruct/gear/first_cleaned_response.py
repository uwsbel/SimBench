import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine


chrono.SetChronoDataPath('./')


mysystem = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht(mysystem)


vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Epicyclic Gears Simulation')
vis.SetCamera(chrono.ChVectorD(0, 0, 2), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))


truss = chrono.ChBodyEasyBox(mysystem, 1, 1, 1, 1000, True, True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetBodyFixed(True)
mysystem.Add(truss)


bar = chrono.ChBodyEasyBox(mysystem, 1, 0.1, 0.1, 1000, True, True)
bar.SetPos(chrono.ChVectorD(0, 0, 1))
bar.SetRot(chrono.ChQuaternionD(chrono.ChVectorD(0, 1, 0), chrono.ChAngleD(chrono.CH_C_PI / 2)))
mysystem.Add(bar)


gear_motor = chrono.ChLinkMotorRotationSpeed()
gear_motor.Initialize(bar, truss, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1)))
gear_motor.SetSpeed(chrono.ChVectorD(0, 0, 1))
mysystem.Add(gear_motor)


gear1 = chrono.ChBodyEasyCylinder(mysystem, 0.5, 0.1, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(1, 0, 1))
gear1.SetRot(chrono.ChQuaternionD(chrono.ChVectorD(0, 1, 0), chrono.ChAngleD(chrono.CH_C_PI / 2)))
mysystem.Add(gear1)

gear2 = chrono.ChBodyEasyCylinder(mysystem, 0.5, 0.1, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(-1, 0, 1))
gear2.SetRot(chrono.ChQuaternionD(chrono.ChVectorD(0, 1, 0), chrono.ChAngleD(chrono.CH_C_PI / 2)))
mysystem.Add(gear2)


contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.5)
contact_material.SetDampingF(0.1)
contact_material.SetCompliance(0.0001)
contact_material.SetComplianceT(0.0001)


contact_truss_gear1 = chrono.ChLinkContactMaterial()
contact_truss_gear1.SetMaterial(contact_material)
contact_truss_gear1.SetMode(chrono.ContactLinkMode.TWO_WAY)
contact_truss_gear1.SetName('truss_gear1')
truss.AddLink(contact_truss_gear1)
gear1.AddLink(contact_truss_gear1)

contact_truss_gear2 = chrono.ChLinkContactMaterial()
contact_truss_gear2.SetMaterial(contact_material)
contact_truss_gear2.SetMode(chrono.ContactLinkMode.TWO_WAY)
contact_truss_gear2.SetName('truss_gear2')
truss.AddLink(contact_truss_gear2)
gear2.AddLink(contact_truss_gear2)


contact_gear1_gear2 = chrono.ChLinkContactMaterial()
contact_gear1_gear2.SetMaterial(contact_material)
contact_gear1_gear2.SetMode(chrono.ContactLinkMode.TWO_WAY)
contact_gear1_gear2.SetName('gear1_gear2')
gear1.AddLink(contact_gear1_gear2)
gear2.AddLink(contact_gear1_gear2)


truss_mat = chrono.ChVisualMaterial()
truss_mat.SetDiffuseColor(chrono.ChColor(0.5, 0.5, 0.5))
truss.SetMaterial(truss_mat)

bar_mat = chrono.ChVisualMaterial()
bar_mat.SetDiffuseColor(chrono.ChColor(0.8, 0.8, 0.8))
bar.SetMaterial(bar_mat)

gear1_mat = chrono.ChVisualMaterial()
gear1_mat.SetDiffuseColor(chrono.ChColor(0.2, 0.2, 0.2))
gear1.SetMaterial(gear1_mat)

gear2_mat = chrono.ChVisualMaterial()
gear2_mat.SetDiffuseColor(chrono.ChColor(0.2, 0.2, 0.2))
gear2.SetMaterial(gear2_mat)


vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    mysystem.DoStepDynamics(0.01)