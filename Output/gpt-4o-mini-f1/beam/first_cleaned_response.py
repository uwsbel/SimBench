import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemSMC()


beam_mesh = chrono.ChBeamMesh()
beam_mesh.SetAutomaticGravity(True)


node = chrono.ChBeamSectionAdvanced()
node.SetDensity(7800)
node.SetYoungModulus(2e11)
node.SetShearModulus(8e10)
node.SetBeamRadius(0.01)
node.SetSectionTaperingRatio(0.5)
node.SetSectionRotation(chrono.ChVector3d(0, 0, 1), chrono.ChVector3d(1, 0, 0))
beam_mesh.AddSection(node)


N = 10
for i in range(N):
    beam_mesh.AddNode(chrono.ChBeamNodeEuler())
    if i > 0:
        beam_mesh.AddBeam(chrono.ChBeamEuler(beam_mesh.GetNode(i - 1), beam_mesh.GetNode(i)))


beam_mesh.SetMeshReferenceFrame(chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))


sys.Add(beam_mesh)


fixed_body = chrono.ChBody()
fixed_body.SetFixed(True)
sys.Add(fixed_body)


frame = chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)
fixed_body.AddAsset(chrono.ChVisualShapeFrame(frame))


box = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
box.SetColor(chrono.ChColor(0.6, 0.6, 0.6))
fixed_body.AddVisualShape(box)


contact_container = chrono.ChContactContainerSMC()
sys.SetContactContainer(contact_container)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(chrono.ChVector2i(1280, 720))
vis.SetWindowTitle('Beam FEM demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, -3), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, 3.5, -2.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)


time_step = 1e-3


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)