import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardiso as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using Euler-Bernoulli beams")

# Create the physical system that will be simulated.
sys = chrono.ChSystemSMC()

# Create a mesh, which is a container for elements and their referenced nodes.
mesh = fea.ChMesh()

# Create a section object for Euler-Bernoulli beam properties.
builder = fea.ChBuilderBeamEuler()
builder.SetNodes(chrono.ChFramed(chrono.ChVector3d(0, 0, -0.1)), chrono.ChFramed(chrono.ChVector3d(0.2, 0, -0.1)))
builder.SetSection(msection)
beam = builder.BuildBeam()

# Fix the last node of the created beam using ChLinkMateGeneric
builder.GetLastBeamNodes().back().SetFixed(True)

# Apply a force to the first node of the created beam section.
beam.GetNodes()[0].SetForce(chrono.ChVector3d(0, -1, 0))

# Create a fixed truss, which is a rigid body that won't move.
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

# Create and initialize a constraint that connects node 1 to the fixed truss.
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(beam.GetNodes()[0], mtruss, False, beam.GetNodes()[0].Frame(), beam.GetNodes()[0].Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, # Constrain x, y, z translations.
                                True, True, True) # Constrain Rx, Ry, Rz rotations.

# Create and initialize a constraint that connects node 1 to the fixed truss.
constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(beam.GetNodes()[0], mtruss, False, beam.GetNodes()[0].Frame(), beam.GetNodes()[0].Frame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True, # Constrain only y, z translations.
                              False, False, False) # Do not constrain any rotations.

# Disable the automatic gravity for FEA elements in this demonstration.
mesh.SetAutomaticGravity(False)

# Add the mesh to the physical system.
sys.Add(mesh)

# Add visualization for the beams in the mesh.
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ) # Visualize the bending moments.
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4) # Set color scale limits.
visualizebeamA.SetSmoothFaces(True) # Smooth the faces for visualization.
visualizebeamA.SetWireframe(False) # Disable wireframe mode.
mesh.AddVisualShapeFEA(visualizebeamA)

# Add visualization for the nodes in the mesh.
visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS) # Visualize coordinate systems at nodes.
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE) # No data type visualization.
visualizebeamC.SetSymbolsThickness(0.006) # Set symbol thickness.
visualizebeamC.SetSymbolsScale(0.01) # Set symbol scale.
visualizebeamC.SetZbufferHide(False) # Do not hide symbols behind objects.
mesh.AddVisualShapeFEA(visualizebeamC)

# Create an Irrlicht visualization window.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# Change the default solver to the MKL Pardiso solver, which is more precise for FEA.
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Simulation loop.
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)