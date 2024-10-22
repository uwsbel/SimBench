import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using beam finite elements")

# Create the physical system that will be simulated.
sys = chrono.ChSystemSMC()

# Create a mesh, which is a container for elements and their referenced nodes.
mesh = fea.ChMesh()

# --- Euler-Bernoulli Beam Setup ---
# Use ChBuilderBeamEuler for easier beam creation
builder = fea.ChBuilderBeamEuler()

# Create a beam from point A to point B
builder.BuildBeam(
    mesh,  # The mesh to add the beam to
    chrono.ChVector3d(0, 0, -0.1),  # Start point A
    chrono.ChVector3d(0.2, 0, -0.1),  # End point B
    chrono.ChVector3d(0, 1, 0),  # 'Y' up direction
    5,  # Number of elements
)

# Fix the last node of the created beam
builder.GetLastBeamNodes().back().SetFixed(True)

# Apply a force to the first node of the created beam section
builder.GetFirstBeamNodes().front().SetForce(chrono.ChVector3d(0, -1, 0))

# --- End Euler-Bernoulli Beam Setup ---

# Create nodes for the positions that will be used for beams.
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0.1, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0.2, 0, 0)))

# Add nodes to the mesh.
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)

# Create the first beam element and set its properties.
belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
# Assign the section properties to this beam element.
mesh.AddElement(belement1)

# Create the second beam element and set its properties.
belement2 = fea.ChElementBeamEuler()
belement2.SetNodes(hnode2, hnode3)
# Assign the section properties to this beam element.
mesh.AddElement(belement2)

# --- Modify Node Fixing Approach ---
# Create a fixed truss, which is a rigid body that won't move.
mtruss = chrono.ChBody()
sys.Add(mtruss)

# Create and initialize a constraint that connects node 1 to the fixed truss.
constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(True, True, True,  # Constrain x, y, z translations.
                              True, True, True)  # Constrain Rx, Ry, Rz rotations.

# --- End Node Fixing Modification ---

# Apply a force to node 2: 4 N in the x-direction and 2 N in the y-direction.
hnode2.SetForce(chrono.ChVector3d(4, 2, 0))

# Apply a torque to node 3: -0.04 Nm in the y-direction.
hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))

# Disable the automatic gravity for FEA elements in this demonstration.
mesh.SetAutomaticGravity(False)

# Add the mesh to the physical system.
sys.Add(mesh)

# Add visualization for the beams in the mesh.
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Add visualization for the nodes in the mesh.
visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetSymbolsScale(0.01)
visualizebeamC.SetZbufferHide(False)
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

# Change the default solver to the MKL Pardiso solver.
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Simulation loop.
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)