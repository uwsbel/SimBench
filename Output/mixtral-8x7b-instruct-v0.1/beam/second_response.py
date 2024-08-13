import pychrono as chrono # Import the main PyChrono module.
import pychrono.fea as fea # Import the finite element analysis (FEA) module.
import pychrono.pardisomkl as mkl # Import the Pardiso MKL linear solver module.
import pychrono.irrlicht as chronoirr # Import the Irrlicht visualization module.

print("Example: PyChrono using beam finite elements") # Print an introductory statement.

# Create the physical system that will be simulated.
sys = chrono.ChSystemSMC()

# Create a mesh, which is a container for elements and their referenced nodes.
mesh = fea.ChMesh()

# Create a section object for beam properties. This will define the characteristics of all beams that use this section.
msection = fea.ChBeamSectionEulerAdvanced()

# Set the width and height of the rectangular section of the beam.
beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz) # Set the cross-sectional shape and size of the beam.

# Set the material properties of the beam.
msection.SetYoungModulus(0.01e9) # Young's modulus, a measure of the stiffness of the material.
msection.SetShearModulus(0.01e9 * 0.3) # Shear modulus, also related to the rigidity of the material.
msection.SetRayleighDamping(0.000) # Damping coefficient for Rayleigh damping, affecting the dynamic response.
msection.SetCentroid(0, 0.02) # Set the position of the centroid.
msection.SetShearCenter(0, 0.1) # Set the position of the shear center.
msection.SetSectionRotation(45 * chrono.CH_RAD_TO_DEG) # Set the rotation angle of the section around its axis (in degrees).

# Define the length of the beam elements.
beam_L = 0.1

# Create nodes for the positions that will be used for beams.
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, -0.1))) # Node at origin.
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0.2, 0, -0.1))) # Node at beam length.

# Create a builder object for Euler-Bernoulli beams.
builder = fea.ChBuilderBeamEuler()

# Use the builder to create a beam from point A to point B with 5 elements.
builder.SetUp(hnode1, hnode2, msection, 5)
beam = builder.BuildBeam()
mesh.AddElement(beam)

# Fix the last node of the created beam.
beam.GetLastBeamNodes().back().SetFixed(True)

# Apply a force to the first node of the created beam section.
beam.GetNode(0).SetForce(chrono.ChVector3d(0, -1, 0))

# Create a fixed truss, which is a rigid body that won't move.
mtruss = chrono.ChBody()
mtruss.SetFixed(True) # Fix the truss so it won't move.
sys.Add(mtruss) # Add the truss to the physical system.

# Create and initialize a constraint that connects node 2 to the fixed truss.
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode2, mtruss, False, hnode2.Frame(), hnode2.Frame())
sys.Add(constr_bc) # Add the constraint to the system.
constr_bc.SetConstrainedCoords(True, True, True, # Constrain x, y, z translations.
                                True, True, True) # Constrain Rx, Ry, Rz rotations.

# Replace the direct setting of a node as fixed with constraints to fix node 1.
constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())
sys.Add(constr_d) # Add the constraint to the system.
constr_d.SetConstrainedCoords(True, False, False, # Constrain x translation only.
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
vis.AttachSystem(sys) # Attach the simulation system to the visual system.
vis.SetWindowSize(1024, 768) # Set the window size.
vis.SetWindowTitle('FEA beams') # Set the window title.
vis.Initialize() # Initialize the visual system.
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) # Add the Chrono logo.
vis.AddSkyBox() # Add a skybox.
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2)) # Add a camera.
vis.AddTypicalLights() # Add typical lights for the scene.

# Change the default solver to the MKL Pardiso solver, which is more precise for FEA.
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver) # Set the MKL Pardiso solver for the system.

# Simulation loop.
while vis.Run():
    vis.BeginScene() # Begin the scene.
    vis.Render() # Render the scene.
    vis.EndScene() # End the scene.
    sys.DoStepDynamics(0.001) # Perform one step of simulation with a step size of 0.001 seconds.