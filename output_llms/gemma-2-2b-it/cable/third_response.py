import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: A chain of beams composed of 10 ANCF cable element, with one end hinged to
# ground, moving under gravity alone.
# This model demonstrates the use of the utility class ChBuilderCableANCF.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        # Create a section, i.e. define thickness and material properties for the cable beam
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  # Set the diameter of the cable section to 15 mm
        msection_cable2.SetYoungModulus(0.01e9)  # Set the Young's modulus of the cable section (0.01 GPa)
        msection_cable2.SetRayleighDamping(0.0001)  # Set Rayleigh damping to zero for this section
        
        # Create a ChBuilderCableANCF helper object to facilitate the creation of ANCF beams
        builder = fea.ChBuilderCableANCF()
        
        # Loop to create the specified number of chains
        for i in range(n_chains):
            # Create a truss body (a fixed reference frame in the simulation)
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  

            # Create and initialize a hinge constraint to fix beam's end point to the truss
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)  

            # Create a new chain of beams
            # Define the starting and ending points of the chain
            start_point = chrono.ChVector3d(0, 0, -0.1)
            end_point = chrono.ChVector3d(0.5, 0, -0.1)
            
            # Create and add the beam elements to the chain
            builder.BuildBeam(
                mesh,  # The mesh to which the created nodes and elements will be added
                msection_cable2,  # The beam section properties to use
                10,  # Number of ANCF elements to create along the beam
                start_point,  # Starting point ('A' point) of the beam
                end_point  # Ending point ('B' point) of the beam
            )
            
            # Apply boundary conditions and loads:
            # Retrieve the end nodes of the beam and apply load/constraints
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))  # Apply forces to the front node

            # Add visualization for the newly created elements and chains
            visualizebeamA = chrono.ChVisualShapeFEA(mesh)
            visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # Display moments along the beam
            visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  # Set color scale for moment visualization
            visualizebeamA.SetSmoothFaces(True)  # Enable smooth faces for better visualization
            visualizebeamA.SetWireframe(False)  # Set to non-wireframe mode
            mesh.AddVisualShapeFEA(visualizebeamA)  

            # Add visualization for node positions
            visualizebeamB = chrono.ChVisualShapeFEA(mesh)
            visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # Display nodes as dots
            visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # No additional FEM data visualization
            visualizebeamB.SetSymbolsThickness(0.006)  # Set thickness of symbols
            visualizebeamB.SetSymbolsScale(0.01)  # Set scale of symbols
            visualizebeamB.SetZbufferHide(False)  # Ensure symbols are not hidden by z-buffer
            mesh.AddVisualShapeFEA(visualizebeamB)  

    # Print the positions of the end bodies of each chain at each step of the simulation
    def PrintBodyPositions(self):
        for i in range(n_chains):
            print(f"Chain {i+1}:")
            for j in range(len(self.system.GetBodies())):
                body = self.system.GetBodies()[j]
                print(f"  Body {j}: {body.GetPosition()}")

# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)  

# Add visualization for the FEM mesh:
# This allows visualization of the forces/moments in the beam elements:
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # Display moments along the beam
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  # Set color scale for moment visualization
visualizebeamA.SetSmoothFaces(True)  # Enable smooth faces for better visualization
visualizebeamA.SetWireframe(False)  # Set to non-wireframe mode
mesh.AddVisualShapeFEA(visualizebeamA)  

# Add visualization for node positions
visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # Display nodes as dots
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # No additional FEM data visualization
visualizebeamB.SetSymbolsThickness(0.006)  # Set thickness of symbols
visualizebeamB.SetSymbolsScale(0.01)  # Set scale of symbols
visualizebeamB.SetZbufferHide(False)  # Ensure symbols are not hidden by z-buffer
mesh.AddVisualShapeFEA(visualizebeamB)  

# Create the Irrlicht visualization for rendering
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach Irrlicht to the Chrono system
vis.SetWindowSize(1024, 768)  # Set the size of the rendering window
vis.SetWindowTitle('FEA cables')  # Set the title of the rendering window
vis.Initialize()  # Initialize the visualization
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the window
vis.AddSkyBox()  # Add a skybox for better aesthetics
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))  # Add a camera with specific position
vis.AddTypicalLights()  # Add typical lights for better illumination

# Set solver type and settings
solver = chrono.ChSolverMINRES() # Choose SparseQR solver
if solver.GetType()== chrono.ChSolver.Type_MINRES :
	print( "Using MINRES solver" )
	sys.SetSolver(solver)
	solver.SetMaxIterations(200)
	solver.SetTolerance(1e-10)
	solver.EnableDiagonalPreconditioner(True)
	solver.EnableWarmStart(True)  # IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
	solver.SetVerbose(False)
# Set the timestepper for the simulation
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop
while vis.Run():
    vis.BeginScene()  # Begin scene rendering
    vis.Render()  # Render the scene
    vis.EndScene()  # End scene rendering
    sys.DoStepDynamics(0.01)  # Advance the simulation by one step with a time step of 0.01 seconds
    
    # Print the positions of the end bodies of each chain at each step of the simulation
    model.PrintBodyPositions()