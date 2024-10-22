import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: Multiple chains of ANCF cable elements, each connected to a box body.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        # Create a section, i.e. define thickness and material properties for the cable beam
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  # Set the diameter of the cable section to 15 mm
        msection_cable2.SetYoungModulus(0.01e9)  # Set the Young's modulus of the cable section (0.01 GPa)
        msection_cable2.SetRayleighDamping(0.0001)  # Set Rayleigh damping to zero for this section

        # Create a ChBuilderCableANCF helper object to facilitate the creation of ANCF beams
        builder = fea.ChBuilderCableANCF()

        # Generate multiple chains of beams
        for i in range(self.n_chains):
            # Number of elements increases with each chain
            n_elements = i * 2 + 10

            # Starting and ending points for each chain
            start_point = chrono.ChVector3d(i * 0.2, 0, -0.1)
            end_point = chrono.ChVector3d(i * 0.2 + 0.5, 0, -0.1)

            # Create the beam structure
            builder.BuildBeam(
                mesh,
                msection_cable2,
                n_elements,
                start_point,
                end_point
            )

            # Create a truss body (fixed reference frame)
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)

            # Create and initialize a hinge constraint to fix beam's end point to the truss
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)

            # Create a box body connected to the beam's endpoint
            mbody = chrono.ChBody()
            mbody.SetMass(1)
            mbody.SetPos(end_point)

            # Connect the box body to the beam endpoint
            constraint_point = fea.ChLinkPointFrame()
            constraint_point.Initialize(builder.GetLastBeamNodes().back(), mbody)
            system.Add(constraint_point)

            # Apply forces to the front node of the beam
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))

    def PrintBodyPositions(self, system):
        for i in range(self.n_chains):
            # Find the box body connected to the i-th chain
            mbody = system.Get_bodylist()[i + 1]  # Assuming truss bodies are added first
            print(f"Chain {i+1} box position: {mbody.GetPos()}")

# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh)
sys.Add(mesh)

# Add visualization for the FEM mesh:
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Add visualization for node positions:
visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamB.SetSymbolsThickness(0.006)
visualizebeamB.SetSymbolsScale(0.01)
visualizebeamB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamB)

# Create the Irrlicht visualization for rendering
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA cables')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()

# Set solver type and settings
solver = chrono.ChSolverMINRES()
if solver.GetType()== chrono.ChSolver.Type_MINRES :
    print( "Using MINRES solver" )
    sys.SetSolver(solver)
    solver.SetMaxIterations(200)
    solver.SetTolerance(1e-10)
    solver.EnableDiagonalPreconditioner(True)
    solver.EnableWarmStart(True)
    solver.SetVerbose(False)

# Set the timestepper for the simulation
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    model.PrintBodyPositions(sys)