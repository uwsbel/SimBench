import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        self.end_bodies = []

        # Create a section for the cable beam
        msection_cable = fea.ChBeamSectionCable()
        msection_cable.SetDiameter(0.015)
        msection_cable.SetYoungModulus(0.01e9)
        msection_cable.SetRayleighDamping(0.0001)

        for i in range(self.n_chains):
            # Create a truss body (fixed reference frame)
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            self.system.Add(mtruss)

            # Create a ChBuilderCableANCF helper object
            builder = fea.ChBuilderCableANCF()

            # Number of ANCF elements increases with each chain
            n_elements = 10 + i * 2

            # Set relative positions for starting and ending points
            start_point = chrono.ChVector3d(i * 0.1, 0, -0.1)
            end_point = chrono.ChVector3d(i * 0.1 + 0.5, 0, -0.1)

            # Build the beam
            builder.BuildBeam(self.mesh, msection_cable, n_elements, start_point, end_point)

            # Apply boundary conditions and loads
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))

            # Create a hinge constraint
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            self.system.Add(constraint_hinge)

            # Create a box body at the end of the beam
            end_body = chrono.ChBodyEasyBox(0.05, 0.05, 0.05, 1000)
            end_body.SetPos(end_point)
            self.system.Add(end_body)
            self.end_bodies.append(end_body)

            # Connect beam endpoint to the box
            constraint_sphere = chrono.ChLinkMateSpherical()
            constraint_sphere.Initialize(builder.GetLastBeamNodes().front(), end_body,
                                         chrono.ChFrameD(end_point))
            self.system.Add(constraint_sphere)

        # Add visualization for the FEM mesh
        visualizebeamA = chrono.ChVisualShapeFEA(self.mesh)
        visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
        visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
        visualizebeamA.SetSmoothFaces(True)
        visualizebeamA.SetWireframe(False)
        self.mesh.AddVisualShapeFEA(visualizebeamA)

        # Add visualization for node positions
        visualizebeamB = chrono.ChVisualShapeFEA(self.mesh)
        visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
        visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
        visualizebeamB.SetSymbolsThickness(0.006)
        visualizebeamB.SetSymbolsScale(0.01)
        visualizebeamB.SetZbufferHide(False)
        self.mesh.AddVisualShapeFEA(visualizebeamB)

    def PrintBodyPositions(self):
        for i, body in enumerate(self.end_bodies):
            print(f"Chain {i + 1} end body position: {body.GetPos()}")

# Initialize the physical system and mesh container
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh)
sys.Add(mesh)

# Create the Irrlicht visualization
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
    model.PrintBodyPositions()  # Print positions of end bodies at each step