import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os

# Output directory
out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"

# Create (if needed) the output directory
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory ")

# Create a Chrono::Engine physical system
sys = chrono.ChSystemSMC()

# Create a mesh, a container for groups of elements and their referenced nodes
mesh = fea.ChMesh()

# Add the created mesh to the physical system
sys.Add(mesh)

# Disable gravity for the system
mesh.SetAutomaticGravity(False)

# Define nodes to plot or load
nodePlotA = fea.ChNodeFEAxyz()
nodePlotB = fea.ChNodeFEAxyz()
nodesLoad = []

# Create interpolation functions for reference tracking (if needed)
ref_X = chrono.ChFunctionInterp()
ref_Y = chrono.ChFunctionInterp()

# Define load force vector
load_force = chrono.ChVector3d()

# Monitoring nodes and elements
mnodemonitor = fea.ChNodeFEAxyz()
melementmonitor = fea.ChElementShellBST()

if (True):
    # Define material properties
    density = 100
    E = 6e4
    nu = 0.0
    thickness = 0.01

    # Create isotropic Kirchhoff material elasticity object
    melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)

    # Create material object by assigning the elasticity property
    material = fea.ChMaterialShellKirchhoff(melasticity)
    material.SetDensity(density)

    # Define the mesh dimensions
    L_x = 1
    nsections_x = 40
    L_z = 1
    nsections_z = 40

    # Create list to hold the nodes
    mynodes = []

    # Create nodes for the mesh grid
    for iz in range(nsections_z + 1):
        for ix in range(nsections_x + 1):
            p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
            mnode = fea.ChNodeFEAxyz(p)
            mesh.AddNode(mnode)
            mynodes.append(mnode)

    # Create elements and associate nodes
    for iz in range(nsections_z):
        for ix in range(nsections_x):
            # Create first element
            melementA = fea.ChElementShellBST()
            mesh.AddElement(melementA)

            if (iz == 0 and ix == 1):
                ementmonitor = melementA

            # Define boundary nodes
            boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
            boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if (ix > 0) else None
            boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if (iz > 0) else None

            # Set nodes to the element
            melementA.SetNodes(
                mynodes[(iz) * (nsections_x + 1) + ix],
                mynodes[(iz) * (nsections_x + 1) + ix + 1],
                mynodes[(iz + 1) * (nsections_x + 1) + ix],
                boundary_1, boundary_2, boundary_3
            )

            # Add layer to the element
            melementA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

            # Create second element
            melementB = fea.ChElementShellBST()
            mesh.AddElement(melementB)

            # Define boundary nodes
            boundary_1 = mynodes[(iz) * (nsections_x + 1) + ix]
            boundary_2 = mynodes[(iz) * (nsections_x + 1) + ix + 2] if (ix < nsections_x - 1) else None
            boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix] if (iz < nsections_z - 1) else None

            # Set nodes to the element
            melementB.SetNodes(
                mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
                mynodes[(iz + 1) * (nsections_x + 1) + ix],
                mynodes[(iz) * (nsections_x + 1) + ix + 1],
                boundary_1, boundary_2, boundary_3
            )

            # Add layer to the element
            melementB.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

    # Fix upper nodes of the mesh
    for j in range(30):
        for k in range(30):
            mynodes[j * (nsections_x + 1) + k].SetFixed(True)

    # Create visualizations for shell elements
    mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
    mvisualizeshellA.SetSmoothFaces(True)  # Enable smooth faces
    mvisualizeshellA.SetWireframe(True)  # Enable wireframe mode
    mvisualizeshellA.SetShellResolution(2)
    mvisualizeshellA.SetBackfaceCull(True)  # Enable backface culling
    mesh.AddVisualShapeFEA(mvisualizeshellA)

    # Create visualization of FEM data
    mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
    mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
    mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
    mvisualizeshellB.SetSymbolsThickness(0.006)
    mesh.AddVisualShapeFEA(mvisualizeshellB)

# Create Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, .3, 1.3), chrono.ChVector3d(.5, -.3, .5))
vis.AddTypicalLights()

# Change solver to PardisoMKL
mkl_solver = mkl.ChSolverPardisoMKL()
mkl_solver.LockSparsityPattern(True)
sys.SetSolver(mkl_solver)

# Define time step for simulation
timestep = 0.005

# Final setup and system updates
sys.Setup()
sys.Update()

# Initialize interpolation function records (if required)
rec_X = chrono.ChFunctionInterp()
rec_Y = chrono.ChFunctionInterp()

# Simulation loop for running the visualization and dynamical system
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(timestep)