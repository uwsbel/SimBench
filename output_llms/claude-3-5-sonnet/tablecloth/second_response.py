import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os
import numpy as np

# Output directory setup
out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory")

# Create Chrono physical system
sys = chrono.ChSystemSMC()

# Create and add mesh to the system
mesh = fea.ChMesh()
sys.Add(mesh)

# Material properties
density = 100
E = 6e4
nu = 0.0
thickness = 0.01

# Create material
melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material = fea.ChMaterialShellKirchhoff(melasticity)
material.SetDensity(density)

# Mesh dimensions
L_x, L_z = 1, 1
nsections_x, nsections_z = 40, 40

# Create nodes
mynodes = []
for iz in range(nsections_z + 1):
    for ix in range(nsections_x + 1):
        p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
        mnode = fea.ChNodeFEAxyz(p)
        mesh.AddNode(mnode)
        mynodes.append(mnode)

# Node monitoring and loading setup
nodePlotA = mynodes[1]
nodePlotB = mynodes[(nsections_z//2) * (nsections_x + 1) + (nsections_x//2)]
nodesLoad = [mynodes[-2], mynodes[-1]]

# Interpolation functions for reference tracking
def ref_X(t):
    return 0

def ref_Y(t):
    if t <= 0.5:
        return -0.05 * (1 - np.cos(2 * np.pi * t))
    else:
        return -0.1

# Load force vector
load_force = chrono.ChVector3d(0, -50, 0)

# Create elements
mnodemonitor = None
melementmonitor = None
for iz in range(nsections_z):
    for ix in range(nsections_x):
        melementA = fea.ChElementShellBST()
        boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
        boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if ix > 0 else None
        boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if iz > 0 else None

        melementA.SetNodes(mynodes[iz * (nsections_x + 1) + ix], mynodes[iz * (nsections_x + 1) + ix + 1],
                           mynodes[(iz + 1) * (nsections_x + 1) + ix], boundary_1, boundary_2, boundary_3)
        melementA.AddLayer(thickness, 0, material)
        mesh.AddElement(melementA)

        if iz == 0 and ix == 1:
            melementmonitor = melementA

        melementB = fea.ChElementShellBST()
        boundary_1 = mynodes[iz * (nsections_x + 1) + ix]
        boundary_2 = mynodes[iz * (nsections_x + 1) + ix + 2] if ix < nsections_x - 1 else None
        boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix] if iz < nsections_z - 1 else None

        melementB.SetNodes(mynodes[(iz + 1) * (nsections_x + 1) + ix + 1], mynodes[(iz + 1) * (nsections_x + 1) + ix],
                           mynodes[iz * (nsections_x + 1) + ix + 1], boundary_1, boundary_2, boundary_3)
        melementB.AddLayer(thickness, 0, material)
        mesh.AddElement(melementB)

# Fix upper nodes
for j in range(30):
    for k in range(30):
        mynodes[j * (nsections_x + 1) + k].SetFixed(True)

# Create visualizations for shell elements
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)
# mvisualizeshellA.SetSmoothFaces(True)
# mvisualizeshellA.SetWireframe(True)
# mvisualizeshellA.SetBackfaceCull(True)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizeshellB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizeshellB.SetSymbolsThickness(0.006)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# Irrlicht visualization system setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Shells FEA test: triangle BST elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.3, 1.3), chrono.ChVector3d(0.5, -0.3, 0.5))
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

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)