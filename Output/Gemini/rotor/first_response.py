import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import numpy as np

# --------------------------------------------------------------------
# Create the Chrono system
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Set gravity

# --------------------------------------------------------------------
# Define material properties
material = fea.ChMaterialShellIsoElastic(fea.ChElasticityReissner())
material.SetYoungModulus(200e9)  # Steel
material.SetPoissonRatio(0.3)
material.SetDensity(7800)

# --------------------------------------------------------------------
# Create the IGA beam
beam_length = 1.0  # Length of the beam
beam_radius = 0.025  # Radius of the beam

# Define the IGA beam geometry (2nd order NURBS curve with 3 control points)
control_points = [
    chrono.ChVectorD(0, 0, 0),
    chrono.ChVectorD(beam_length / 2.0, 0, 0),
    chrono.ChVectorD(beam_length, 0, 0),
]
knots = [0, 0, 0, 1, 1, 1]
degree = 2
beam_geometry = fea.ChLineNurbs(control_points, knots, degree)

# Create the IGA beam section
beam_section = fea.ChBeamSectionCosseratEasyCS(beam_radius, material)

# Create the IGA beam element mesh
n_elements = 20
beam_mesh = fea.ChMesh()
fea.ChMeshUtilities.CreateBeamFromLine(
    beam_mesh,  # Mesh to be populated
    beam_geometry,  # Beam geometry
    beam_section,  # Beam section
    n_elements,  # Number of elements
)

# Create the IGA beam
beam = fea.ChBeamIGA(beam_mesh)
beam.SetInitialPos(chrono.ChVectorD(0, 0, 0))
system.Add(beam)

# --------------------------------------------------------------------
# Create the flywheel
flywheel_mass = 1.0  # Mass of the flywheel
flywheel_radius = 0.1  # Radius of the flywheel
flywheel_inertia = 0.5 * flywheel_mass * flywheel_radius**2

flywheel = chrono.ChBodyEasyCylinder(
    flywheel_radius, 0.01, flywheel_mass  # Height for visualization
)
flywheel.SetInertiaXX(chrono.ChVectorD(flywheel_inertia, flywheel_inertia, flywheel_inertia))
flywheel.SetPos(chrono.ChVectorD(beam_length / 2.0, 0, 0))
system.Add(flywheel)

# --------------------------------------------------------------------
# Connect the beam and flywheel
beam_flywheel_link = chrono.ChLinkMateGeneric()
beam_flywheel_link.Initialize(
    beam,
    flywheel,
    False,
    chrono.ChFrameD(chrono.ChVectorD(beam_length / 2.0, 0, 0)),
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)),
    chrono.ChFrameD(chrono.ChVectorD(1, 0, 0)),
    chrono.ChFrameD(chrono.ChVectorD(1, 0, 0)),
)
system.Add(beam_flywheel_link)

# --------------------------------------------------------------------
# Apply motor to one end of the beam
motor_torque = 10.0  # Motor torque

motor_function = chrono.ChFunction_Const(motor_torque)

motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(
    beam,
    system.Get_body_list()[0],  # Connect to the ground
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI / 2)),
)
motor.SetMotorFunction(motor_function)
system.Add(motor)

# --------------------------------------------------------------------
# Set up the solver
system.SetSolverType(chrono.ChSolver.Type_BARZ)
system.SetSolverMaxIterations(100)
system.SetSolverMaxIterationsBilateral(100)
system.SetTol(1e-10)
system.SetTolForce(1e-8)

# --------------------------------------------------------------------
# Set up visualization
application = irr.ChIrrApp(system)
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(
    pos=chrono.ChVectorD(0.5, 0.5, 1.0), look_at=chrono.ChVectorD(0.5, 0, 0)
)

# FEM visualization
mesh_asset = fea.ChVisualizationFEAmesh(beam_mesh)
mesh_asset.SetFEMdataType(fea.ChVisualizationFEAmesh.EDataType.E_DENSITY)
mesh_asset.SetShrinkElements(True, 0.85)
mesh_asset.SetWireframe(True)
beam.AddAsset(mesh_asset)

# Run the simulation
application.AssetBindAll()
application.AssetUpdateAll()
application.SetTimestep(0.001)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()