import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# Change Beam Length:
beam_L = 10  # Modified beam length

# Change Beam Thickness:
beam_ro = 0.060  # Modified outer radius
beam_ri = 0.055  # Modified inner radius

CH_PI = 3.1456

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

# Note: SetAutomaticGravity is redundant here as we set gravitational acceleration explicitly later.
# mesh.SetAutomaticGravity(True, 2)

# Change Gravitational Acceleration:
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))  # Modified gravity

# Create a section, i.e. thickness and material properties
# for beams. This will be shared among some beams.

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(CH_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
# Note: Iyy and Izz are already set in 'minertia'. No need to set them here again.
# melasticity.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
# melasticity.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)

msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)  # SetAsCircularSection(..) would overwrite Ixx Iyy J etc.

# Use the ChBuilderBeamIGA tool for creating a straight rod
# divided in Nel elements:

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,  # the mesh to put the elements in
                  msection,  # section of the beam
                  20,  # number of sections (spans)
                  chrono.ChVector3d(0, 0, 0),  # start point
                  chrono.ChVector3d(beam_L, 0, 0),  # end point
                  chrono.VECT_Y,  # suggested Y direction of section
                  1)  # order (3 = cubic, etc)

node_mid = builder.GetLastBeamNodes()[m.floor(builder.GetLastBeamNodes().size() / 2.0)]

# Create the flywheel and attach it to the center of the beam

# Change Flywheel Dimensions:
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1,
                                        7800)  # R, h, density # Modified flywheel radius
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),  # flywheel initial center (plus Y offset)
                       chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Z))
    # flywheel initial alignment (rotate 90° so cylinder axis is on X)
)
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Create the truss
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Create the end bearing
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes().back(),
                   truss,
                   chrono.ChFrameD(builder.GetLastBeamNodes().back().GetPos())
                   )
sys.Add(bearing)

# Create the motor that rotates the beam
rotmotor1 = chrono.ChLinkMotorRotationSpeed()

# Connect the rotor and the stator and add the motor to the system:
rotmotor1.Initialize(builder.GetLastBeamNodes().front(),  # body A (slave)
                     truss,  # body B (master)
                     chrono.ChFrameD(builder.GetLastBeamNodes().front().GetPos(),
                                     chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y))
                     # motor frame, in abs. coords
                     )
sys.Add(rotmotor1)

# Change Motor Function:
f_ramp = chrono.ChFunctionSine(60, 0.1)  # Modified motor function
rotmotor1.SetMotorFunction(f_ramp)

# Attach a visualization of the FEM mesh.

mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizebeamA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()

# Change Camera Position:
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L / 2, 0, 0))  # Modified camera position
vis.AddTypicalLights()

msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

sys.DoStaticLinear()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)