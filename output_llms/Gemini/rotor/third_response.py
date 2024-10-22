import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# ---------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------

# Use MKL
use_mkl = True

# Timestepper
use_hht = False  # if True, use HHT timestepper, otherwise use default Euler implicit

# ---------------------------------------------------------------------
# Create the system and set method parameters
# ---------------------------------------------------------------------

sys = chrono.ChSystemSMC()

# Set solver parameters
if use_mkl:
    msolver = mkl.ChSolverPardisoMKL()
    sys.SetSolver(msolver)
    msolver.LockSparsityPattern(True)
else:
    sys.SetSolverType(chrono.ChSolver.Type_SPARSE_LU)
    sys.SetSolverMaxIterations(100)
    sys.SetSolverForceSparsity(True)

# Set integration method parameters
if use_hht:
    sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
    # sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    # hht_timestepper = chrono.ChTimestepperHHT(sys)
    # sys.SetTimestepper(hht_timestepper)
    # hht_timestepper.SetAlpha(-0.2)
    # hht_timestepper.SetMaxiters(100)
    # hht_timestepper.SetAbsTolerances(1e-5, 1e-3)
    # hht_timestepper.SetMode(chrono.ChTimestepperHHT.ACCELERATION)
    # hht_timestepper.SetScaling(True)
else:
    sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# Set other system parameters
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# ---------------------------------------------------------------------
# Create the FEM mesh
# ---------------------------------------------------------------------

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True,
                         2)  # for max precision in gravity of FE, at least 2 integration points per element when using cubic IGA

# ---------------------------------------------------------------------
# Create the beam section and material properties
# ---------------------------------------------------------------------

beam_L = 6
beam_ro = 0.050
beam_ri = 0.045
CH_PI = 3.1456

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
melasticity.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)

msection.SetCircular(True)
msection.SetDrawCircularRadius(
    beam_ro)  # SetAsCircularSection(..) would overwrite Ixx Iyy J etc.

# ---------------------------------------------------------------------
# Create the beam using the ChBuilderBeamIGA tool
# ---------------------------------------------------------------------

builder = fea.ChBuilderBeamIGA()

builder.BuildBeam(mesh,  # the mesh to put the elements in
                  msection,  # section of the beam
                  20,  # number of sections (spans)
                  chrono.ChVector3d(0, 0, 0),  # start point
                  chrono.ChVector3d(beam_L, 0, 0),  # end point
                  chrono.VECT_Y,  # suggested Y direction of section
                  1)  # order (3 = cubic, etc)

node_mid = builder.GetLastBeamNodes()[m.floor(builder.GetLastBeamNodes().size() / 2.0)]

# ---------------------------------------------------------------------
# Create the flywheel and attach it to the center of the beam
# ---------------------------------------------------------------------

mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1,
                                         7800)  # R, h, density
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),  # flywheel initial center (plus Y offset)
                       chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Z))
    # flywheel initial alignment (rotate 90° so cylinder axis is on X)
)
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# ---------------------------------------------------------------------
# Create the truss and end bearing
# ---------------------------------------------------------------------

truss = chrono.ChBody()
truss.SetBodyFixed(True)
sys.Add(truss)

# Create the end bearing
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes().back(),
                   truss,
                   chrono.ChFrameD(builder.GetLastBeamNodes().back().GetPos())
                   )
sys.Add(bearing)

# ---------------------------------------------------------------------
# Create and apply the motor that rotates the beam
# ---------------------------------------------------------------------

# 1. Motor Function Modification:
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self, A1, A2, T1, T2, T3, w):
        chrono.ChFunction.__init__(self)
        self.A1 = A1
        self.A2 = A2
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.w = w

    def Get_y(self, x):
        if 0 <= x < self.T1:
            return self.A1 * (1.0 - chrono.CH_C_COS(self.w * x)) / 2.0
        elif self.T1 <= x < self.T2:
            return self.A1
        elif self.T2 <= x < self.T3:
            return -self.A2 * (1.0 - chrono.CH_C_COS(self.w * (x - self.T2))) / 2.0 + self.A1
        else:
            return self.A1 - self.A2


# Create the motor that rotates the beam
rotmotor1 = chrono.ChLinkMotorRotationSpeed()

# Custom motor function parameters
A1 = 50
A2 = 25
T1 = 1.0
T2 = 2.0
T3 = 3.0
w = chrono.CH_C_PI / 2

# Connect the rotor and the stator and add the motor to the system:
rotmotor1.Initialize(builder.GetLastBeamNodes().front(),  # body A (slave)
                     truss,  # body B (master)
                     chrono.ChFrameD(builder.GetLastBeamNodes().front().GetPos(),
                                     chrono.Q_from_AngAxis(CH_PI / 2.0, chrono.VECT_Y))
                     # motor frame, in abs. coords
                     )
sys.Add(rotmotor1)

# Use custom motor function
f_ramp = ChFunctionMyFun(A1, A2, T1, T2, T3, w)
rotmotor1.SetMotorFunction(f_ramp)

# ---------------------------------------------------------------------
# Visual settings for the FEM mesh
# ---------------------------------------------------------------------

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

# ---------------------------------------------------------------------
# Create the Irrlicht visualization
# ---------------------------------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, 4), chrono.ChVectorD(beam_L / 2, 0, 0))
vis.AddTypicalLights()

# ---------------------------------------------------------------------
# Perform a static linear analysis
# ---------------------------------------------------------------------

sys.DoStaticLinear()

# ---------------------------------------------------------------------
# Run the simulation
# ---------------------------------------------------------------------

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)