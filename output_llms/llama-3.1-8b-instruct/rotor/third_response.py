import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# Define a custom motor function class
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self, A1, A2, T1, T2, T3, w):
        chrono.ChFunction.__init__(self)
        self.A1 = A1
        self.A2 = A2
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.w = w

    def GetVal(self, x):
        if x < self.T1:
            return self.A1 * x
        elif x < self.T2:
            return self.A1 * self.T1 + self.A2 * (x - self.T1)
        else:
            return self.A1 * self.T1 + self.A2 * (self.T2 - self.T1) + self.w * (x - self.T2)

# Create a system
sys = chrono.ChSystemSMC()

# Create a mesh
mesh = fea.ChMesh()
sys.Add(mesh)

# Set automatic gravity
mesh.SetAutomaticGravity(True, 2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Define beam parameters
beam_L = 6
beam_ro = 0.050
beam_ri = 0.045
CH_PI = 3.14159

# Create a section for the beam
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
msection.SetDrawCircularRadius(beam_ro)

# Create a builder for the beam
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, msection, 20, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(beam_L, 0, 0), chrono.VECT_Y, 3)

# Get the last beam nodes
node_mid = builder.GetLastBeamNodes()[m.floor(builder.GetLastBeamNodes().size() / 2.0)]

# Create the flywheel and attach it to the center of the beam
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)
mbodyflywheel.SetCoordsys(chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0), chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Z)))
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
bearing.Initialize(builder.GetLastBeamNodes().back(), truss, chrono.ChFramed(builder.GetLastBeamNodes().back().GetPos()))
sys.Add(bearing)

# Create the motor that rotates the beam
rotmotor1 = chrono.ChLinkMotorRotationSpeed()

# Connect the rotor and the stator and add the motor to the system:
rotmotor1.Initialize(builder.GetLastBeamNodes().front(), truss, chrono.ChFramed(builder.GetLastBeamNodes().front().GetPos(), chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y)))
sys.Add(rotmotor1)

# Define the custom motor function parameters
A1 = 10
A2 = 20
T1 = 1
T2 = 2
T3 = 3
w = 5

# Create the custom motor function
f_ramp = ChFunctionMyFun(A1, A2, T1, T2, T3, w)

# Set the motor function
rotmotor1.SetMotorFunction(f_ramp)

# Attach a visualization of the FEM mesh
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
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam_L / 2, 0, 0))
vis.AddTypicalLights()

# Set the solver
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Set the timestepper (optional)
# sys.SetTimestepper(chrono.ChTimestepperHHT(0.01))

# Perform static linear analysis
sys.DoStaticLinear()

# Run the simulation
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)