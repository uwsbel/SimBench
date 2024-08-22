import math as m  
import pychrono as chrono  
import pychrono.fea as fea  
import pychrono.pardisomkl as pardiso  
import pychrono.irrlicht as chronoirr  
import os  



class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):
        
        if x > 0.4:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.4)) / 2.0



out_dir = chrono.GetChronoOutputPath() + "BEAM_BUCKLING"


sys = chrono.ChSystemSMC()


L = 1  
H = 0.25  
K = 0.05  
vA = chrono.ChVector3d(0, 0, 0)  
vC = chrono.ChVector3d(L, 0, 0)  
vB = chrono.ChVector3d(L, -H, 0)  
vG = chrono.ChVector3d(L - K, -H, 0)  
vd = chrono.ChVector3d(0, 0, 0.0001)  


body_truss = chrono.ChBody()
body_truss.SetFixed(True)  
sys.AddBody(body_truss)  


boxtruss = chrono.ChVisualShapeBox(0.02, 0.2, 0.1)
body_truss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))


body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)  
sys.AddBody(body_crank)  


boxcrank = chrono.ChVisualShapeBox(K, 0.02, 0.02)
body_crank.AddVisualShape(boxcrank)


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFramed(vG))  
myfun = ChFunctionMyFun()  
motor.SetAngleFunction(myfun)  
sys.Add(motor)  


mesh = fea.ChMesh()


beam_wy = 0.10  
beam_wz = 0.01  


minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)  

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(73.0e9)  
melasticity.SetShearModulusFromPoisson(0.3)  
melasticity.SetAsRectangularSection(beam_wy, beam_wz)  

msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)  
msection1.SetDrawThickness(beam_wy, beam_wz)  


builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 32, vA, vC, chrono.VECT_Y, 3)  


builder_iga.GetLastBeamNodes().front().SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[-1]  
node_mid = builder_iga.GetLastBeamNodes()[17]  


section2 = fea.ChBeamSectionEulerAdvanced()
hbeam_d = 0.024  
section2.SetDensity(2700)  
section2.SetYoungModulus(73.0e9)  
section2.SetShearModulusFromPoisson(0.3)  
section2.SetRayleighDamping(0.000)  
section2.SetAsCircularSection(hbeam_d)  


builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 3, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))  


node_top = builderA.GetLastBeamNodes()[0]
node_down = builderA.GetLastBeamNodes()[-1]


constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, True, True, False, False, False)  


sphereconstr2 = chrono.ChVisualShapeSphere(0.01)
constr_bb.AddVisualShape(sphereconstr2)


section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.048  
section3.SetDensity(2700)  
section3.SetYoungModulus(73.0e9)  
section3.SetShearModulusFromPoisson(0.3)  
section3.SetRayleighDamping(0.000)  
section3.SetAsCircularSection(crankbeam_d)  


builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 3, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))  


node_crankG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes()[-1]


constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)


constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, False)


sphereconstr3 = chrono.ChVisualShapeSphere(0.01)
constr_bc.AddVisualShape(sphereconstr3)


mesh.SetAutomaticGravity(False)  


sys.Add(mesh)


mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(
    chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)  
mvisualizebeamA.SetColorscaleMinMax(-500, 500)  
mvisualizebeamA.SetSmoothFaces(True)  
mvisualizebeamA.SetWireframe(False)  
mesh.AddVisualShapeFEA(mvisualizebeamA)  

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(
    chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  
mvisualizebeamC.SetSymbolsThickness(0.006)  
mvisualizebeamC.SetSymbolsScale(0.01)  
mvisualizebeamC.SetZbufferHide(False)  
mesh.AddVisualShapeFEA(mvisualizebeamC)  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('Beams and constraints')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0.0, 0.6, -1.0))  
vis.AddTypicalLights()  


pardiso_solver = pardiso.ChSolverPardisoMKL()
sys.SetSolver(pardiso_solver)  


ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)  


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    chronoirr.drawGrid(vis, 0.05, 0.05, 20, 20,
                       chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  
    vis.EndScene()  
    sys.DoStepDynamics(0.001)