import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))  


ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  
ground.EnableCollision(False)  


sphere_1 = chrono.ChVisualShapeSphere(0.2)  
ground.AddVisualShape(sphere_1, chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))


pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  
pend_1.EnableCollision(False)  
pend_1.SetMass(2)  
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  


sphere_1 = chrono.ChVisualShapeSphere(0.1)  
pend_1.AddVisualShape(sphere_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))



pend_1.SetPos(chrono.ChVector3d(1, 0, 1))
pend_1.SetAngVel(chrono.ChVector3d(0, 0, chrono.CH_PI_2))  



sph_1 = chrono.ChLinkSpherical(chrono.ChFrame3d(chrono.ChVector3d(0, 0, 1)), chrono.ChFrame3d(chrono.ChVector3d(0, 0, 1)), chrono.ChQuaterniond(1, 0, 0, 0))
sph_1.Initialize(ground, pend_1)
sys.AddLink(sph_1)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('ChBodyAuxRef demo')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, 3, 6))  
vis.AddTypicalLights()  


log_info = True  
while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(1e-3)  

    
    if log_info and sys.GetChTime() > 1:
        pos_1 = pend_1.GetPos()  
        print("t = ", sys.GetChTime())
        print("     ", pos_1.x, "  ", pos_1.y)
        lin_vel_1 = pend_1.GetPosDt()  
        print("     ", lin_vel_1.x, "  ", lin_vel_1.y)
        ang_vel_1 = pend_1.GetAngVel()  
        print("     ", ang_vel_1.x, "  ", ang_vel_1.y, "  ", ang_vel_1.z)
        log_info = False