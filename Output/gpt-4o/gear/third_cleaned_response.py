import pychrono as chrono                         
import pychrono.irrlicht as chronoirr             
import math as m                                  


sys = chrono.ChSystemNSC()                        


mat = chrono.ChMaterialSurfaceNSC()               


radA = 1.5                                        
radB = 3.5                                        


mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,      
                                   1000,          
                                   True,          
                                   False,         
                                   mat)           
sys.Add(mbody_truss)                              
mbody_truss.SetFixed(True)                        
mbody_truss.SetPos(chrono.ChVector3D(0, 0, 3))    


vis_mat = chrono.ChVisualMaterial()                       
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))  


mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0,  
                                   1000,          
                                   True,          
                                   False,         
                                   mat)           
sys.Add(mbody_train)                              
mbody_train.SetPos(chrono.ChVector3D(3, 0, 0))    


link_revoluteTT = chrono.ChLinkLockRevolute()                         
link_revoluteTT.Initialize(mbody_truss, mbody_train,                  
                           chrono.ChFrameD(chrono.ChVector3D(0,0,0),  
                                           chrono.QUNIT))             
sys.AddLink(link_revoluteTT)                                          


mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,             
                                        radA, 0.5,                  
                                        1000, True, False, mat)     
sys.Add(mbody_gearA)                                                
mbody_gearA.SetPos(chrono.ChVector3D(0, 0, -1))                     
mbody_gearA.SetRot(chrono.Q_from_AngX(m.pi / 2))                    
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)               


mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)                                
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFrameD(chrono.ChVector3D(0, 3.5, 0),     
                                                          chrono.Q_from_AngX(chrono.CH_C_PI_2)))  


link_motor = chrono.ChLinkMotorRotationSpeed()                      
link_motor.Initialize(mbody_gearA, mbody_truss,                     
                      chrono.ChFrameD(chrono.ChVector3D(0, 0, 0),   
                                      chrono.QUNIT))                
link_motor.SetSpeedFunction(chrono.ChFunction_Const(3))             
sys.AddLink(link_motor)                                             


interaxis12 = radA + radB                                           
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,            
                                        radB, 0.4,                  
                                        1000, True, False, mat)     
sys.Add(mbody_gearB)                                                
mbody_gearB.SetPos(chrono.ChVector3D(interaxis12, 0, -2))           
mbody_gearB.SetRot(chrono.Q_from_AngX(m.pi / 2))                    
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)               


link_revolute = chrono.ChLinkLockRevolute()                         
link_revolute.Initialize(mbody_gearB, mbody_train,                  
                         chrono.ChFrameD(chrono.ChVector3D(interaxis12, 0, 0), chrono.QUNIT))  
sys.AddLink(link_revolute)                                          



link_gearAB = chrono.ChLinkGear()                                     
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFrameD())   
link_gearAB.SetFrame1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))    
link_gearAB.SetFrame2(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))    
link_gearAB.SetTransmissionRatio(radA / radB)                         
sys.AddLink(link_gearAB)                                              



radC = 2 * radB + radA                                                    
link_gearBC = chrono.ChLinkGear()                                         
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFrameD())       
link_gearBC.SetFrame1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))    
link_gearBC.SetFrame2(chrono.ChFrameD(chrono.ChVector3D(0, 0, -4), chrono.QUNIT))      
link_gearBC.SetTransmissionRatio(radB / radC)                             
link_gearBC.SetEpicyclic(True)                                            
sys.AddLink(link_gearBC)                                                  


radD = 5
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radD, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVector3D(-10, 0, -9))
mbody_gearD.SetRot(chrono.Q_from_AngZ(m.pi / 2))
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)

link_revoluteTD = chrono.ChLinkLockRevolute()
link_revoluteTD.Initialize(mbody_truss, mbody_gearD, chrono.ChFrameD(chrono.ChVector3D(-10, 0, -9), chrono.Q_from_AngY(m.pi / 2)))
sys.AddLink(link_revoluteTD)

link_gearAD = chrono.ChLinkGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFrameD())
link_gearAD.SetFrame1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))
link_gearAD.SetFrame2(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngZ(-m.pi / 2)))
link_gearAD.SetTransmissionRatio(1)  
sys.AddLink(link_gearAD)


radE = 2
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radE, 0.5, 1000, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVector3D(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.Q_from_AngZ(m.pi / 2))
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)

link_revoluteTE = chrono.ChLinkLockRevolute()
link_revoluteTE.Initialize(mbody_truss, mbody_pulleyE, chrono.ChFrameD(chrono.ChVector3D(-10, -11, -9), chrono.Q_from_AngY(m.pi / 2)))
sys.AddLink(link_revoluteTE)


link_beltDE = chrono.ChLinkBeltPulley()
link_beltDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFrameD())
link_beltDE.SetPulley1(chrono.ChFrameD(chrono.ChVector3D(-10, 0, -9), chrono.Q_from_AngZ(-m.pi / 2)))
link_beltDE.SetPulley2(chrono.ChFrameD(chrono.ChVector3D(-10, -11, -9), chrono.Q_from_AngZ(-m.pi / 2)))
link_beltDE.SetTransmissionRatio(1)  
sys.AddLink(link_beltDE)


vis = chronoirr.ChVisualSystemIrrlicht()                        
vis.AttachSystem(sys)                                           
vis.SetWindowSize(1024, 768)                                    
vis.SetWindowTitle('Gears and pulleys')                         
vis.Initialize()                                                
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) 
vis.AddSkyBox()                                                 
vis.AddCamera(chrono.ChVector3D(12, 15, -20))                   
vis.AddTypicalLights()                                          


sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED) 


while vis.Run():                                    
    vis.BeginScene()                                
    vis.Render()                                    
    vis.EndScene()                                  
    sys.DoStepDynamics(1e-3)