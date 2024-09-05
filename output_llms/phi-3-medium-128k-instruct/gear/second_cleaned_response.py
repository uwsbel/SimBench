import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m


sys = chrono.ChSystemNSC()  


mat = chrono.ChContactMaterialNSC()  


radA = 1.5  
radB = 3.5  


mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,  
                                   1000,          
                                   True,          
                                   False,         
                                   mat)           
sys.Add(mbody_truss)  
mbody_truss.SetFixed(True)  
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))  


vis_mat = chrono.ChVisualMaterial()  
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))  


mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0,  
                                   1000,          
                                   True,          
                                   False,         
                                   mat)           
sys.Add(mbody_train)  
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))  


link_revoluteTT = chrono.ChLinkLockRevolute()  
link_revoluteTT.Initialize(mbody_truss, mbody_train,  
                           chrono.ChFramed(chrono.ChVector3d(0,0,0),  
                                           chrono.QUNIT))             
sys.AddLink(link_revoluteTT)