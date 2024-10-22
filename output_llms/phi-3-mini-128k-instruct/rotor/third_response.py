import math as m
import chrono as chrono
import chrono.fea as fea
import chrono.pardisomkl as mkl
import chrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True,
                          2)  # for max precision in gravity of FE, at least 2 integration points per element when using cubic IGA
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0));

beam_L = 6
beam_ro = 0.050
beam_ri = 0.045
CH_PI = 3.141592653589793

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
msection.SetDrawCircularRadius(beam_ro)

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

mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)  # R, h, density
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
                   chrono.ChFramed(builder.GetLastBeamNodes().back().GetPos())
                   )
sys.Add(bearing)

# Create the motor that rotates the beam
rotmotor1 = chrono.ChLinkMotorRotationSpeed()

# Connect the rotor and the stator and add the motor to the system:
rotmotor1.Initialize(builder.GetLastBeamNodes().front(),  # body A (slave)
                     truss,  # body B (master)
                     chrono.ChFramed(builder.GetLastBeamNodes().front().GetPos(),
                                     chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y))
                     # motor frame, in abs. coords
                     )
sys.Add(rotmotor1)

# Define a custom motor function class
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self, A1, A2, T1, T2, T3, w):
        super(ChFunctionMyFun, self).__init__()
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
            return self.A2 * x
        else:
            return self.A3 * math.sin(self.w * x)

# Instantiate the custom motor function with specified parameters
f_ramp = ChFunctionMyFun(A1=10, A2=20, T1=1, T2=2, T3=3, w=0.1)
rotmotor1.SetMotorFunction(f_ramp)

# Attach a visualization of the FEM mesh.

mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizebeamA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE.





































and.























.



























































self.































































l #_





.




.0.


.0.0


.






.







.
.





.l.th











































.



.0.




















.





 

 
































 0_0_ 0 0  # 0. 0.
.
.0 0     # 0 #00 0l20_0 # #: 120 0.or.m.00_l_
.2.0200.
.00
0.



.0.e.r.
 1 0. 0.0.00. 
. .0.0. .        
l.0.0.else.return.return.return #  #.000

.03000.e.return.00.0.return else else else

000.0.0ile.0.0.
.0


l.0.0.0        0.0.
.000.

       



.     1.         #.2.
       0.
0.
       0.0.        
       

00.0.00
l
       
11




.0.0.00


.l.0.0.0.0.0.

 0 

.0.0.0.
.00.0.0.
300.0.0.0.
.0.0.0.0.00.else.0.00.0.l. 0.0.0.0







.0.0)


.0.000
l.0.
.0.0.
. #.0.00.0.sub000)
)


)
.
)







0

return
01.0.220)
.
.return_0.return)
orile.
)
or)
)0.0

















.0.0.0.0.return.0.return.return)
else.else.return.
0.0.00.0.0.
















.return.




.
. 

)
)



)







0.0.
.




)
)
)
)
)

.
.



)





)
)



)
)

)







)


















)
)





.0.0) 1)0) ) )) 2)
) l)
)
)


.


0)
0)

l)
0)
l)l)
))l) l.0.3. .0.0.00.0.030000.0, 
.while.0.0.0.0,0.0.0, 3,0, 3.30)
l.3.3,00, 3,0,0,0,0,l,0,0,0.l,

, 
, 
.0.0,0,0,0,0,

l, 0)
)


l)






   
)

,     l.3l

l



)

   
l. 2)l)l)


0.l)
)
)


l.







   







l




l)
l)
0)
)

)
)
)
)
)
l)


l.3.

l)
l
4)

l.l.l.3.3.3.3.3.
l.0l)l)


l)




l.l.
)



l)
,

)



,self)
)
l.0)
)
l.0)




)


)
)
)
l.3,0)


,
,


)
)l)





,


)

,  #2,



l.3.3()
0(0,3,


(.
.3.0)
return return.3.3l.
l.3


.


















m.





















.
.0.0


,
,


,


1.1
1








0









.3.





 



,
.







,








,





 0.sub.
.
)
l)
 
 0, sub1.
y.
.4.

)
)
.center)
)
)
)
)



)





, return)l)
l)
l)l)





return)
.





0,
,



)
l.return.0.0.g.00.return.return.0

l.th.0)
return)
l)
l)



)
)
)

 #)




g)
.0)






) 2


)






)
)
)
)











0)









)
)
)
)






)


)
)
)
)
)

)
)
)

)
)
















)
)
)
)
)
)
)












l

















)

)


3




l

















l.3















    












,

















,
,




























,

,




,
,

,
,

























































































































































,


)









































)




)
)



m.r.




















return,







r,

















return





l.

return
return


return.return
return.return.return.
else else else.return.return
else

returnmelse

return
return.return


return


return

elseelse


return.return.
return,

return,return,













)





center,







else.

 # # 1

  #of.
else
else




























































)f)



,




 #
b
























































































































































,



























)
)
)






































.




2)


)
)
) )
)
)
)
















































)




























 

)
)




, 
)








   2)
















,
,



   
   








































,

















   







b)














,


















    #

,

,



,
.
_

,
,




,
,

















.






)
)
)
) sub)

)
)



,
)
)
)






,
)


)
)
).
)
)
)
)
)
)
center,
)
)
)
)
b,b,











self)
self.











































































r,




























)
)











)



































































































































































self,
























   
   

self
,








,













_












       , 





,    )
) #l)
, 
,


















self,  #, 2, 
, 
,
,
,
,  #,3)
, # # #   )())
)

, 


   
,
,
   
,    ,
    #)
)
    #    #)
l    #   3)






   3. #)


   3)










       


      
   
         l)l) l)

)

l
l




   






   
        #            self 
   





   l 
   

   
    2
               l)
 

 
       

 
    
   
    #     #. l.l

.l, l, 
,        , 
,
,
   
   








,



,
,
, 

, 
    
   self.        
   
   
   .     
       else 
              
             # #              #
       
   


    
   
    #   


,




   
          







,
,
   


,    ,    
, 
       
 #        





   
   
                  .
   
   .
.l,



, 








,,m,
   
,


   


,     

,
, 

,
,
 
   )
       )
       

, #.
,


   
   
    #   return
       
             .return, 
,
,
,
,
,
,
,
return,
,
,



 #sub)
   


sub,



, 
,


return,


)
 #return,
)


return,



)
,
, 
    # 

return,
return,  # #

   

return)


return,
,
,



    #3






)
)






)





























else)
l, 
l)  #.


       


)


if 

return,












   if. 




   



   
   
)
,






















2.
































































 

   
    #


       
        # #  #    #    
    #)








   
 

           



     #