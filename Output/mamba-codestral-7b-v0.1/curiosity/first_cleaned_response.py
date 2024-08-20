import pychrono


system = pychrono.ChSystemSMC()


ground = pychrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, False)
ground.SetPos(pychrono.ChVectorD(0, -0.1, 0))
ground.SetBodyFixed(True)
system.Add(ground)


box = pychrono.ChBodyEasyBox(1, 1, 1, 1000, True, False)
box.SetPos(pychrono.ChVectorD(0, 1, 0))
system.Add(box)


system.Set_G_acc(pychrono.ChVectorD(0, -9.81, 0))
system.SetTimestepType(pychrono.ChTimestepper.Type_EULER_IMPLICIT)
system.SetTimestep(0.01)


system.DoStaticLinear()
system.DoDynamic()