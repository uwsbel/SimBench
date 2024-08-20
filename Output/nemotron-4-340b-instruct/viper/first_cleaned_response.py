import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.ChSystemSMC()


ground = chrono.ChBodyEasyBox(200, 1, 200, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
chrono.GetSystem().Add(ground)


from pychrono.vehicle import viper
rover = viper.Viper(chrono.GetSystem())
rover.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
rover.GetVehicle().SetChassisVisualizationType(chrono.ChVehicleVisualizationType.PRIMITIVES)
rover.GetVehicle().SetSuspensionVisualizationType(chrono.ChVehicleVisualizationType.PRIMITIVES)
rover.GetVehicle().SetSteeringVisualizationType(chrono.ChVehicleVisualizationType.PRIMITIVES)
rover.GetVehicle().SetWheelVisualizationType(chrono.ChVehicleVisualizationType.PRIMITIVES)


driver = rover.CreateDriver()
driver.SetSteeringController(chrono.ChFunction_Ramp(0, 1, 5))  


chrono.GetSystem().Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


rover.GetVehicle().GetTire(0).SetContactCallback(chrono.ChMaterialSurface.SMC)
rover.GetVehicle().GetTire(1).SetContactCallback(chrono.ChMaterialSurface.SMC)
rover.GetVehicle().GetTire(2).SetContactCallback(chrono.ChMaterialSurface.SMC)
rover.GetVehicle().GetTire(3).SetContactCallback(chrono.ChMaterialSurface.SMC)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(chrono.GetSystem())
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper Rover Simulation')
vis.AddCamera(chrono.ChVectorD(0, 2, -5), chrono.ChVectorD(0, 0.5, 0))
vis.AddLight(chrono.ChVectorD(10, 10, 10), chrono.ChColor(1, 1, 1))
vis.Initialize()


step_size = 0.01
end_time = 10

while vis.Run() and chrono.GetSystem().GetChTime() < end_time:
    chrono.GetSystem().DoStepDynamics(step_size)

    
    driver.Synchronize(chrono.GetSystem().GetChTime())

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()