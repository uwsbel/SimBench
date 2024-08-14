from pychrono import core, irrlicht, vehicle
from pychrono.core import ChVectorD, ChQuaternion, ChSystemNSC

# 1. Initialize the PyChrono environment and core components
my_system = ChSystemNSC()
my_system.Set_G_acc(ChVectorD(0, 0, -9.81))

# 2. Add the required physical systems and objects as specified

# Create a fixed truss
truss_mat = core.ChMaterialSurfaceNSC()
truss_mat.SetFriction(0.5)
truss = core.ChBodyEasyBox(1, 1, 1, 1000, True, True, truss_mat)
truss.SetPos(ChVectorD(0, 0, 0))
truss.SetBodyFixed(True)
my_system.Add(truss)

# Create a rotating bar
bar_mat = core.ChMaterialSurfaceNSC()
bar_mat.SetFriction(0.5)
bar = core.ChBodyEasyCylinder(0.1, 1, 1000, True, True, bar_mat)
bar.SetPos(ChVectorD(2, 0, 0))
bar.SetRot(ChQuaternion(1, 0, 0, 0))
my_system.Add(bar)

# Create two gears
gear1_mat = core.ChMaterialSurfaceNSC()
gear1_mat.SetFriction(0.5)
gear1 = core.ChBodyEasyCylinder(0.5, 0.2, 1000, True, True, gear1_mat)
gear1.SetPos(ChVectorD(4, 0, 0))
gear1.SetRot(ChQuaternion(1, 0, 0, 0))
my_system.Add(gear1)

gear2_mat = core.ChMaterialSurfaceNSC()
gear2_mat.SetFriction(0.5)
gear2 = core.ChBodyEasyCylinder(0.5, 0.2, 1000, True, True, gear2_mat)
gear2.SetPos(ChVectorD(6, 0, 0))
gear2.SetRot(ChQuaternion(1, 0, 0, 0))
my_system.Add(gear2)

# Create a gear motor to enforce a constant rotation speed
gear_motor = vehicle.ChGear()
gear_motor.Initialize(bar, gear1, 1, 1)
my_system.Add(gear_motor)

# 3. Set necessary default parameters such as positions, forces, and interactions

# Set the motor speed
gear_motor.SetMotorSpeed(10)

# 3. Visualize the entire setup using the Irrlicht visualization system with appropriate materials and lighting

vis_system = irrlicht.ChIrrApp(my_system, 'Epicyclic Gears', irrlicht.dimension2du(800, 600), False, False)
vis_system.AddTypicalLights()
vis_system.AddSkyBox()
vis_system.AddLogo()
vis_system.AssetBindAllMaterials()
vis_system.AssetUpdateAllMaterials()

# Simulation loop
while vis_system.GetDevice().run():
    vis_system.BeginScene()
    vis_system.DrawAll()
    vis_system.DoStep()
    vis_system.EndScene()