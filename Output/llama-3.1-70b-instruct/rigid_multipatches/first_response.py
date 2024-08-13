import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np

# Initialize the PyChrono environment and core components
chrono.SetChronoDataPath('./data/')

# Create the simulation system and set the timestep
mysystem = chrono.ChSystemNSC()
mysystem.SetSolverType(chrono.ChSolver.Type_PSSOR)
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the vehicle
veh_params = veh.HMMWV()
veh_params.SetContactMethod(chrono.ChContactMethodType.MULTICORE_SMC)
veh_params.SetChassisFixed(False)
veh_params.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
veh_params.SetPowertrainType(veh.PowertrainType_SHAFTS)
veh_params.SetDriveType(veh.DrivetrainType_FOUR_WHEEL_DRIVE)
veh_params.SetEngineType(veh.EngineType_INTERNAL_COMBUSTION)
veh_params.SetBrakeType(veh.BrakeType_SHAFTS)
veh_params.SetSteeringType(veh.SteeringTypeAckermann())
veh_params.SetTireType(veh.TireModelType_FrictionCircle)
veh_params.SetTireStepSize(chrono.ChTime(0.01))

# Create the vehicle object
my_vehicle = veh.HMMWV(veh_params)
mysystem.Add(my_vehicle.GetChassisBody())
mysystem.Add(my_vehicle.GetChassisBodyAuxRef())

# Add the vehicle components to the simulation
for axle in my_vehicle.GetAxles():
    mysystem.Add(axle)
for wheel in my_vehicle.GetWheels():
    mysystem.Add(wheel)
for tire in my_vehicle.GetTires():
    mysystem.Add(tire)

# Create the terrain
terrain = chrono.ChBodyEasyBox(mysystem, 100, 10, 100, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -10, 0))
terrain.SetBodyFixed(True)
mysystem.Add(terrain)

# Create a flat patch with a texture
patch1 = chrono.ChBodyEasyBox(mysystem, 20, 2, 20, 1000, True, True)
patch1.SetPos(chrono.ChVectorD(-20, -8, 0))
patch1.SetBodyFixed(True)
mysystem.Add(patch1)

# Create a flat patch with a different texture
patch2 = chrono.ChBodyEasyBox(mysystem, 20, 2, 20, 1000, True, True)
patch2.SetPos(chrono.ChVectorD(20, -8, 0))
patch2.SetBodyFixed(True)
mysystem.Add(patch2)

# Create a mesh-based patch for a bump
bump = chrono.ChBodyEasyMesh(mysystem, 'bump.obj', 1000, True, True)
bump.SetPos(chrono.ChVectorD(0, -8, 20))
bump.SetBodyFixed(True)
mysystem.Add(bump)

# Create a heightmap-based patch for varying elevations
heightmap = chrono.ChBodyEasyHeightField(mysystem, 'heightmap.png', 1000, True, True)
heightmap.SetPos(chrono.ChVectorD(0, -8, -20))
heightmap.SetBodyFixed(True)
mysystem.Add(heightmap)

# Create the interactive driver system
driver = veh.ChIrrVehicleDriver(my_vehicle)
driver.SetSteeringMode(veh.SteeringMode_LOCKED_DIFFERENTIAL)
driver.SetThrottleMode(veh.ThrottleMode_LOCKED_RATIO)
driver.SetBrakingMode(veh.BrakingMode_LOCKED_RATIO)

# Create the Irrlicht visualization
myapplication = chronoirr.ChVisualSystemIrrlicht()
myapplication.SetWindowSize(1024, 768)
myapplication.SetWindowTitle('HMMWV on Complex Rigid Terrain')
myapplication.SetTypicalLenght(1.0)
myapplication.SetCameraRigID(my_vehicle.GetChassisBody().GetFrameID())
myapplication.SetCameraDistance(2.0)
myapplication.SetCameraElevation(45.0)
myapplication.SetCameraAzimuth(90.0)
myapplication.SetCameraLookAtPoint(chrono.ChVectorD(0, 0, 0))

# Add the vehicle components to the visualization
myapplication.AddSkyBox()
myapplication.AddTypicalLigths()
myapplication.AddCameraLight()
myapplication.AddCameraSkyBox()
myapplication.AddCameraNearFarZones(0.1, 1000)
myapplication.AddCameraVerticalWallZones(0.1, 1000)
myapplication.AddCameraGroundClippingPlane(chrono.ChVectorD(0, 1, 0), 0.1)
myapplication.AddCameraTarget(my_vehicle.GetChassisBody())

# Set the visualization for all vehicle components
for axle in my_vehicle.GetAxles():
    myapplication.AssetBindAllChildren(axle, chronoirr.AssetLevel.MESH)
for wheel in my_vehicle.GetWheels():
    myapplication.AssetBindAllChildren(wheel, chronoirr.AssetLevel.MESH)
for tire in my_vehicle.GetTires():
    myapplication.AssetBindAllChildren(tire, chronoirr.AssetLevel.MESH)

# Set the visualization for the terrain
myapplication.AssetBindAllChildren(terrain, chronoirr.AssetLevel.MESH)
myapplication.AssetBindAllChildren(patch1, chronoirr.AssetLevel.MESH)
myapplication.AssetBindAllChildren(patch2, chronoirr.AssetLevel.MESH)
myapplication.AssetBindAllChildren(bump, chronoirr.AssetLevel.MESH)
myapplication.AssetBindAllChildren(heightmap, chronoirr.AssetLevel.MESH)

# Set the simulation timestep and start the simulation
mysystem.SetTimestep(0.01)
myapplication.SetTimestep(0.01)
myapplication.SetTryRealtime(True)

# Run the simulation
while myapplication.GetDevice().run():
    mysystem.DoStepDynamics(0.01)
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.EndScene()