import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math

# Initialize the ChronoEngine and vehicle model
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data/"))
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# Create the simulation system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV Simulation on Complex Terrain")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 5, -15))
vis.AddTypicalLights()

# Create the vehicle
driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.02)

driver.Initialize()

# Create the HMMWV vehicle
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.0, 0), chrono.QUNIT))
hmmwv.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.SetTireStepSize(1e-3)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# Create the terrain
terrain = veh.RigidTerrain(system)

# Flat patch 1
patch1 = terrain.AddPatch(chrono.ChCoordsysD(chrono.VNULL, chrono.QUNIT), 
                          chrono.ChVectorD(100, 0, 100))
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)

# Flat patch 2
patch2 = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(100, 0, 0), chrono.QUNIT), 
                          chrono.ChVectorD(100, 0, 100))
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)

# Bump patch
bump = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, -50), chrono.QUNIT),
                        veh.GetDataFile("terrain/bump.mesh"))

# Heightmap patch
heightmap = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(-100, 0, 0), chrono.QUNIT),
                             veh.GetDataFile("terrain/height_map.dat"),
                             "height_map", 128, 128, 0, 4)
heightmap.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 16, 16)

terrain.Initialize()

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
step_size = 1e-3

while vis.Run():
    time = system.GetChTime()

    # Driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize("", driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    system.DoStepDynamics(step_size)
    vis.Advance(step_size)

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

vis.GetDevice().closeDevice()