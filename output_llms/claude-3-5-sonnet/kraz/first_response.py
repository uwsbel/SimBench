import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# Initialize the ChSystem
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain_patch = terrain.AddPatch(patch_mat, 
                                 chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                                 100, 100)
terrain.Initialize()

# Create the vehicle
vehicle_file = veh.GetDataPath() + 'vehicle/Kraz_64431/vehicle/Kraz_64431_Vehicle.json'
powertrain_file = veh.GetDataPath() + 'vehicle/Kraz_64431/powertrain/Kraz_64431_SimplePowertrain.json'
tire_file = veh.GetDataPath() + 'vehicle/Kraz_64431/tire/Kraz_64431_TMeasy.json'

vehicle = veh.WheeledVehicle(system, vehicle_file)
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))

powertrain = veh.ReadPowertrainJSON(powertrain_file)
vehicle.InitializePowertrain(powertrain)

for axle in vehicle.GetAxles():
    for wheel in axle.GetWheels():
        tire = veh.ReadTireJSON(tire_file)
        vehicle.InitializeTire(tire, wheel, veh.VisualizationType_MESH)

# Create the driver system
driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())
driver.Initialize()

# Create the visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz Vehicle Simulation")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVectorD(0, 0, -10), 6.0, 0.5)
vis.Initialize()
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Simulation loop
step_size = 1e-3
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Ensure real-time simulation
    realtime_timer.Spin(step_size)

    # Check for simulation end
    if time > 1000:  # End simulation after 1000 seconds
        break

# Clean up
vis.GetDevice().closeDevice()