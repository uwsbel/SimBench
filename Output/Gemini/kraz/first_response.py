import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# ---------------------------------------------------------------------
# Create the Chrono system and set default parameters

system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# ---------------------------------------------------------------------
# Create the terrain

terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetContactMethod(chrono.ChContactMethod_NSC)
patch_mat.SetCompliance(0.00001)
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChVectorD(-50, -50, 0), chrono.ChVectorD(50, -50, 0), 
                         chrono.ChVectorD(50, 50, 0), chrono.ChVectorD(-50, 50, 0))
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
terrain.Initialize()

# ---------------------------------------------------------------------
# Create the Kraz vehicle

vehicle = veh.KrazTruck(system)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.6)))
vehicle.SetInitFwdVel(0.0)
vehicle.Initialize()

# ---------------------------------------------------------------------
# Create the driver system

driver = veh.ChDriver(vehicle)
driver.Initialize()

# ---------------------------------------------------------------------
# Create the Irrlicht application

app = chronoirr.ChIrrApp(system, "Kraz Vehicle Demo", chronoirr.dimension2du(1024,768))
app.AddTypicalSky()
app.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
app.AddTypicalCamera(chronoirr.vector3df(10,10,10), chronoirr.vector3df(0,0,1.6))
app.AddTypicalLights()

app.AssetBindAll()
app.AssetUpdateAll()

# ---------------------------------------------------------------------
# Simulation loop

app.SetTimestep(0.01)
app.SetTryRealtime(True)

while app.GetDevice().run():
    time = system.GetChTime()

    # Driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)

    # Advance simulation for one timestep for all modules
    driver.Advance(step=0.01)
    terrain.Advance(step=0.01)
    vehicle.Advance(step=0.01)

    app.BeginScene()
    app.DrawAll()
    app.EndScene()