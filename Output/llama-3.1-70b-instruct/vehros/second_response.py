"""
PyChrono simulation script for HMMWV vehicle with ROS integration and visualization.
"""

import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
from pychrono import irrlicht as chronoirr

def main():
    # Create the HMMWV vehicle and set its parameters.
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)  # Set the contact method for physics.
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)  # Disable collision for the chassis.
    hmmwv.SetChassisFixed(False)  # Make the chassis movable.
    hmmwv.SetInitPosition(ch.ChCoordsysd(ch.ChVector3d(0, 0, 1.6), ch.ChQuaterniond(1, 0, 0, 0)))  # Initialize vehicle position and orientation.
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)  # Use shaft-based engine model.
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # Use automatic transmission with shafts.
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)  # Set all-wheel drive.
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)  # Use pitman arm steering.
    hmmwv.SetTireType(veh.TireModelType_TMEASY)  # Set tire model.
    hmmwv.SetTireStepSize(1e-3)  # Set the tire simulation step size.
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)  # Set chassis visualization type.
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # Set suspension visualization type.
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)  # Set steering visualization type.
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)  # Set wheel visualization type.
    hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)  # Set tire visualization type.
    hmmwv.Initialize()  # Initialize the vehicle.

    # Create the terrain for the vehicle to interact with.
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChContactMaterialNSC()  # Create a contact material for the terrain.
    patch_mat.SetFriction(0.9)  # Set friction for the terrain.
    patch_mat.SetRestitution(0.01)  # Set restitution (bounciness) for the terrain.
    patch = terrain.AddPatch(patch_mat, ch.CSYSNORM, 100.0, 100.0)  # Add a patch to the terrain.
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)  # Set the texture for the terrain patch.
    terrain.Initialize()  # Initialize the terrain.

    # Create and initialize the driver system.
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()  # Initialize the driver system.

    # Create the ROS manager and register handlers for communication.
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())  # Register the clock handler to synchronize ROS with the simulation.
    # Register the driver inputs handler for ROS topic '~/input/driver_inputs'.
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    # Register the vehicle state handler to publish vehicle state to ROS topic '~/output/hmmwv/state'.
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    ros_manager.Initialize()  # Initialize the ROS manager.

    # Create the Irrlicht application for visualization.
    app = chronoirr.ChVisualSystemIrrlicht()
    app.SetWindowSize(1024, 768)
    app.SetWindowTitle('HMMWV Simulation')
    app.SetLogo('logo.png')
    app.SetCamera(ch.ChVector3D(0, 0, 1.5), ch.ChVector3D(0, 0, 0))
    app.AddTypicalLights()
    app.AddSkyBox()
    app.AddCameraVerticalTrack()
    app.AddCameraHorizontalTrack()
    app.AddCameraZoom()
    app.AddTypicalLights()
    app.AddSkyBox()
    app.AddCameraVerticalTrack()
    app.AddCameraHorizontalTrack()
    app.AddCameraZoom()

    # Start the simulation loop.
    time = 0
    time_step = 1e-3  # Define the simulation time step.
    time_end = 30  # Set the total duration of the simulation.

    hmmwv.GetVehicle().EnableRealtime(True)  # Enable real-time simulation for the vehicle.
    while time < time_end:
        time = hmmwv.GetSystem().GetChTime()  # Update simulation time.

        # Get driver inputs and synchronize the vehicle, terrain, and driver systems.
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)  # Synchronize the driver system.
        terrain.Synchronize(time)  # Synchronize the terrain.
        hmmwv.Synchronize(time, driver_inputs, terrain)  # Synchronize the vehicle with inputs and terrain.

        # Advance the simulation for all modules by one timestep.
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        # Update the ROS manager to handle data publishing.
        if not ros_manager.Update(time, time_step):
            break  # Exit loop if ROS manager update fails.

        # Render the scene.
        app.BeginScene()
        app.Render()
        app.EndScene()

        # Handle events.
        app.GetDevice().run()

if __name__ == "__main__":
    main()