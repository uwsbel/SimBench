import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr
import math

def main():
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')
    # Create the HMMWV vehicle and set its parameters.
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
    hmmwv.Initialize()  # Initialize the vehicle.
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    # Create the terrain for the vehicle to interact with.
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChMaterialSurfaceNSC()  # Create a contact material for the terrain.
    patch_mat.SetFriction(0.9)  # Set friction for the terrain.
    patch_mat.SetRestitution(0.01)  # Set restitution (bounciness) for the terrain.
    patch = terrain.AddPatch(patch_mat, ch.CSYSNORM, 100.0, 100.0)  # Add a patch to the terrain.
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()  # Initialize the terrain.
    
    # Create run-time visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV Vehicle Simulation')
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVector3d(1.5, -2.5, 5.5), ch.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

    # Add a visualization box
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True)
    box.SetPos(ch.ChVector3d(2, 2, 0.5))
    box.SetBodyFixed(True)
    hmmwv.GetSystem().Add(box)

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

    # Set up the sensor manager
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())
    # Create and configure a Lidar sensor
    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),  # body to which the sensor is attached
        10,  # scanning rate in Hz
        ch.ChFrame(ch.ChVector3d(0, 0, 2.5), ch.ChQuaterniond(1, 0, 0, 0)),  # offset pose
        100,  # horizontal samples
        1,  # vertical samples
        3.1415,  # horizontal field of view
        0.1,  # vertical field of view
        100.0  # max distance
    )
    lidar.PushFilter(sens.ChFilterLidarNoise(0.1))
    lidar.PushFilter(sens.ChFilterLidarReduce())
    lidar.PushFilter(sens.ChFilterLidarSavePCD())
    sens_manager.AddSensor(lidar)

    # Register the Lidar handler to publish data to ROS
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(25, lidar, "~/output/lidar"))

    # Start the simulation loop.
    time = 0
    time_step = 1e-3  # Define the simulation time step.
    time_end = 30  # Set the total duration of the simulation.
    # setup render related variables
    # Set the simulation time step
    time_step = 1e-3
    step_number = 0
    # Time interval between two render frames
    render_step_size = 1.0 / 25  # FPS = 25
    render_steps = math.ceil(render_step_size / time_step)
    hmmwv.GetVehicle().EnableRealtime(True)  # Enable real-time simulation for the vehicle.
    while vis.Run():  # Run the simulation loop.
        time = hmmwv.GetSystem().GetChTime()  # Update simulation time.
        # Render the scene
        if (step_number % render_steps == 0):
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
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

        # Update the sensor manager
        sens_manager.Update()

        step_number += 1

if __name__ == "__main__":
    main()