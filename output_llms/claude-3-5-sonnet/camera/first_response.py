import pychrono as chrono
import pychrono.sensor as sens
import math
import numpy as np

# Initialize the Chrono system
chrono.SetChronoDataPath("/path/to/chrono/data/directory")
system = chrono.ChSystemNSC()

# Create the ground
ground = chrono.ChBodyEasyBox(20, 0.1, 20, 1000, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)
system.Add(ground)

# Load the triangular mesh
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh("path/to/your/mesh.obj", False, True)

# Create a rigid body and attach the mesh to it
mesh_body = chrono.ChBodyAuxRef()
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_body.SetBodyFixed(True)

visualization_mesh = chrono.ChTriangleMeshShape()
visualization_mesh.SetMesh(mesh)
mesh_body.AddAsset(visualization_mesh)

system.Add(mesh_body)

# Create a sensor manager
manager = sens.ChSensorManager(system)

# Create a camera and attach it to the mesh body
cam_pos = chrono.ChVectorD(-5, 0, 0)
cam_target = chrono.ChVectorD(0, 0, 0)
cam_up = chrono.ChVectorD(0, 1, 0)

camera = sens.ChCameraSensor(
    mesh_body,                 # body camera is attached to
    30.0,                      # update rate in Hz
    chrono.ChFrameD(cam_pos),  # offset pose
    1280,                      # image width
    720,                       # image height
    1.2                        # FOV
)

# Set camera properties
camera.SetName("Camera Sensor")
camera.SetLag(0.0)
camera.SetCollectionWindow(0.0)

# Add noise models to the camera
camera.PushFilter(sens.ChFilterCameraNoiseNormal(0.0, 0.1, 0.1))
camera.PushFilter(sens.ChFilterSave("camera/"))

# Add the camera to the manager
manager.AddSensor(camera)

# Simulation loop
time_step = 0.01
sim_time = 0
orbit_radius = 5
orbit_speed = 0.5

while sim_time < 10:  # Run for 10 seconds
    # Update camera position in an orbit
    angle = orbit_speed * sim_time
    new_cam_pos = chrono.ChVectorD(
        orbit_radius * math.cos(angle),
        0,
        orbit_radius * math.sin(angle)
    )
    camera.SetOffsetPose(chrono.ChFrameD(new_cam_pos, chrono.Q_from_AngAxis(angle, chrono.ChVectorD(0, 1, 0))))

    # Step the system
    system.DoStepDynamics(time_step)
    manager.Update()

    # Access and print camera data
    camera_data = camera.GetMostRecentRGBA8Buffer()
    if camera_data.HasData():
        rgba8_data = camera_data.GetRGBA8Data()
        print(f"Time: {sim_time:.2f}, Camera data shape: {rgba8_data.shape}")

    sim_time += time_step

# Cleanup
manager.ResetCamera(camera)