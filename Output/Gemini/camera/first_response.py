import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono.sensor as sens

# ---------------------------------------------------------------------
#  Create the simulation system and add items
# ---------------------------------------------------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))

# Set default contact material
system.SetSolverMaxIterations(100)
chrono.SetChronoDataPath("./chrono_data/")

# Load the mesh from an OBJ file
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontOBJ("path/to/your/mesh.obj")  # Replace with your OBJ file path
mesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))  # Position/rotate if needed

# Create a visual material for the mesh
vis_mesh = chrono.ChVisualShapeTriangleMesh()
vis_mesh.SetMesh(mesh)
vis_mesh.SetCheckFaceNormals(True)

# Create a body based on the mesh and add it to the system
mesh_body = chrono.ChBodyEasyMesh(
    mesh,  # Mesh data
    1000,  # Density (adjust as needed)
    True,  # Compute mass and inertia
    True,  # Enable visualization asset
    False  # Enable collision shape (not needed for static body)
)
mesh_body.AddVisualShape(vis_mesh)
mesh_body.SetBodyFixed(True)  # Fix the mesh in space
system.Add(mesh_body)

# Create a camera sensor
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVectorD(5, 5, 5), chrono.ChColor(1, 1, 1), 50)

camera = sens.ChCameraSensor(
    mesh_body,               # Body the sensor is attached to
    10,                     # Update rate in Hz
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 2), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),  # Position and direction of the camera
    1280,                    # Image width
    720,                     # Image height
    chrono.CH_C_PI / 4,     # Field of view (FOV) in radians
)
camera.SetName("Camera Sensor")
camera.AddNoise(sens.GaussianNoiseModel(0.0, 5))  # Add Gaussian noise (adjust parameters)
camera.PushFilter(sens.MedianFilter(5))  # Apply a median filter (adjust window size)

# Create visualisations for the sensor buffers
camera_vis = sens.ChVisualizationSensor(camera)
camera_vis.SetTitle("Camera Sensor")
camera_vis.SetWindowSize(1280, 720)
camera_vis.SetCameraPos(chrono.ChVectorD(0, 0, -5))
manager.AddSensor(camera)
manager.AddVisualization(camera_vis)

# ---------------------------------------------------------------------
#  Simulation loop
# ---------------------------------------------------------------------
time_step = 1e-3
time = 0

while time < 10:
    system.DoStepDynamics(time_step)
    time += time_step

    # Update camera position (orbiting)
    orbit_radius = 3
    orbit_speed = 0.5
    camera_pos = chrono.ChVectorD(
        orbit_radius * chrono.ChCos(orbit_speed * time),
        0,
        orbit_radius * chrono.ChSin(orbit_speed * time)
    )
    camera.SetOffsetPos(chrono.ChFrameD(camera_pos, chrono.Q_from_AngAxis(orbit_speed * time, chrono.ChVectorD(0, 1, 0))))

    manager.Update()

    # Print camera buffer data (adjust as needed)
    # Example: print(camera.GetMostRecentRGBA8Buffer().GetRGBA8Data())

# ---------------------------------------------------------------------
#  Postprocessing
# ---------------------------------------------------------------------
# Optionally save simulation results or generate animations using PyChrono's postprocessing features