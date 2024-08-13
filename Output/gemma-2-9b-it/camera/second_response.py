import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time

def main():
    # -----------------
    # Create the system
    # -----------------
    # Initialize the Chrono physical system (non-smooth contact)
    mphysicalSystem = chrono.ChSystemNSC()

    # -----------------------------------
    # Add a mesh to be sensed by a camera
    # -----------------------------------
    # Load a triangular mesh from a Wavefront .obj file
    mmesh = chrono.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    # Scale the mesh uniformly by a factor of 2
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))

    # Create a visual representation of the mesh
    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)  # Set the mesh to be immutable

    # Create a body to which the visual shape will be attached
    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))  # Set the position of the body
    mesh_body.AddVisualShape(trimesh_shape)  # Attach the visual shape to the body
    mesh_body.SetFixed(True)  # Fix the body in space
    mphysicalSystem.Add(mesh_body)  # Add the body to the physical system

    # -----------------------
    # Create a sensor manager
    # -----------------------
    # Initialize the sensor manager to manage all sensors in the simulation
    manager = sens.ChSensorManager(mphysicalSystem)

    # Add point lights to the scene for illumination
    intensity = 1.0  # Set the light intensity
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))

    # ------------------------------------------------
    # Create a camera and add it to the sensor manager
    # ------------------------------------------------
    # Define the camera offset pose relative to the body it is attached to
    offset_pose = chrono.ChFramed(chrono.ChVector3d(-7, 0, 2), chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0)))

    # Initialize the camera sensor
    cam = sens.ChCameraSensor(
        mesh_body,              # Body the camera is attached to
        update_rate,            # Camera update rate in Hz
        offset_pose,            # Offset pose of the camera
        image_width,            # Image width in pixels
        image_height,           # Image height in pixels
        fov                     # Camera's horizontal field of view in radians
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)  # Set the lag between sensing and data accessibility
    cam.SetCollectionWindow(exposure_time)  # Set the exposure time for the camera

    # ------------------------------------------------------------------
    # Create a filter graph for post-processing the data from the camera
    # ------------------------------------------------------------------
    # Apply noise model to the camera sensor based on the specified type
    if noise_model == "CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))  # Add constant normal noise
    elif noise_model == "PIXEL_DEPENDENT":
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))  # Add pixel-dependent noise
    elif noise_model == "NONE":
        # No noise model applied
        pass

    # Visualize the image before applying grayscale filter
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale Filter"))

    # Provide host access to the RGBA8 buffer from the camera
    cam.PushFilter(sens.ChFilterRGBA8Access())

    # Save the current image to a PNG file at the specified path
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))

    # Convert the camera image to grayscale
    cam.PushFilter(sens.ChFilterGrayscale())

    # Visualize the grayscaled image
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Grayscale Image"))

    # Save the grayscaled image to a PNG file at the specified path
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

    # Resize the image to the specified width and height
    cam.PushFilter(sens.ChFilterImageResize(int(image_width / 2), int(image_height / 2)))

    # Access the grayscaled image buffer as R8 pixels
    cam.PushFilter(sens.ChFilterR8Access())

    # Add the camera sensor to the manager
    manager.AddSensor(cam)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10  # Radius of the camera orbit
    orbit_rate = 0.1   # Rate of the camera orbit in radians per second
    ch_time = 0.0      # Initialize simulation time

    t1 = time.time()  # Record the start time of the simulation

    while ch_time < end_time:
        # Dynamically set the camera's position around the orbit
        cam.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))))

        # Access the RGBA8 buffer from the camera
        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
print("error happened with only start ```python")