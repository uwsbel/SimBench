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
    # Add a box to be sensed by a camera
    # -----------------------------------
    # Create a box object
    box = chrono.ChBodyEasyBox(side=1000, side=1000, side=1000, 1000)
    box.SetPos(chrono.ChVector3d(0, 0, 0))  # Set the position of the box
    box.SetFixed(True)  # Fix the box in space

    # Create a visual representation of the box
    box_shape = chrono.ChVisualShapeBox()
    box_shape.SetMesh(box)
    box_shape.SetName("Box")
    box_shape.SetMutable(False)  # Set the box to be immutable

    # Create a camera and attach it to the box
    camera = sens.ChCameraSensor(
        box,              # Body the camera is attached to
        update_rate,      # Camera update rate in Hz
        offset_pose,      # Offset pose of the camera
        image_width,      # Image width in pixels
        image_height,     # Image height in pixels
        fov                 # Camera's horizontal field of view in radians
    )
    camera.SetName("Camera Sensor")
    camera.SetLag(lag)  # Set the lag between sensing and data accessibility
    camera.SetCollectionWindow(exposure_time)  # Set the exposure time for the camera

    # ------------------------------------------------------------------
    # Create a filter graph for post-processing the data from the camera
    # ------------------------------------------------------------------
    # Apply noise model to the camera sensor based on the specified type
    if noise_model == "CONST_NORMAL":
        camera.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))  # Add constant normal noise
    elif noise_model == "PIXEL_DEPENDENT":
        camera.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))  # Add pixel-dependent noise
    elif noise_model == "NONE":
        # No noise model applied
        pass

    # Visualize the image before applying grayscale filter
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Before Grayscale Filter"))

    # Provide host access to the RGBA8 buffer from the camera
    camera.PushFilter(sens.ChFilterRGBA8Access())

    # Save the current image to a PNG file at the specified path
    if save:
        camera.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))

    # Convert the camera image to grayscale
    camera.PushFilter(sens.ChFilterGrayscale())

    # Visualize the grayscaled image
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Grayscale Image"))

    # Save the grayscaled image to a PNG file at the specified path
    if save:
        camera.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

    # Resize the image to the specified width and height
    camera.PushFilter(sens.ChFilterImageResize(int(image_width / 2), int(image_height / 2)))

    # Access the grayscaled image buffer as R8 pixels
    camera.PushFilter(sens.ChFilterR8Access())

    # Add the camera sensor to the manager
    manager.AddSensor(camera)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10  # Radius of the camera orbit
    orbit_rate = 0.5   # Rate of the camera orbit in radians per second
    ch_time = 0.0      # Initialize simulation time

    t1 = time.time()  # Record the start time of the simulation

    while ch_time < end_time:
        # Dynamically set the camera's position around the orbit
        camera.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))))

        # Access the RGBA8 buffer from the camera
        rgba8_buffer = camera.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print('RGBA8 buffer received from cam. Camera resolution: {0}x{1}'.format(rgba8_buffer.Width, rgba8_buffer.Height))
            print('First Pixel: {0}'.format(rgba8_data[0, 0, :]))

        # Update the sensor manager (render/save/filter data automatically)
        manager.Update()

        # Perform a step of dynamics simulation
        mphysicalSystem.DoStepDynamics(step_size)

        # Update the current simulation time
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

# -----------------
# Camera parameters
# -----------------

# Noise model attached to the sensor
noise_model = "CONST_NORMAL"  # Constant normal noise model

# Camera lens model
lens_model = sens.PINHOLE  # Pinhole lens model

# Update rate in Hz
update_rate = 30

# Image width and height
image_width = 1280
image_height = 720

# Camera's horizontal field of view
fov = 1.408  # Horizontal field of view in radians

# Lag (in seconds) between sensing and when data becomes accessible
lag = 0

# Exposure (in seconds) of each image
exposure_time = 0

# ---------------------
# Simulation parameters
# ---------------------

# Simulation step size
step_size = 1e-3

# Simulation end time
end_time = 20.0

# Save camera images
save = False

# Render camera images
vis = True

# Output directory
out_dir = "SENSOR_OUTPUT/"

# Main function entry point
main()