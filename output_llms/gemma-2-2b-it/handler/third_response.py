import pychrono as ch
import pychrono.ros as chros
import irrlicht
import irrlicht.core as core
import irrlicht.video as video
import irrlicht.gui as gui
import irrlicht.scene as scene
import irrlicht.image as image

# Define a custom ROS handler for publishing integer messages.
class MyCustomHandler(chros.ChROSPythonHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""

    def __init__(self, topic, publish_rate=10):
        super().__init__(1)  # Initialize the handler with a 1 Hz publishing rate.
        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0
        self.publish_rate = publish_rate

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        print(f"Creating publisher for topic {self.topic} ...")
        # Create a ROS publisher for the specified topic.
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, self.publish_rate)
        return True  # Return True to indicate successful initialization.

    def Tick(self, time: float):
        print(f"Publishing {self.ticker} ...")
        msg = Int64()  # Create a message object of type Int64.
        msg.data = self.ticker  # Set the message data to the current ticker value.
        self.publisher.publish(msg)  # Publish the message to the ROS topic.
        self.ticker += 1  # Increment the ticker for the next message.

# Define Irrlicht parameters
irrlicht_window_width = 800
irrlicht_window_height = 600
irrlicht_window_title = "PyChrono Simulation"

# Create Irrlicht engine
engine = core.IrrlichtEngine()

# Create Irrlicht window
window = engine.createWindow(irrlicht_window_width, irrlicht_window_height, irrlicht_window_title)

# Create Irrlicht camera
camera = window.getCamera()
camera.setPosition(ch.ChVector3d(0, 0, 10))  # Position the camera
camera.setLookAt(ch.ChVector3d(0, 0, 0))  # Set the camera's look at point

# Create Irrlicht lights
light = scene.Light()
light.setPosition(ch.ChVector3d(0, 10, 0))  # Position the light
light.setColor(ch.ChColor(1, 1, 1, 1))  # Set the light color

# Create Irrlicht scene
scene = scene.Scene()
scene.addLight(light)

# Set textures for the floor and box
floor_texture = image.loadTexture("floor.png")
box_texture = image.loadTexture("box.png")

# Create Irrlicht floor and box objects
floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, ch.ChContactMaterialNSC())
floor.SetPos(ch.ChVector3d(0, 0, -1))  # Position the floor
floor.SetFixed(True)  # Fix the floor in place
floor.SetName("base_link")  # Set the name for ROS communication
scene.addBody(floor)

box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, ch.ChContactMaterialNSC())
box.SetPos(ch.ChVector3d(0, 0, 5))  # Position the box above the floor
box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  # Rotate the box slightly
box.SetName("box")  # Set the name for ROS communication
scene.addBody(box)

# Create and configure the ROS manager
ros_manager = chros.ChROSPythonManager()

# Register a clock handler for the simulation time
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# Register a body handler to communicate the box's state
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))

# Create and register a transform handler for coordinate transformations
tf_handler = chros.ChROSTFHandler(30)
tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)

# Register the custom handler to publish messages
custom_handler = MyCustomHandler("~/my_topic", publish_rate=10)
ros_manager.RegisterPythonHandler(custom_handler)

# Initialize the ROS manager
ros_manager.Initialize()

# Simulation loop
time = 0
time_step = 1e-3  # Define the simulation time step
time_end = 30  # Set the duration for the simulation
realtime_timer = ch.ChRealtimeStepTimer()

while time < time_end:
    sys.DoStepDynamics(time_step)  # Advance the simulation by one time step
    time = sys.GetChTime()  # Update the simulation time

    if not ros_manager.Update(time, time_step):  # Update ROS communication
        break  # Exit the loop if the ROS manager indicates a problem.

    realtime_timer.Spin(time_step)  # Maintain real-time step execution

# Render the scene
while engine.isRunning():
    engine.run()

# Clean up Irrlicht resources
engine.shutdown()