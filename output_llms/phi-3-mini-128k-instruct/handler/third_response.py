import pychrono as ch
import pychrono.ros as chros
import rclpy.node
from std_msgs.msg import Int64

# Define a custom ROS handler for publishing integer messages.
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""

    def __init__(self, topic, publish_rate):
        super().__init__(publish_rate)  # Initialize the handler with the specified publish rate.

        self.topic = topic
        self.publisher: rclpy.node.Publisher = None
        self.ticker = 0  # Initialize a counter for published messages.

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Initialize the ROS publisher."""
        print(f"Creating publisher for topic {self.topic} ...")
        # Create a ROS publisher for the specified topic.
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, self.publish_rate)
        return True  # Return True to indicate successful initialization.

    def Tick(self, time: float):
        """Publish an integer message to the ROS topic."""
        print(f"Publishing {self.ticker} ...")
        msg = Int64()  # Create a message object of type Int64.
        msg.data = self.ticker  # Set the message data to the current ticker value.
        self.publisher.publish(msg)  # Publish the message to the ROS topic.
        self.ticker += 1  # Increment the ticker for the next message.

def main():
    # Create the Chrono simulation system.
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))  # Set gravitational acceleration.

    # Define physical material properties for contact.
    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)  # Set friction coefficient.

    # Create a floor object with a texture.
    floor_texture = ch.ChTexture("path/to/floor_texture.jpg")
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -1))  # Position the floor.
    floor.SetTexture(floor_texture)  # Set the texture for the floor.
    floor.SetFixed(True)  # Fix the floor in place.
    floor.SetName("base_link")  # Set the name for ROS communication.
    sys.Add(floor)  # Add the floor to the simulation system.

    # Create a box object with a texture.
    box_texture = ch.ChTexture("path/to/box_texture.jpg")
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  # Position the box above the floor.
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  # Rotate the box slightly.
    box.SetTexture(box_texture)  # Set the texture for the box.
    box.SetName("box")  # Set the name for ROS communication.
    sys.Add(box)  # Add the box to the simulation system.

    # Create and configure the ROS manager.
    ros_manager = chros.ChROSPythonManager()
    
    # Register a clock handler for the simulation time.
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    
    # Register a body handler to communicate the box's state.
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))
    
    # Register a transform handler for coordinate transformations.
    tf_handler = chros.ChROSTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    # Register the custom handler to publish messages.
    custom_handler = MyCustomHandler("~/my_topic", 10)  # Set the publish rate to 10 Hz.
    ros_manager.RegisterPythonHandler(custom_handler)

    # Initialize the ROS manager.
    ros_manager.Initialize()

    # Setup Irrlicht for visualization.
    irr_scene = ch.IrrlichtSceneManager()
    irr_scene.AddUniverse(ch.IrrlichtDevice::SINGLE_EXAMPLE)
    irr_scene.AddCamera("main_camera", ch.IrrlichtDevice::UNIVERSE_FOREGROUND)
    irr_scene.AddLight("main_light", ch.IrrlichtDevice::UNIVERSE_FOREGROUND)
    irr_scene.AddWindow("window", 1024, 768, "PyChrono Visualization", ch.IrrlichtDevice::WINDOW_EXCLUDE_MENU)

    # Setup Irrlicht for rendering.
    irr_scene.SetRenderWindow(ch.IrrlichtDevice::RenderWnd_EXCLUDE_MENU)
    irr_scene.SetWindowShown(True)
    irr_scene.AddCameraSceneNode(ch.IrrlichtDevice::UNIVERSE_FOREGROUND)

    # Setup Irrlicht for rendering the floor and box.
    floor_mesh = irr_scene.AddMesh("path/to/floor_mesh.obj", ch.IrrlichtDevice::UNIVERSE_FOREGROUND)
    box_mesh = irr_scene.AddMesh("path/to/box_mesh.obj", ch.IrrlichtDevice::UNIVERSE_FOREGROUND)
    floor_mesh.SetTexture("path/to/floor_texture.jpg")
    box_mesh.SetTexture("path/to/box_texture.jpg")

    # Setup variables for controlling rendering frame rate.
    step_number = 0
    render_step_size = 10
    render_steps = 100

    # Run the simulation loop.
    time = 0
    time_step = 1e-3  # Define the simulation time step.
    time_end = 30  # Set the duration for the simulation.

    realtime_timer = ch.ChRealtimeStepTimer()  # Create a real-time step timer.
    while time < time_end:
        sys.DoStepDynamics(time_step)  # Advance the simulation by one time step.
        time = sys.GetChTime()  # Update the simulation time.

        if not ros_manager.Update(time, time_step):  # Update ROS communication.
            break  # Exit the loop if the ROS manager indicates a problem.

        # Conditional rendering based on the render_steps variable.
        if step_number % render_step_size == 0:
            irr_scene.RenderWindow()
            step_number += 1

        realtime_timer.Spin(time_step)  # Maintain real-time step execution.

# Entry point of the script.
if __name__ == "__main__":
    main()