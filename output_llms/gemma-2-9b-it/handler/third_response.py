import pychrono as ch
import pychrono.ros as chros
import pychrono.irrlicht as irr

import rclpy.publisher
from std_msgs.msg import Int64

# Define a custom ROS handler for publishing integer messages.
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will publish integer messages to a specified ROS topic."""

    def __init__(self, topic, publish_rate):
        super().__init__(publish_rate)  # Initialize the handler with the specified publishing rate.

        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0  # Initialize a counter for published messages.

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        """Initialize the ROS publisher."""
        print(f"Creating publisher for topic {self.topic} ...")
        # Create a ROS publisher for the specified topic.
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 10)
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

    # Create a floor object.
    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -1))  # Position the floor.
    floor.SetFixed(True)  # Fix the floor in place.
    floor.SetName("base_link")  # Set the name for ROS communication.
    floor.SetTexture("path/to/floor_texture.png")  # Set texture for the floor
    sys.Add(floor)  # Add the floor to the simulation system.

    # Create a box object.
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))  # Position the box above the floor.
    box.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))  # Rotate the box slightly.
    box.SetName("box")  # Set the name for ROS communication.
    box.SetTexture("path/to/box_texture.png")  # Set texture for the box
    sys.Add(box)  # Add the box to the simulation system.

    # Create and configure the ROS manager.
    ros_manager = chros.ChROSPythonManager()
    
    # Register a clock handler for the simulation time.
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    
    # Register a body handler to communicate the box's state.
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))
    
    # Create and register a transform handler for coordinate transformations.
    tf_handler = chros.ChROSTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)
    
    # Register the custom handler to publish messages.
    custom_handler = MyCustomHandler("~/my_topic", 10)  # Set publish rate to 10 Hz
    ros_manager.RegisterPythonHandler(custom_handler)

    # Initialize the ROS manager.
    ros_manager.Initialize()

    # Visualization setup
    app = irr.IrrlichtDevice(irr.createIrrlichtDevice(
        irr.dimension2d(800, 600), 
        irr.videoMode(irr.driverType_OpenGL, 32, 0, 0), 
        False, 
        False, 
        False, 
        False
    ))
    
    driver = app.getVideoDriver()
    scene = app.getSceneManager()
    camera = scene.addCameraSceneNode(0, irr.vector3df(0, 0, 10), irr.vector3df(0, 0, 0))
    light = scene.addLightSceneNode(0, irr.vector3df(0, 0, 0), irr.SColorf(1, 1, 1, 1))

    # Rendering parameters
    step_number = 0
    render_step_size = 10
    render_steps = 10

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

        step_number += 1
        if step_number % render_step_size == 0:
            # Update the scene every few steps
            driver.beginScene(True, True, irr.SColor(255, 140, 0, 0))
            scene.drawAll()
            driver.endScene()

        realtime_timer.Spin(time_step)  # Maintain real-time step execution.

# Entry point of the script.
if __name__ == "__main__":
    main()