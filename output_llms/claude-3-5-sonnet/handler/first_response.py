import pychrono as chrono
import pychrono.ros as chr_ros
import rospy
from std_msgs.msg import Int32

# Initialize the Chrono system
system = chrono.ChSystemNSC()

# Set gravitational acceleration
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create a contact material
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetRestitution(0.1)

# Create the floor
floor = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True, material)
floor.SetBodyFixed(True)
floor.SetPos(chrono.ChVectorD(0, -0.5, 0))
system.Add(floor)

# Create a movable box
box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, material)
box.SetPos(chrono.ChVectorD(0, 5, 0))
system.Add(box)

# Initialize ROS node
rospy.init_node('chrono_simulation', anonymous=True)

# Create a custom ROS handler
class CustomROSHandler(chr_ros.ChROSHandler):
    def __init__(self):
        super().__init__()
        self.pub = rospy.Publisher('custom_topic', Int32, queue_size=10)
        self.counter = 0

    def PublishCallback(self):
        msg = Int32()
        msg.data = self.counter
        self.pub.publish(msg)
        self.counter += 1

# Create ROS manager
ros_manager = chr_ros.ChROSManager()

# Add handlers to the ROS manager
clock_handler = chr_ros.ChROSClockHandler()
ros_manager.AddHandler(clock_handler)

body_handler = chr_ros.ChROSBodyHandler(system)
ros_manager.AddHandler(body_handler)

transform_handler = chr_ros.ChROSTransformHandler()
ros_manager.AddHandler(transform_handler)

custom_handler = CustomROSHandler()
ros_manager.AddHandler(custom_handler)

# Initialize the ROS manager
ros_manager.Initialize()

# Set up the simulation loop
step_size = 0.01
simulation_time = 0.0
real_time_factor = 1.0

while not rospy.is_shutdown():
    # Advance the simulation
    system.DoStepDynamics(step_size)
    
    # Update ROS communication
    ros_manager.Synchronize(simulation_time)
    
    # Increment simulation time
    simulation_time += step_size
    
    # Maintain real-time execution
    rospy.sleep(step_size / real_time_factor)

# Finalize the ROS manager
ros_manager.Finalize()