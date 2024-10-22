import chrono.chrono as c
import chrono.ros as ros
import rospy
from std_msgs.msg import Int32

# Initialize PyChrono environment
c.Initialize()

# Create Chrono system and set gravitational acceleration
sys = c.ChSystemSMC()
sys.Set_Gravity(c.ChVectorD(0, -9.81, 0))

# Set physical material properties
material_floor = c.ChMaterialSurfaceNSC()
material_floor.SetFriction(0.6)
material_box = c.ChMaterialSurfaceNSC()
material_box.SetFriction(0.6)

# Add fixed floor
floor = c.ChBodyEasyBox(1, 1, 0.1, material_floor)
floor.SetPos(c.ChVectorD(0, -0.5, 0))
floor.SetBodyFixed(True)
sys.Add(floor)

# Add movable box
box = c.ChBodyEasyBox(0.2, 0.2, 0.2, material_box)
box.SetPos(c.ChVectorD(0, 0.5, 0))
box.SetMass(1)
box.SetInertiaXX(c.ChVectorD(0.008, 0.008, 0.016))
sys.Add(box)

# Create custom ROS handler to publish integer messages
class CustomRosHandler(ros.ChRosNodeHandler):
    def __init__(self, name):
        ros.ChRosNodeHandler.__init__(self, name)
        self.pub = self.create_publisher('custom_topic', Int32, queue_size=10)

    def Update(self, my_system, my_items):
        msg = Int32()
        msg.data = int(box.GetPos().y)
        self.pub.publish(msg)

# Initialize ROS node and create ROS manager
rospy.init_node('pychrono_ros_node')
ros_manager = ros.ChRosManager()
ros_manager.Initialize('pychrono_ros_node')

# Configure ROS manager with clock, body, and transform handlers
ros_manager.RegisterClock()
ros_manager.RegisterBody(floor.GetName(), floor)
ros_manager.RegisterBody(box.GetName(), box)
ros_manager.RegisterTransform(box.GetName(), floor.GetName())

# Register custom ROS handler
custom_handler = CustomRosHandler('custom_handler')
ros_manager.RegisterHandler(custom_handler)

# Simulation loop
while not rospy.is_shutdown():
    sys.DoStepDynamics(0.01)
    ros_manager.Update()