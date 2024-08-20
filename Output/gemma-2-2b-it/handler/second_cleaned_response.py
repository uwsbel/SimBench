import pychrono as ch
import pychrono.ros as chros

import rclpy.publisher
from std_msgs.msg import String


class MyCustomHandler(chros.ChROSHandler):
    

    def __init__(self, topic):
        super().__init__(1)  

        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None
        self.ticker = 0  
        self.message = "Hello, world! At time: "

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        
        print(f"Creating publisher for topic {self.topic} ...")
        
        self.publisher = interface.GetNode().create_publisher(String, self.topic, 1)
        return True  

    def Tick(self, time: float):
        
        print(f"Publishing {self.message} ...")
        msg = String()  
        msg.data = self.message + str(self.ticker)
        self.publisher.publish(msg)  
        self.ticker += 1