import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class PrintData(Node):
     
    def __init__(self):
        super().__init__('Print_data_node')
        self.laser_subscriber=self.create_subscription(LaserScan,'/scan',self.laser_callback,10)

    def laser_callback(self,msg_data):
        print("first node direction " + str(msg_data.angle_min))
        print("second node direction " + str(msg_data.angle_max))
        print("each node increment "+ str(msg_data.angle_increment))
        print("first node distance " +  str(msg_data.ranges[0]))
        print("second node distance "+  str(msg_data.ranges[1]))





def main(args=None):
    rclpy.init(args=args)
    printData=PrintData()
    rclpy.spin(printData)
    printData.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    main()
