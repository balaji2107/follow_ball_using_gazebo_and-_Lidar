import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class FindDirection(Node):
    def __init__(self):
        super().__init__('find_direction_node')
        self.FOV_DEGREE=15
        self.MAX_INVALID=0.08
        self.laser_data=self.create_subscription(LaserScan,'/scan',self.lidar_callback,10)

        self.twist_publish=self.create_publisher(Twist,'/cmd_vel',10)
    
    def lidar_callback(self,msg_data):

        twist_keyboard = Twist()
        angle_data=[]
        total_val=len(msg_data.ranges)
        for i in range(total_val):
            self.angle=msg_data.angle_min + i * msg_data.angle_increment
            angle_data.append(self.angle)
        
        valid_ranges=[]
        valid_direction=[]

        for i in range(len(msg_data.ranges)):
            if not math.isnan(msg_data.ranges[i]) and not math.isinf(msg_data.ranges[i]) and msg_data.ranges[i]>self.MAX_INVALID:
                valid_ranges.append(msg_data.ranges[i])
                valid_direction.append(angle_data[i])

        if not valid_ranges:
            self.get_logger().warn("No valid LiDAR data! Skipping this cycle.")
            twist_keyboard.linear.x=0.0
            twist_keyboard.angular.z=0.0
            self.twist_publish.publish(twist_keyboard)
            return  

        min_dist = min(valid_ranges)
        min_index = valid_ranges.index(min_dist)
        closest_angle = valid_direction[min_index]

        KEEP_DISTANCE=0.4

        if closest_angle < -0.1:
            if min_dist>KEEP_DISTANCE:
                twist_keyboard.angular.z=-0.3
        elif closest_angle > 0.1:
            if min_dist>KEEP_DISTANCE:
                twist_keyboard.angular.z=0.3
        else:
            twist_keyboard.angular.z=0.0
            if min_dist>KEEP_DISTANCE:
                twist_keyboard.linear.x=0.3
            else:     
                twist_keyboard.linear.x=0.0
        
        self.twist_publish.publish(twist_keyboard)



        # if valid_ranges and valid_direction: 
        #     twist_keyboard = Twist()
            
        
        #     KP_FOLLOW = 0.8         
        #     DESIRED_SPEED = 0.3     # Constant forward speed
        #     KEEP_DISTANCE = 0.4     # Target distance to stop at (e.g., 40 cm from the ball)
            
        #     min_dist = min(valid_ranges)
        #     min_index = valid_ranges.index(min_dist)
        #     closest_angle = valid_direction[min_index]

        #     distance_error = min_dist - KEEP_DISTANCE # bot stop before that KEEP_DISTANCE
            
        #     twist_keyboard.linear.x = max(0.0, min(0.5, distance_error)) 
        #     twist_keyboard.angular.z = max(-1.0, min(1.0, closest_angle))
        #     self.twist_publish.publish(twist_keyboard)

        # else:
        #     twist_board=Twist()
        #     twist_board.linear.x = 0.0          
        #     twist_board.angular.z = 0.0
        #     self.twist_publish.publish(twist_board)


def main(args=None):
    rclpy.init(args=args)
    find_direction=FindDirection()
    rclpy.spin(find_direction)
    find_direction.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()