import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class LaserData(Node):

    def __init__(self):
        super().__init__('Laster_data_node')
        self.FOV_DEGREES=15
        self.MAX_INVALID_DISTANCE = 0.08
        self.laser_subscriber=self.create_subscription(LaserScan,'/scan',self.laser_calback,10)

        self.publisher_ = self.create_publisher(
            String, 
            'closest_object_distance', 
            10
        )

    def laser_calback(self, msg_data):
        # frontal_ranges = msg_data.ranges

        fov_radians = math.radians(self.FOV_DEGREES) 
        
        angle_min_target = -fov_radians
        angle_max_target = fov_radians

        
        try:
            start_index = int((angle_min_target - msg_data.angle_min) / msg_data.angle_increment)
            end_index = int((angle_max_target - msg_data.angle_min) / msg_data.angle_increment)
            
            num_ranges = len(msg_data.ranges)
            start_index = max(0, start_index)
            end_index = min(num_ranges - 1, end_index)
            
        except AttributeError as e:
            self.get_logger().error(f"Error accessing scan properties: {e}")
            return
        except ZeroDivisionError:
            self.get_logger().error("scan_msg.angle_increment is zero, cannot calculate indices.")
            return

        frontal_ranges = msg_data.ranges[start_index:end_index + 1]
       
        valid_distances = [
            d for d in frontal_ranges 
            if not math.isinf(d) and not math.isnan(d) and d > self.MAX_INVALID_DISTANCE
        ]
        
        if valid_distances:
            closest_distance = min(valid_distances)
        else:
            closest_distance = msg_data.range_max
            
                
        distance_msg = String()
        

        if closest_distance < msg_data.range_max:
             distance_str = f"{closest_distance:.3f} meters"
             self.get_logger().info(f'Closest object in full scan: {closest_distance:.3f} meters')
        else:
             distance_str = "No valid object detected in full scan FOV."
             self.get_logger().info('No valid object detected in full scan FOV.')

        distance_msg.data = distance_str
        self.publisher_.publish(distance_msg)


def main(args=None):
    rclpy.init(args=args)
    laser_data=LaserData()
    rclpy.spin(laser_data)
    laser_data.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()