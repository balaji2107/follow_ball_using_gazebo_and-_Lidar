import rclpy 
import math
from std_msgs.msg import String
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from scipy.signal import medfilt


class FindMedian(Node):
    def __init__(self):
        super().__init__('Find_median_node')
        self.findMedianSubscriber=self.create_subscription(LaserScan,'/scan',self.laser_callback,10)
        self.MAX_INVALID_DISTANCE = 0.08
        self.MEDIAN_KERNEL_SIZE = 3
        self.FOV_DEGREE=15

        self.filter_median_pub=self.create_publisher(String,'closest_object_distance',10)

    def laser_callback(self,msg_data):

        find_radians=math.radians(self.FOV_DEGREE)
        angle_min_target=-find_radians
        angle_max_target=find_radians

        start_index=int((angle_min_target - msg_data.angle_min) / msg_data.angle_increment)
        end_index= int((angle_max_target-msg_data.angle_min) / msg_data.angle_increment)

        total_range=len(msg_data.ranges)
        start_index=max(0,start_index)
        end_index=min(total_range-1,end_index)

        frontal_ranges = msg_data.ranges[start_index:end_index + 1]

        
        valid_distances = [] # Initialize an empty list
        for d in frontal_ranges: 
            if not math.isinf(d) and not math.isnan(d) and d > self.MAX_INVALID_DISTANCE:
                valid_distances.append(d)

        filtered_slice = medfilt(valid_distances, kernel_size=self.MEDIAN_KERNEL_SIZE)

        if valid_distances:
            closest_distance = min(filtered_slice)
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
        self.filter_median_pub.publish(distance_msg)


def main(args=None):
    rclpy.init(args=args)
    find_median=FindMedian()
    rclpy.spin(find_median)
    find_median.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()