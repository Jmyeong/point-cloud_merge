import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

class PointCloudMerge(Node):

    def __init__(self):
        super().__init__('pointcloud_merge')
        self.merged_pub = self.create_publisher(PointCloud2, '/merged_point_cloud', 10)
        self.subscriber_front = self.create_subscription(
            PointCloud2, '/wonbot500_01/vl_l1_front/point_cloud', self.front_callback, 10)
        self.subscriber_rear = self.create_subscription(
            PointCloud2, '/wonbot500_01/vl_l1_rear/point_cloud', self.rear_callback, 10)

        self.front_pc = None
        self.rear_pc = None

    def front_callback(self, msg):
        self.front_pc = msg
        self.merge_and_publish()

    def rear_callback(self, msg):
        self.rear_pc = msg
        self.merge_and_publish()

    def merge_and_publish(self):
        if self.front_pc is not None and self.rear_pc is not None:
            merged_pc = self.merge_pointclouds(self.front_pc, self.rear_pc)
            self.merged_pub.publish(merged_pc)

    def merge_pointclouds(self, front_pc, rear_pc):
        front_data = self.extract_points(front_pc)
        rear_data = self.extract_points(rear_pc)

        merged_data = np.vstack((front_data, rear_data))

        merged_pc = self.numpy_to_pointcloud(merged_data, front_pc)

        return merged_pc

    def extract_points(self, pc_msg):
        # Extracting points from PointCloud2 manually without using sensor_msgs.point_cloud2
        pc_data = pc_msg.data
        fields = pc_msg.fields
        point_step = pc_msg.point_step
        data = np.frombuffer(pc_data, dtype=np.uint8).reshape(-1, point_step)

        points = []
        for point in data:
            point_values = []
            for field in fields:
                offset = field.offset
                dtype = np.dtype(np.uint8).base
                value = int.from_bytes(point[offset:offset+field.count*dtype.itemsize], byteorder='little', signed=False)
                point_values.append(value)
            points.append(point_values)
        
        return np.array(points)

    def numpy_to_pointcloud(self, pc_data, ref_pc_msg):
        # Convert the NumPy array back to PointCloud2
        pc_msg = PointCloud2()
        pc_msg.header = ref_pc_msg.header
        pc_msg.header.frame_id = 'wonbot500_01/robot_frame'  # Setting the frame_id
        pc_msg.height = 1
        pc_msg.width = len(pc_data)
        pc_msg.fields = ref_pc_msg.fields
        pc_msg.is_bigendian = False
        pc_msg.point_step = pc_msg.width * pc_msg.height
        pc_msg.row_step = pc_msg.point_step * pc_msg.width
        pc_msg.is_dense = True
        pc_msg.data = pc_data.astype(np.float32).tobytes()
        
        return pc_msg

def main(args=None):
    rclpy.init(args=args)
    pointcloud_merge = PointCloudMerge()
    rclpy.spin(pointcloud_merge)
    pointcloud_merge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

