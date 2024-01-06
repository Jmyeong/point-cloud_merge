import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class TopicMergingNode(Node):

    def __init__(self):
        super().__init__('topic_merging_node')
        self.subscription1 = self.create_subscription(
            PointCloud2, '/wonbot500_01/vl_l1_front/point_cloud', self.topic1_callback, 10)
        self.subscription2 = self.create_subscription(
            PointCloud2, '/wonbot500_01/vl_l1_rear/point_cloud', self.topic2_callback, 10)
        self.publisher = self.create_publisher(PointCloud2, '/merged_topic', 10)
        self.message = PointCloud2()
        #PointCloud2().header.frame_id = 'wonbot500_01/robot_frame'

    def topic1_callback(self, msg):
        # Handle message from topic 1
        self.message.data = msg.data
        self.message.height = msg.height
        self.message.width = msg.width
        self.message.fields = msg.fields
        self.message.is_bigendian = msg.is_bigendian
        self.message.point_step = msg.point_step
        self.message.row_step = msg.row_step
        self.message.is_dense = msg.is_dense
        
        self.message.header.frame_id = 'wonbot500_01/robot_frame'
        self.publish_message()

    def topic2_callback(self, msg):
        # Handle message from topic 2
        self.message.data = msg.data
        self.message.height = msg.height
        self.message.width = msg.width
        self.message.fields = msg.fields
        self.message.is_bigendian = msg.is_bigendian
        self.message.point_step = msg.point_step
        self.message.row_step = msg.row_step
        self.message.is_dense = msg.is_dense
        
        self.message.header.frame_id = 'wonbot500_01/robot_frame'
        self.publish_message()

    def publish_message(self):
        # Publish merged message
        
        self.publisher.publish(self.message)


def main(args=None):
    rclpy.init(args=args)
    node = TopicMergingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
