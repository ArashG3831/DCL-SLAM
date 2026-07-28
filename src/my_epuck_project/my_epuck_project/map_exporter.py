import rclpy
from my_epuck_interfaces.msg import PeerMap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class MapExporter(Node):
    """Export only this robot's original local SLAM map as source-aware evidence."""

    def __init__(self):
        super().__init__('map_exporter')
        self.declare_parameter('source_robot_id', '')
        self.declare_parameter('input_topic', 'map')
        self.declare_parameter('output_topic', '/cslam/local_map')
        self.declare_parameter('export_rate_hz', 1.0)

        self.source_robot_id = self.get_parameter('source_robot_id').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        export_rate = float(self.get_parameter('export_rate_hz').value)
        if not self.source_robot_id:
            raise ValueError('source_robot_id must not be empty')
        if export_rate <= 0.0:
            raise ValueError('export_rate_hz must be positive')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(PeerMap, output_topic, qos)
        self.map_subscription = self.create_subscription(
            OccupancyGrid, input_topic, self.map_callback, qos
        )
        self.latest_map = None
        self.revision = 0
        self.timer = self.create_timer(1.0 / export_rate, self.export_map)
        self.get_logger().info(
            f'Exporting local evidence only: {self.resolve_topic_name(input_topic)} '
            f'-> {self.resolve_topic_name(output_topic)} at {export_rate:.2f} Hz '
            f'as source {self.source_robot_id}'
        )

    def map_callback(self, message):
        self.latest_map = message

    def export_map(self):
        if self.latest_map is None:
            return
        self.revision += 1
        message = PeerMap()
        message.source_robot_id = self.source_robot_id
        message.revision = self.revision
        message.export_stamp = self.get_clock().now().to_msg()
        message.local_evidence_only = True
        message.occupancy_grid = self.latest_map
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MapExporter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
