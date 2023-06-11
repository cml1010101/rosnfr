import rclpy
from rclpy.node import Node
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray, AprilTagDetection
from networktables import NetworkTables
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
class NTClient(Node):
    def __init__(self):
        super().__init__('ntclient')
        self.declare_parameter('apriltag_interface_namespaces', rclpy.Parameter.Type.STRING_ARRAY)
        for param in self.get_parameter('apriltag_interface_namespaces').get_parameter_value().string_array_value:
            self.get_logger().info('Subscribing to %s' % (param + '/tag_detections'))
            self.create_subscription(AprilTagDetectionArray, '/' + param + '/tag_detections',
                lambda arr : self.on_detect(param, arr), 10)
        #if self.get_parameter('publish_odometry_to_nav2').get_parameter_value().bool_value:
        #    self.odom_publisher = self.create_publisher(Odometry, 'wheel/odometry', 10)
        #    self.odom_entry = NetworkTables.getTable('ros').getSubTable('odom')
        #    self.create_timer(0.02, self.timer_callback)
        NetworkTables.initialize('10.1.72.2')
        self.ros_table = NetworkTables.getTable('/ros')
        self.status_entry = NetworkTables.getEntry('/ros/status')
        self.status_entry.setString('online')
    def on_detect(self, param: str, arr: AprilTagDetectionArray):
        camera_table = self.ros_table.getSubTable('camera_' + param)
        ids = []
        data = []
        for detection in arr.detections:
            detection: AprilTagDetection
            ids.append(detection.id)
            pose: Pose = detection.pose.pose.pose
            data.append(detection.pose.header.stamp.sec + detection.pose.header.stamp.nanosec / (10 ** 9))
            data.append(pose.position.x)
            data.append(pose.position.y)
            data.append(pose.position.z)
            data.append(pose.orientation.w)
            data.append(pose.orientation.x)
            data.append(pose.orientation.y)
            data.append(pose.orientation.z)
        camera_table.getEntry('ids').setDoubleArray(ids)
        camera_table.getEntry('data').setDoubleArray(data)
def main(args=None):
    rclpy.init(args=args)
    ntclient = NTClient()
    rclpy.spin(ntclient)
    ntclient.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()