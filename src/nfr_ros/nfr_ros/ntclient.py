import rclpy
from rclpy.node import Node
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray, AprilTagDetection
from networktables import NetworkTables
from geometry_msgs.msg import Pose, Twist, PoseStamped
from nav_msgs.msg import Odometry
from math import sin, cos
from robot_navigator import BasicNavigator, NavigationResult
from ament_index_python.packages import get_package_share_directory
import os
class NTClient(Node):
    def __init__(self):
        super().__init__('ntclient')
        self.declare_parameter('apriltag_interface_namespaces', rclpy.Parameter.Type.STRING_ARRAY)
        for param in self.get_parameter('apriltag_interface_namespaces').get_parameter_value().string_array_value:
            self.get_logger().info('Subscribing to %s' % (param + '/tag_detections'))
            self.create_subscription(AprilTagDetectionArray, '/' + param + '/tag_detections',
                lambda arr : self.on_detect(param, arr), 10)
        if self.get_parameter('publish_odometry_to_nav2').get_parameter_value().bool_value:
            self.odom_publisher = self.create_publisher(Odometry, 'wheel/odometry', 10)
            self.odom_table = NetworkTables.getTable('ros').getSubTable('odom')
            self.create_timer(0.02, self.timer_callback)
        NetworkTables.initialize('10.1.72.2')
        self.ros_table = NetworkTables.getTable('/ros')
        self.status_entry = NetworkTables.getEntry('/ros/status')
        self.status_entry.setString('online')
        self.navigator = BasicNavigator()
        self.navigator.lifecycleStartup()
        self.navigator.waitUntilNav2Active()
        self.navigator.changeMap(os.path.join(get_package_share_directory('nfr_ros'), 'config', 'charged_up.yaml'))
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
    def timer_callback(self):
        timestamp = self.odom_table.getEntry('timestamp').getDouble(0.0)
        x = self.odom_table.getEntry('x').getDouble(0.0)
        y = self.odom_table.getEntry('y').getDouble(0.0)
        theta = self.odom_table.getEntry('theta').getDouble(0.0)
        vx = self.odom_table.getEntry('vx').getDouble(0.0)
        vy = self.odom_table.getEntry('vy').getDouble(0.0)
        vtheta = self.odom_table.getEntry('vtheta').getDouble(0.0)
        odometry: Odometry = Odometry()
        odometry.header.stamp = timestamp
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_link'
        pose: Pose = odometry.pose.pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = sin(theta / 2)
        pose.orientation.w = cos(theta / 2)
        twist: Twist = odometry.twist.twist
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = vtheta
        self.odom_publisher.publish(odometry)
        if self.ros_table.getSubTable("nav").getEntry("do_nav").getBoolean(False):
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = self.ros_table.getSubTable("nav").getEntry("x").getDouble(0.0)
            goal_pose.pose.position.y = self.ros_table.getSubTable("nav").getEntry("y").getDouble(0.0)
            theta = self.ros_table.getSubTable("nav").getEntry("theta").getDouble(0.0)
            goal_pose.pose.orientation.z = sin(theta / 2)
            goal_pose.pose.orientation.w = cos(theta / 2)
            self.navigator.goToPose(goal_pose)
            self.ros_table.getSubTable("nav").getEntry("do_nav").setBoolean(False)
        feedback = self.navigator.getFeedback()
        self.ros_table.getSubTable("nav").getEntry("cmd_vx").setDouble(feedback)
def main(args=None):
    rclpy.init(args=args)
    ntclient = NTClient()
    rclpy.spin(ntclient)
    ntclient.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()