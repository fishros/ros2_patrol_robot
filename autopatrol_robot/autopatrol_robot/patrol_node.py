from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from autopatrol_interfaces.srv import SpeachText
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)
        # 导航相关定义
        self.declare_parameter('target_points', [0.0, 0.0, 0.0])
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.target_points = self.get_parameter('target_points').value
        self.initial_point = self.get_parameter('initial_point').value
        # 实时位置获取 TF 相关定义
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        # 语音合成客户端
        self.speach_client = self.create_client(SpeachText, 'speech_text')
        # 订阅与保存图像相关定义
        self.declare_parameter('image_save_path', '')
        self.image_save_path = self.get_parameter('image_save_path').value
        self.bridge = CvBridge()
        self.latest_image = None
        self.subscription_image = self.create_subscription(
            Image, '/camera_sensor/image_raw', self.image_callback, 10)

    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        通过 x,y,yaw 合成 PoseStamped
        """
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        rotation_quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = rotation_quat[0]
        pose.pose.orientation.y = rotation_quat[1]
        pose.pose.orientation.z = rotation_quat[2]
        pose.pose.orientation.w = rotation_quat[3]
        return pose

    def init_robot_pose(self):
        """
        初始化机器人位姿
        """
        # 从参数获取初始化点
        self.initial_point = self.get_parameter('initial_point').value
        # 合成位姿并进行初始化
        self.setInitialPose(self.get_pose_by_xyyaw(
            self.initial_point[0], self.initial_point[1], self.initial_point[2]))
        # 等待直到导航激活
        self.waitUntilNav2Active()

    def get_target_points(self):
        """
        通过参数值获取目标点集合        
        """
        points = []
        self.target_points = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points)/3)):
            x = self.target_points[index*3]
            y = self.target_points[index*3+1]
            yaw = self.target_points[index*3+2]
            points.append([x, y, yaw])
            self.get_logger().info(f"获取到目标点: {index}->({x},{y},{yaw})")
        return points
    
    def nav_to_pose(self, target_pose):
        """
        导航到指定位姿
        """
        self.waitUntilNav2Active()
        result = self.goToPose(target_pose)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback:
                self.get_logger().info(
                    f'预计: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s 后到达')
        # 最终结果判断
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航结果：成功')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('导航结果：被取消')
        elif result == TaskResult.FAILED:
            self.get_logger().error('导航结果：失败')
        else:
            self.get_logger().error('导航结果：返回状态无效')

    def get_current_pose(self):
        """
        通过TF获取当前位姿
        """
        while rclpy.ok():
            try:
                tf = self.buffer.lookup_transform(
                    "map", "base_footprint", rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
                transform = tf.transform
                rotation_euler = euler_from_quaternion([
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w
                ])
                self.get_logger().info(
                    f"平移:{transform.translation},旋转四元数:{transform.rotation}:旋转欧拉角:{rotation_euler}")
                return transform
            except Exception as e:
                self.get_logger().warn(f"不能够获取坐标变换，原因: {str(e)}")

    def image_callback(self, msg):
        self.latest_image = msg

    def record_image(self):
        """
        记录图像
        """
        pose = self.get_current_pose()
        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image)
        cv2.imwrite(
            f"{self.image_save_path}image_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.png", cv_image)

    def speach_text(self, text):
        """
        调用服务播放语音
        """
        while not self.speach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('语音课程服务未上线，等待中。。。')

        request = SpeachText.Request()
        request.text = text
        future = self.speach_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            result = future.result().result
            if result:
                self.get_logger().info(f"语音合成成功：{text}")
            else:
                self.get_logger().warn(f"语音合成失败：{text}")
        else:
            self.get_logger().warn("语音合成服务请求失败")


def main():
    rclpy.init()
    patrol = PatrolNode()
    patrol.speach_text(text="正在初始化位置")
    patrol.init_robot_pose()
    patrol.speach_text(text="位置初始化完成")

    while rclpy.ok():
        for point in patrol.get_target_points():
            x, y, yaw = point[0], point[1], point[2]
            # 导航到目标点
            target_pose = patrol.get_pose_by_xyyaw(x, y, yaw)
            patrol.speach_text(text=f"准备前往目标点{x},{y}")
            patrol.nav_to_pose(target_pose)
            # 记录图像
            # patrol.speach_text(text=f"已到达目标点{x},{y},准备记录图像")
            # patrol.record_image()
            # patrol.speach_text(text=f"图像记录完成")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
