from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from autopatrol_interfaces.srv import SpeachText
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Image
import cv2

class RobotNav(BasicNavigator):
    def __init__(self, node_name='robot_nav'):
        super().__init__(node_name)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.speach_client = self.create_client(SpeachText, 'speech_text')
        
        # self.subscription.

    def get_pose_by_xyyaw(self, frame_id, x, y, yaw):
        """
        通过 frame_id,x,y,yaw 合成 PoseStamped
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

    def init_robot_pose(self, initial_pose):
        """
        初始化机器人位姿
        """
        self.setInitialPose(initial_pose)
        self.waitUntilNav2Active()

    def nav_to_pose(self, target_pose):
        self.waitUntilNav2Active()
        result = self.goToPose(target_pose)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            # self.get_logger().info(
            #     f'预计: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s 后到达')
            # 超时自动取消
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                self.cancelTask()
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

    def current_pose(self):
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



    def record_image(self):
        # Implement your image recording logic here
        class _vfm(object):
            def __init__(self) -> None:
                self.msg = None
                
            def cb(self, msg):
                self.msg = msg
        vfm = _vfm()
        recv_msg = None
        def image_callback(msg):
            self.get_logger().info("recv image")
            nonlocal recv_msg
            recv_msg = msg
        subscription = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            vfm.cb,
            1
        )
        import time
        while rclpy.ok() and vfm.msg is None:
            rclpy.spin_once(self)
            time.sleep(0.1)
        # unsubcription
        subscription.destroy()

    def speach_text(self, text):
        """
        调用服务播放语音
        """
        while not self.speach_client.wait_for_service(timeout_sec=1.0):
             self.get_logger().info('Service not available. Waiting...')

        request = SpeachText.Request()
        request.text = text
        future = self.speach_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            result = future.result().result
            if result:
                print("Speech text request succeeded.")
            else:
                print("Speech text request failed.")
        else:
            print("Service call failed.")



def task_spin():
    # Implement your task execution logic here
    pass


if __name__ == '__main__':
    rclpy.init()
    nav = RobotNav()

    nav.speach_text(text="你好，我准备出发了")
    initpose = nav.get_pose_by_xyyaw("map",x=0.0, y=0.0, yaw=0.0)
    nav.init_robot_pose(initpose)

    target_pose = nav.get_pose_by_xyyaw("map",x=1.0, y=2.0, yaw=3.14)
    nav.nav_to_pose(target_pose)
    nav.speach_text(text="你好，我到达了第二个点")
    nav.record_image()

    target_pose = nav.get_pose_by_xyyaw("map",x=-4.5, y=1.5, yaw=1.57)
    nav.nav_to_pose(target_pose)
    nav.speach_text(text="你好，我到达了第一个点")
    nav.record_image()


    target_pose = nav.get_pose_by_xyyaw("map",x=-8.0, y=-5.0, yaw=1.57)
    nav.nav_to_pose(target_pose)
    nav.speach_text(text="你好，我到达了第三个点")
    nav.record_image()

    target_pose = nav.get_pose_by_xyyaw("map",x=-5.0, y=1.0, yaw=3.14)
    nav.nav_to_pose(target_pose)
    nav.speach_text(text="你好，我到达了第四个点")
    nav.record_image()

    target_pose = nav.get_pose_by_xyyaw("map",x=0.0, y=0.0, yaw=0.0)
    nav.nav_to_pose(target_pose)
    nav.speach_text(text="你好，我到达了第五个点")
    nav.record_image()

    # Call your task execution function here
    task_spin()

    rclpy.shutdown()
