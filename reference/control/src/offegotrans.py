#!/usr/bin/env python3
import threading
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion
import tf

class OdometryOffsetNode:
    def __init__(self):
        # 从ROS param读取偏移值（默认0.0）
        self.offset_x = rospy.get_param('~offset_x', -2.0)
        self.offset_y = rospy.get_param('~offset_y', 0.0)
        self.offset_z = rospy.get_param('~offset_z', 0.0)
        
        # 订阅原始odometry
        self.odom_sub = rospy.Subscriber('/Odometry', Odometry, self.odom_callback)
        
        # 发布偏移后的odometry
        self.odom_pub = rospy.Publisher('/Odometry_offset', Odometry, queue_size=10)
        
        rospy.loginfo(f"Odometry offset node started with offsets: x={self.offset_x}, y={self.offset_y}, z={self.offset_z}")

    def odom_callback(self, msg):
        # 复制消息
        offset_odom = msg
        
        # 应用偏移（只偏移位置，速度/姿态保持原样）
        offset_odom.pose.pose.position.x += self.offset_x
        offset_odom.pose.pose.position.y += self.offset_y
        offset_odom.pose.pose.position.z += self.offset_z
        
        # 更新时间戳并发布
        offset_odom.header.stamp = rospy.Time.now()
        self.odom_pub.publish(offset_odom)


class T265ToMavrosNode:
    def __init__(self):
        # 缓存最新odom位置/姿态，用于定频发布
        self.pos = None
        self.q = None
        self.lock = threading.Lock()

        self.t265_sub = rospy.Subscriber('/Odometry_offset', Odometry, self.t265_callback, queue_size=5)
        self.vision_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)

        self.publish_rate = 50.0
        self.thread = threading.Thread(target=self.publish_loop)
        self.thread.daemon = True
        self.thread.start()

    def t265_callback(self, msg):
        # 只保存最新数据，发布线程读取
        with self.lock:
            self.pos = msg.pose.pose.position
            self.q = msg.pose.pose.orientation

    def publish_loop(self):
        # 保持与原C++节点一致的50Hz输出
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            vision = PoseStamped()
            with self.lock:
                if self.pos is not None:
                    vision.pose.position.x = self.pos.x
                    vision.pose.position.y = self.pos.y
                    vision.pose.position.z = self.pos.z
                if self.q is not None:
                    vision.pose.orientation.x = self.q.x
                    vision.pose.orientation.y = self.q.y
                    vision.pose.orientation.z = self.q.z
                    vision.pose.orientation.w = self.q.w
            vision.header.stamp = rospy.Time.now()
            self.vision_pub.publish(vision)
            rate.sleep()


class EgoTransferNode:
    def __init__(self):
        # 预设固定旋转（与原脚本一致）
        self.local_pose = PoseStamped()
        self.local_pose.header.frame_id = 'world'
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.q = Quaternion([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])

        self.vision_sub = rospy.Subscriber(
            '/mavros/vision_pose/pose', PoseStamped, self.vision_callback, queue_size=1
        )
        self.position_pub = rospy.Publisher('/camera_pose', PoseStamped, queue_size=1)

        self.publish_rate = 60.0
        self.thread = threading.Thread(target=self.publish_loop)
        self.thread.daemon = True
        self.thread.start()

    def vision_callback(self, data):
        # 只做姿态变换与转发
        self.local_pose.pose.position.x = data.pose.position.x
        self.local_pose.pose.position.y = data.pose.position.y
        self.local_pose.pose.position.z = data.pose.position.z
        q_ = Quaternion(
            [data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z]
        )
        q_ = q_ * self.q
        self.local_pose.pose.orientation.w = q_[0]
        self.local_pose.pose.orientation.x = q_[1]
        self.local_pose.pose.orientation.y = q_[2]
        self.local_pose.pose.orientation.z = q_[3]

    def publish_loop(self):
        # 保持与原脚本一致的60Hz输出
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.local_pose.header.stamp = rospy.Time.now()
            self.position_pub.publish(self.local_pose)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('odometry_offset_and_transfer_node')
    try:
        OdometryOffsetNode()
        T265ToMavrosNode()
        EgoTransferNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass