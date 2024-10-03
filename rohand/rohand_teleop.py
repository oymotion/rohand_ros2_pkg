#!/usr/bin/env python3

import threading
import time
from pynput import keyboard

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


FRAME_ID_PREFIX = 'rohand_'


class ROHandTeleopNode(Node):

    def __init__(self):
        super().__init__('rohand_teleop_node')
        self.bus_mutex = threading.Lock()

        self.get_logger().info("node %s init.." % self.get_name())

        self.declare_parameters(
            namespace='',
            parameters=[
                ('hand_id', rclpy.Parameter.Type.INTEGER)
            ]
        )

        self.hand_id_ = self.get_parameter_or('~hand_id', 2)
        self.get_logger().info("hand_id: %d" % (self.hand_id_))

        # 创建并初始化发布者成员属性pub_joint_states_
        self.joint_states_publisher_ = self.create_publisher(msg_type=JointState, topic="~/target_joint_states", qos_profile=10)

        # 初始化数据
        self.angles_ = [36.0, 174.0, 174.0, 174.0, 178.0, 0.0]
        self.pub_rate = self.create_rate(30)

        self.pub_thread_ = threading.Thread(target=self._thread_pub)
        self.pub_thread_.start()

        self.listener_ = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release)
        self.listener_.start()


    def _on_press(key):
        try:
            self.get_logger().info('alphanumeric key {0} pressed'.format(key.char))
            match argument:
                case 'q':
                    self.angles_[0] += 1.0

                case 'a':
                    self.angles_[0] -= 1.0

                case 'w':
                    self.angles_[1] += 1.0

                case 's':
                    self.angles_[1] -= 1.0

                case 'e':
                    self.angles_[2] += 1.0

                case 'd':
                    self.angles_[2] -= 1.0

                case 'r':
                    self.angles_[3] += 1.0

                case 'f':
                    self.angles_[3] -= 1.0

                case 't':
                    self.angles_[4] += 1.0

                case 'g':
                    self.angles_[4] -= 1.0

                case 'y':
                    self.angles_[5] += 1.0

                case 'h':
                    self.angles_[5] -= 1.0

        except AttributeError:
            self.get_logger().info('special key {0} pressed'.format(key))


    def _on_release(key):
        self.get_logger().info('{0} released'.format(key))
        if key == keyboard.Key.esc:
            # Stop listener
            return False

    def _thread_pub(self):
        last_update_time = time.time()

        while rclpy.ok():
            delta_time = time.time() - last_update_time
            last_update_time = time.time()

            joint_states = JointState()

            joint_states.header.stamp = self.get_clock().now().to_msg()
            joint_states.header.frame_id = FRAME_ID_PREFIX + str(self.hand_id_)
            joint_states.name = ['thumb', 'index', 'middle', 'ring', 'little', 'thumb_rotation']

            # Current position
            joint_states.position = self.angles_ 

            # TODO：当前速度
            joint_states.velocity = []

            # TODO: Current force
            joint_states.effort = []


            # 更新 header
            joint_states.header.stamp = self.get_clock().now().to_msg()

            # 发布关节数据
            self.joint_states_publisher_.publish(joint_states)

            self.pub_rate.sleep()


def main(args=None):
    rclpy.init(args=args)  # 初始化rclpy

    node = ROHandTeleopNode()  # 新建一个节点

    rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    node.destroy_node()
    rclpy.shutdown()  # 关闭rclpy

