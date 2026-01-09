#!/usr/bin/env python3

import time
import traceback

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray

from common.rohand_can_serial_protocol_v1 import *


FRAME_ID_PREFIX = "rohand_"

PUB_RATE = 30  # Hz

ROH_HARDWARE_TYPE = 0x2001


class ROHandSerialNode(Node):

    def __init__(self):
        super().__init__("rohand_node")

        self.get_logger().info("node %s init.." % self.get_name())

        # Declare parameters with defaults (correct types)
        self.declare_parameter("port_name", 1)
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("hand_ids", [2, 3])
        self.declare_parameter("master_ids", [1, 4])

        # Read parameter values
        self.port_name_ = str(self.get_parameter("port_name").value)
        self.baudrate_ = self.get_parameter("baudrate").value
        self.hand_ids_ = self.get_parameter("hand_ids").value
        self.master_ids_ = self.get_parameter("master_ids").value
        self.get_logger().info("port: %s, baudrate: %d, hand_ids: %s, master_ids: %s" % (self.port_name_, self.baudrate_, str(self.hand_ids_), str(self.master_ids_)))

        self.position_ = []
        self.velocity_ = []
        self.effort_ = []
        self.data_time_ = time.time() - 1000  # Out of date
        self.status_ = []

        # 使用 PCAN (python-can) 作为传输，PcanTransport 在 protocol 文件中定义
        try:
            serial_port = PcanTransport(hand_id=self.hand_ids_, master_id=self.master_ids_, port_num=self.port_name_, baudrate=self.baudrate_)
            self.get_logger().info(f"Use CAN transport channel= Pcan{self.port_name_}")
        except Exception as exc:
            traceback.print_exc()
            self.get_logger().error(f"Can't create PcanTransport: {exc}")
            raise

        # 创建并初始化发布者成员属性pub_joint_states_
        self.joint_states_subscriber_ = self.create_subscription(
            msg_type=JointState, topic="~/target_joint_states", callback=self._joint_states_callback, qos_profile=10
        )

        # 创建并初始化发布者成员属性pub_joint_states_
        self.joint_states_publisher_ = self.create_publisher(msg_type=JointState, topic="~/current_joint_states", qos_profile=10)
        
        # 创建并初始化发布者成员属性pub_finger_states_
        self.finger_states_publihser_ = self.create_publisher(msg_type=UInt8MultiArray, topic="~/finger_state", qos_profile=10)

        # 初始化数据
        # self._init_joint_states()
        self.pub_rate = self.create_rate(PUB_RATE)

        # Initialize bus context
        rohand_ctx = OHandContext(serial=serial_port, timeout=200)
        self.rohand_protocol = OHandProtocol(rohand_ctx)

        for i in range(10):
            matched_cnt = 0

            for j in range(len(self.hand_ids_)):
                try:
                    err, remote_err, major, minor = self.rohand_protocol.get_protocol_version(self.hand_ids_[j], self.master_ids_[j])
                except Exception as exc:
                    traceback.print_exc()
                    self.get_logger().error(f"exception occurred when calling get_protocol_version(): {exc}")
                    continue

                if err != HAND_RESP_SUCCESS:
                    self.get_logger().error(f"rohand: {self.hand_ids_[j]} get_protocol_version() returned an error: {err}")
                    continue

                # if major == PROTOCOL_VERSION_MAJOR:
                #     matched_cnt += 1
                # else:
                #     self.get_logger().error(
                #         f"major protocol version of rohand '{self.hand_ids_[j]}' is '{major}', expected '{PROTOCOL_VERSION_MAJOR}'"
                #     )
                #     raise Exception("Protocol version NOT matched") 

            # if matched_cnt == len(self.hand_ids_):
            #     break

            time.sleep(1.0)

        # if matched_cnt != len(self.hand_ids_):
        #     raise Exception("Get protocol version failed")

        self.thread_ = threading.Thread(target=self._thread_pub)
        self.thread_.start()

    def _joint_states_callback(self, msg):
        self.get_logger().info("I heard: %s" % msg)

        try:
            hand_id = int(msg.header.frame_id.replace(FRAME_ID_PREFIX, ""))
            master_id = 1
            for i in range(len(self.hand_ids_)):
            	if hand_id == self.hand_ids_[i]:
                    hand_id = self.hand_ids_[i]
                    master_id = self.master_ids_[i]
            	
        except ValueError as e:
            hand_id = -1
        try:
            index = self.hand_ids_.index(hand_id)
        except ValueError as e:
            index = -1

        if index >= 0:
            # Send to OHand and read
            err, remote_err, position, angle, current, force, status = self.rohand_protocol.set_custom(
                hand_id, master_id, speed=msg.velocity, angle=msg.position, get_flag=SUB_CMD_GET_ANGLE | SUB_CMD_GET_STATUS | SUB_CMD_GET_FORCE
            )

                if err == HAND_RESP_SUCCESS:
                    self.position_ = angle if angle is not None else []
                    self.velocity_ = []  # TODO: Calculate speed according to position diff and time
                    self.effort_ = force if force is not None else []
                    self.data_time_ = time.time()
                    self.status_ = status if status is not None else []


    def _thread_pub(self):

        while rclpy.ok():
            for i in range(len(self.hand_ids_)):
                joint_states = JointState()
                finger_states = UInt8MultiArray()

                joint_states.header.stamp = self.get_clock().now().to_msg()
                joint_states.header.frame_id = str(self.hand_ids_[i])
                joint_states.name = ["thumb", "index", "middle", "ring", "little", "thumb_rotation"]

                if time.time() - self.data_time_ > 0.5 / PUB_RATE:
                    err, remote_err, position, angle, current, force, status = self.rohand_protocol.set_custom(
                        self.hand_ids_[i], self.master_ids_[i], get_flag=SUB_CMD_GET_ANGLE | SUB_CMD_GET_STATUS | SUB_CMD_GET_FORCE
                    )

                        if err != HAND_RESP_SUCCESS:
                            continue

                        joint_states.position = angle if angle is not None else []
                        joint_states.velocity = []  # TODO: Calculate speed according to position diff and time
                        joint_states.effort = force if force is not None else []
                        
                        finger_states.data = status if status is not None else []
                    
                else:
                    # Use still fresh data
                    joint_states.position = self.position_
                    joint_states.velocity = self.velocity_
                    joint_states.effort = self.effort_
                    finger_states.data = self.status_

                # 更新 header
                joint_states.header.stamp = self.get_clock().now().to_msg()

                # 发布关节数据
                self.joint_states_publisher_.publish(joint_states)
                
                # 发布手指状态数据
                self.finger_states_publihser_.publish(finger_states)

            self.pub_rate.sleep()


def main(args=None):
    rclpy.init(args=args)  # 初始化rclpy

    node = ROHandSerialNode()  # 新建一个节点

    rclpy.spin(node)  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    node.destroy_node()
    rclpy.shutdown()  # 关闭rclpy
