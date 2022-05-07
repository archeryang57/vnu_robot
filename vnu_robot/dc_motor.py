#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .DFRobot_DC_Motor import DFRobot_DC_Motor_IIC as Board

class MotorNode(Node):                 
    def __init__(self):
        super().__init__("dc_motor") 
        self.get_logger().info("Hello DC Motor")
        self.init_board()

        # * 建立 Subscription
        self.create_subscription(Twist, "/cmd_vel", self.callback_func, 10)

    def init_board(self):
        self.board = Board(1, 0x10)    # Select bus 1, set address to 0x10
        self.board_detect()
        while self.board.begin() != self.board.STA_OK:    # Board begin and check board status
            self.print_board_status()
            print("board begin faild")
            time.sleep(2)

        self.board.set_encoder_enable(self.board.ALL)                 # Set selected DC motor encoder enable
        # self.board.set_encoder_disable(self.board.ALL)              # Set selected DC motor encoder disable
        self.board.set_encoder_reduction_ratio(self.board.ALL, 43)    # Set selected DC motor encoder reduction ratio, test motor reduction ratio is 43.8

        self.board.set_moter_pwm_frequency(1000)   # Set DC motor pwm frequency to 1000HZ
       
        print("board begin success")

    # * 建立 call back function, 撰寫執行動作, for subscription
    def callback_func(self, twist:Twist):
 
        # tb3 maxForwardForce = 0.21
        # tb3 maxTurnForce = 2.6
        forwardRate = 400  # 0.2 * 500 = 100 80:let turn works
        turnRate = 40      # 2.5 * 40 = 100
        m1_forward = twist.linear.x * forwardRate
        m2_forward = m1_forward
        max_frd = 0.2 * forwardRate
        
        turn = twist.angular.z * turnRate
        if turn > 0:  # right turn
            m1_forward = min(max_frd, m1_forward + turn )
            m2_forward = m2_forward - turn / 2
        elif turn < 0:
            m1_forward = m1_forward + turn / 2
            m2_forward = min(max_frd, m2_forward - turn )
        m1_dir = self.board.CW
        m2_dir = self.board.CCW

        if m1_forward < 0 :
            m1_dir = self.board.CCW

        if m2_forward < 0 :
            m2_dir = self.board.CW

        self.board.motor_movement(
            [self.board.M1], 
            m1_dir, 
            abs(m1_forward))    # DC motor 1 movement, orientation clockwise
        self.board.motor_movement(
            [self.board.M2], 
            m2_dir, 
            abs(m2_forward))   # DC motor 2 movement, orientation count-clockwise

        print(f"m1:{m1_forward}  m2:{m2_forward}")

    def board_detect(self):
        l = self.board.detecte()
        print("Board list conform:")
        print(l)

    ''' print last operate status, users can use this variable to determine the result of a function call. '''
    def print_board_status(self):
        if self.board.last_operate_status == self.board.STA_OK:
            print("board status: everything ok")
        elif self.board.last_operate_status == self.board.STA_ERR:
            print("board status: unexpected error")
        elif self.board.last_operate_status == self.board.STA_ERR_DEVICE_NOT_DETECTED:
            print("board status: device not detected")
        elif self.board.last_operate_status == self.board.STA_ERR_PARAMETER:
            print("board status: parameter error, last operate no effective")
        elif self.board.last_operate_status == self.board.STA_ERR_SOFT_VERSION:
            print("board status: unsupport board framware version")


def main(args=None):
    #  初始化 ROS
    rclpy.init(args=args)
    # * 建立 Node
    node = MotorNode()               

    #  Spin Node 以持續運行Node工作(執行 callback function)
    rclpy.spin(node)
    #  Shutdown ROS
    node.destroy_node()  # optional, it will be destoried when GC.
    rclpy.shutdown()

if __name__=="__main__":
    main()