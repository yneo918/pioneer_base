import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import String

from .my_ros_module import JoyBase


class GetMoveCmds(JoyBase):

    def __init__(self, n_rover=3):
        super().__init__('rover_state_controler')

        self.N_ROVER = n_rover
        self.toggle_button_mode = "RB"
        self.mode_list = ["NEU_M", "JOY_M", "NAV_M"]
        self.mode_dict = {"NEU_M": 0, "JOY_M": 1, "NAV_M": 2}
        self.rover_modeC = self.mode_list[0]

        self.joy_cmd = [0.0, 0.0]

        self.pubsub.create_subscription(
            Twist,
            'cmd_vel',
            self.joy_cmd_callback,
            5)

        self.nav = []
        self.u = []
        for i in range(self.N_ROVER):
            self.nav.append([0.0, 0.0])
            self.u.append([0.0, 0.0])
            self.pubsub.create_subscription(
                Twist,
                f'/p{i+1}/nav_cmd_vel',
                lambda msg: self.nav_cmd_callback(msg, i),
                5)
            self.pubsub.create_publisher(
                Twist, 
                f'/p{i+1}/cmd_vel', 
                5)

        timer_period_core = 0.05  # seconds
        self.timer = self.create_timer(timer_period_core, self.core_cmd_vel_callback)

        # self.pub_rover_en = self.create_publisher(Bool, 'r4/enable', 1)
        # timer_period = 0.2  # seconds
        # self.timer = self.create_timer(timer_period, self.rover_en_callback)
        # self.i = 0

        self.pubsub.create_publisher(String, '/modeC', 1)
        timer_period_mode = 0.1  # seconds
        self.timer = self.create_timer(timer_period_mode, self.robot_mode_callback)
        
    def joy_callback(self, msg):
        _toggle = self.joy_toggle(msg)
        if _toggle[self.button_dict[self.toggle_button_mode]] == 1:
            print(f"Mode Button Toggled: {self.mode[self.mode_list[self.mode]]} to {self.mode[self.mode_list[(self.mode + 1) % 3]]}")
            self.rover_modeC = self.mode[self.mode_list[(self.mode + 1) % 3]]
        
    def joy_cmd_callback(self, msg):
        self.joy_cmd = [msg.linear.x, msg.angular.z]
    
    def nav_cmd_callback(self, msg, i):
        self.nav[i] = [msg.linear.x, msg.angular.z]
    
    def core_cmd_vel_callback(self):
        msg = Twist()

        if (self.rover_modeC == "JOY_M"):
            for i in range(self.N_ROVER):
                self.u[i] = self.joy_cmd
        elif (self.rover_modeC == "NAV_M"):
            for i in range(self.N_ROVER):
                self.u[i] = self.nav[i]
        elif (self.rover_modeC == "NEU_M"):
            for i in range(self.N_ROVER):
                self.u[i] = [0.0, 0.0]
        
        for i in range(self.N_ROVER):
            msg.linear.x = self.u[i][0]
            msg.angular.z = self.u[i][1]
            self.pubsub.publish(f'/p{i+1}/cmd_vel', msg)

    def robot_mode_callback(self):
        msg = String()
        msg.data = self.rover_modeC
        self.pubsub.publish('/modeC', msg)


def main(args=None):
    rclpy.init(args=args)
    sub_move_cmds = GetMoveCmds()
    rclpy.spin(sub_move_cmds)
    sub_move_cmds.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()