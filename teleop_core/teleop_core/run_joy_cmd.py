import rclpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int16
from std_msgs.msg import String

from .my_ros_module import JoyBase


class JoyCmd(JoyBase):
    def __init__(self, n_rover=6):
        super().__init__('multi_teleop')

        self.lx_axisN = self.axis_dict.get("LY")
        self.az_axisN = self.axis_dict.get("RX")
        self.en_buttonN = self.button_dict.get("LB")
        self.rover_sel_button = self.button_dict.get("Y")
        self.mode_sel_button = "RB"

        self.mode_list = ["NEU_M", "JOY_M", "NAV_M"]
        self.mode_dict = {"NEU_M": 0, "JOY_M": 1, "NAV_M": 2}
        self.rover_modeC = self.mode_list[0]

        self.N_ROVER = n_rover
        self.select = 1

        self.pubsub.create_publisher(Twist, '/joy/cmd_vel', 5)
        self.pubsub.create_publisher(Bool, '/joy/enable', 5)
        
        self.pubsub.create_publisher(Int16, '/select_rover', 1)
        self.pubsub.create_publisher(String, '/modeC', 1)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def joy_callback(self, msg):
        _toggle = self.joy_toggle(msg)

        j_lx = msg.axes[self.lx_axisN]
        j_az = msg.axes[self.az_axisN]

        val = Twist()           # For selected rover
        en_state = Bool()       # For selected rover
        empty_twist = Twist()   # For other rovers
        false_state = Bool()    # For other rovers
        
        empty_twist.linear.x = 0.0
        empty_twist.angular.z = 0.0
        false_state.data = False

        # Toggle select rover
        if _toggle[self.rover_sel_button] == 1:
            self.select = self.select%self.N_ROVER + 1

        if _toggle[self.button_dict[self.mode_sel_button]] == 1:
            print(f"Mode Button Toggled: {self.mode[self.mode_list[self.mode]]} to {self.mode[self.mode_list[(self.mode + 1) % 3]]}")
            self.rover_modeC = self.mode[self.mode_list[(self.mode + 1) % 3]]

        if msg.buttons[self.en_buttonN] == 1:
            en_state.data = True
            val.linear.x = 0.7*j_lx
            val.angular.z = 0.5*j_az
            self.pubsub.publish('/joy/cmd_vel', val)
            self.pubsub.publish('/joy/enable', en_state)
        else:
            self.pubsub.publish('/joy/cmd_vel', empty_twist)
            self.pubsub.publish('/joy/enable', false_state)

    def timer_callback(self):
        select_msg = Int16()
        select_msg.data = self.select
        self.pubsub.publish('/select_rover', select_msg)

        mode_msg = String()
        mode_msg.data = self.rover_modeC
        self.pubsub.publish('/modeC', mode_msg) 

    def robot_mode_callback(self):
        msg = String()
        msg.data = self.rover_modeC
        self.pubsub.publish('/modeC', msg)            


def main(args=None):
    rclpy.init(args=args)
    joy_handle = JoyCmd()
    rclpy.spin(joy_handle)
    joy_handle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#ros2 run joy joy_node
