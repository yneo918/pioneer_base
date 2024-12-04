import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int16
from std_msgs.msg import String

from .my_ros_module import PubSubManager


class Demux(Node):
    def __init__(self, n_rover=6):
        super().__init__('demux')
        self.N_ROVER = n_rover
        self.select = 0

        self.broadcast = False

        self.pubsub = PubSubManager(self)
        self.pubsub.create_subscription(
            Twist,
            '/joy/cmd_vel',
            self.joy_cmd_callback,
            5)
        self.pubsub.create_subscription(
            Bool,
            '/joy/enable',
            self.joy_en_callback,
            5)
        self.pubsub.create_subscription(
            Int16,
            '/select_rover',
            self.sel_callback,
            1)
        self.pubsub.create_subscription(
            Bool,
            '/joy/broadcast',
            self.broadcast_callback,
            1)
        for i in range(self.N_ROVER):
            self.pubsub.create_publisher(Twist, f'/p{i+1}/cmd_vel', 5)
            self.pubsub.create_publisher(Bool, f'/p{i+1}/enable', 5)
        
    
    def joy_cmd_callback(self, msg):
        self.lx = msg.linear.x
        self.az = msg.angular.z

    def broadcast_callback(self, msg):
        self.broadcast = msg.data
    
    def sel_callback(self, msg):
        self.select = msg.data
        
    def joy_en_callback(self,msg):
        _en = msg.data
        val = Twist()           # For selected rover
        en_state = Bool()       # For selected rover
        empty_twist = Twist()   # For other rovers
        false_state = Bool()    # For other rovers
        empty_twist.linear.x = 0.0
        empty_twist.angular.z = 0.0
        false_state.data = False

        for i in range(self.N_ROVER):
            if self.broadcast:
                en_state.data = True
                val.linear.x = self.lx
                val.angular.z = self.az
                self.pubsub.publish(f'/p{i+1}/cmd_vel', val)
                self.pubsub.publish(f'/p{i+1}/enable', en_state)
            elif _en and self.select == i+1:
                en_state.data = True
                val.linear.x = self.lx
                val.angular.z = self.az
                self.pubsub.publish(f'/p{i+1}/cmd_vel', val)
                self.pubsub.publish(f'/p{i+1}/enable', en_state)
            else:
                self.pubsub.publish(f'/p{i+1}/cmd_vel', empty_twist)
                self.pubsub.publish(f'/p{i+1}/enable', false_state)         


def main(args=None):
    rclpy.init(args=args)
    demultiplexer = Demux()
    rclpy.spin(demultiplexer)
    demultiplexer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#ros2 run joy joy_node
