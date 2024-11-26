import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class JoyBase(Node):
    def __init__(self, node_name='joy_node'):
        super().__init__(node_name)
        self.button_dict = { 
            "A": 0,
            "B": 1,
            "X": 2,
            "Y": 3,
            "LB": 4,
            "RB": 5,
            "BACK": 6,
            "START": 7,
            "POWER": 8,
            "LS": 9,
            "RS": 10
        }

        self.axis_dict = {
            "LX": 0,
            "LY": 1,
            "LT": 2,
            "RX": 3,
            "RY": 4,
            "RT": 5,
            "cross_lr": 6,
            "cross_ud": 7
        }
        self.pubsub = PubSubManager(self)

        self.pubsub.create_subscription(Joy, 'joy', self.joy_callback)

        self.prev_joy = Joy()
        #self.lx_axisN = self.axis_dict.get("LY")
    
    def joy_callback(self, msg):
        _toggle = self.joy_toggle(msg)
    
    def joy_toggle(self, joy_msg):
        if len(self.prev_joy.buttons) != len(joy_msg.buttons):
            self.prev_joy = joy_msg

        ret = [0] * len(joy_msg.buttons)
        for i in range(len(joy_msg.buttons)):
            ret[i] = self.prev_joy.buttons[i] - joy_msg.buttons[i]
        self.prev_joy = joy_msg
        return ret


class PubSubManager:
    def __init__(self, node=None):
        self.node = node
        self._subscribers = [[], {}]  # [list of subscriber objects, dict of topic_name]
        self._publishers = [[], {}]  # [list of subscriber objects, dict of topic_name]
        
    def create_subscription(self, msg_type, topic_name, callback, qos=10, **kwargs):
        self._subscribers[0].append(
            self.node.create_subscription(
                msg_type,
                topic_name,
                callback,
                qos,
                **kwargs
            )
        )
        self._subscribers[1][topic_name] = len(self._subscribers[0]) - 1

    def create_publisher(self, msg_type, topic_name, qos=10, **kwargs):
        self._publishers[0].append(
            self.node.create_publisher(
                msg_type,
                topic_name,
                qos,
                **kwargs
            )
        )
        self._publishers[1][topic_name] = len(self._publishers[0]) - 1
    
    def publish(self, topic_name, msg):
        if topic_name in self._publishers[1]:
            self._publishers[0][self._publishers[1][topic_name]].publish(msg)
        else:
            print(f"Publisher for {topic_name} not found.")


def main(args=None):
    rclpy.init(args=args)
    joy_handle = JoyBase()
    rclpy.spin(joy_handle)
    joy_handle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
