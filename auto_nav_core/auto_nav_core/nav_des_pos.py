import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix

from .my_ros_module import PubSubManager, NavNode


INITIAL_DESIRED_COORDINATES = [37.35228, -121.941788] # Garage
ROVER_FORMATION = [[5.0, 0.0, 0.0], [5.0, 120.0, 0.0], [5.0, 240.0, 0.0]] # [distance, angle, heading]

class NavController(NavNode):
    def __init__(self, node_name='nav_controller', target_lat_lon=None, n_rover=3, rover_formation=ROVER_FORMATION):
        super().__init__(node_name=node_name)
        self.n_rover = n_rover
        self.rover_formation = rover_formation
        self.cur_lat = []
        self.cur_lon = []
        self.cur_heading = []
        self.status_gps = [0]*self.n_rover
        if target_lat_lon is not None:
            self.des_lat, self.des_lon = target_lat_lon
        else:
            self.des_lat, self.des_lon = None, None
        self.status_des_gps = 0
        self.frame_des_heading = None

        for i in range(self.n_rover):
            self.cur_lat.append(None)
            self.cur_lon.append(None)
            self.cur_heading.append(None)
            self.pubsub.create_subscription(
                Float32MultiArray,
                f'/p{i+1}/imu/eulerAngle',
                lambda msg: self.euler_callback(msg, i),
                5)
            self.pubsub.create_subscription(
                NavSatFix,
                f'/p{i+1}/gps1',
                lambda msg: self.current_gps_callback(msg, i),
                5)
        self.pubsub.create_subscription(
            NavSatFix,
            '/nav/frame_target_gps',
            self.destination_gps_callback,
            5)
        self.pubsub.create_subscription(
            Float32MultiArray,
            '/nav/rover_formations',
            self.formation_callback,
            5)
        self.pubsub.create_subscription(
            Float32MultiArray,
            '/nav/rover_formation_params',
            self.formation_params_callback,
            5)
        for i in range(n_rover):
            self.pubsub.create_publisher(
                NavSatFix,
                f'/nav/p{i+1}/target_gps',
                5)
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.nav_callback)

    def current_gps_callback(self, msg_cur_pos:NavSatFix, i):
        self.status_gps[i] = msg_cur_pos.status.status
        print("[Current] Status GPS:", self.status_gps , "Lat/Lon:", msg_cur_pos.latitude,msg_cur_pos.longitude)
        if self.status_gps[i] != 0:
            self.cur_lat[i] = msg_cur_pos.latitude
            self.cur_lon[i] = msg_cur_pos.longitude

    def destination_gps_callback(self, msg_des_pos:NavSatFix):
        self.status_des_gps = msg_des_pos.status.status
        print("[Destination] Status GPS:", self.status_gps , "Lat/Lon:", msg_des_pos.latitude,msg_des_pos.longitude)
        if self.status_des_gps != 0:
            self.des_lat = msg_des_pos.latitude
            self.des_lon = msg_des_pos.longitude
            
    def euler_callback(self, msg_imu_euler:Float32MultiArray, i):
        self.cur_heading[i] =  msg_imu_euler.data[0]
    
    def formation_callback(self, msg_formation:Float32MultiArray):
        if len(msg_formation.data) != self.n_rover*3:
            print("Invalid formation data : Number of rovers does not match. Current number of rovers: ", self.n_rover)
            return
        for i in range(self.n_rover):
            self.rover_formation[i] = msg_formation.data[i*3:i*3+3]
    
    def formation_params_callback(self, msg_formation_params:Float32MultiArray):
        distance, heading_offset = msg_formation_params.data
        for i in range(self.n_rover):
            self.rover_formation[i] = [distance, (i*360/self.n_rover+heading_offset)%360, 0.0]
    
    def nav_callback(self):
        print(f"Frame destination: [{self.des_lat}, {self.des_lon}, {self.frame_des_heading}, rover formation: {self.rover_formation}")
        msg_rover_pos = NavSatFix()
        for i in range(self.n_rover):
            msg_rover_pos.latitude, msg_rover_pos.longitude = self.get_target_pos(self.des_lat, self.des_lon, self.rover_formation[i][0], self.rover_formation[i][1])
            self.pubsub.publish(f'/nav/p{i+1}/target_gps', NavSatFix())
            print(f"Rover {i+1} target position: [{msg_rover_pos.latitude}, {msg_rover_pos.longitude}]")


def main(args=None):
    rclpy.init(args=args)
    sub_move_cmds = NavController(target_lat_lon=INITIAL_DESIRED_COORDINATES)
    rclpy.spin(sub_move_cmds)
    sub_move_cmds.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()