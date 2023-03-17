import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
import time  
import cv2
import json
import numpy as np
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
import std_msgs.msg as std_msg
import rclpy.qos as qos
import scipy.interpolate as si

class StanleyController:
    def __init__(self, k=0.3, kp=1, kv=1, L=2.5, tv=0.3):
        self.k = k
        self.kp = kp
        self.kv = kv 
        self.L = L
        self.tv = tv
        
    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
    
    def step(self, cv, pos, ref):

        x, y, theta = pos
        t_x, t_y = ref[0:2]
        t_theta = ref[2]
        fx = x + self.L * np.cos(theta)
        fy = y + self.L * np.sin(theta)

        dx = fx - t_x
        dy = fy - t_y
        d = np.hypot(dx, dy)

        front_vector = np.array([np.sin(theta), -np.cos(theta)])
        nearest_vector = np.array([dx, dy])
        e = np.sign(nearest_vector @ front_vector) * d
        e_delta = np.arctan2((self.k * e), (self.kp * self.tv))

        e_phi = self.normalize_angle(t_theta - theta)
        delta = e_phi + e_delta

        delta_min = - np.pi / 6
        delta_max = np.pi / 6
        if delta > delta_max:
            delta = delta_max
        elif delta < delta_min:
            delta = delta_min

        a = self.kv * (self.tv - cv)
        return delta, a

class ControlStrategy(Node):
    def __init__(self, delta_t, ):
        super().__init__('control_strategy')
        self.publisher_ = self.create_publisher(Twist, '/hagen/cmd_vel', 30)
        self.vel_sub = self.create_subscription(Twist, '/hagen/cmd_vel', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/hagen/odom", self.set_pose, 20)
        self.path_publisher = self.create_publisher(Path, '/hagen_path', 30)
        self.car_path_publisher = self.create_publisher(Path, '/hagen_car_path', 30)
        self.img_sub = self.create_subscription(Image
            , '/depth_camera/image_raw', self.img_callback, 3)
        
        self.img_info_sub = self.create_subscription(CameraInfo
            , '/depth_camera/camera_info', self.camera_info_callback, 3)
        
        self.scan = self.create_subscription(LaserScan, '/hagen/scan', self.scan_callback, 3)
        
        self.i = 0
        self.set_q_init = None
        self.q = None 
        self.qs = []
        self.rqs =  []
        self.r = 0.3 # Wheel radius
        self.L = 2.5 # Axle length
        self.D = 0.07 # Distance between the front front whell and rear axle
        self.Ts = delta_t # Sampling time
        self.t = np.arange(0, 10, self.Ts) # Simulation time
        self.end_controller = False
        self.timer = self.create_timer(self.Ts, self.timer_callback)
        self.proj_init = False
        self.P = np.zeros((3, 4))
        self.scan_data = []
        self.laser_range_max = 0
        self.laser_range_min = 0
        self.stanley_controller = StanleyController()

    def img_callback(self, m : Image):
        np_img = np.reshape(m.data, (m.height, m.width, 3)).astype(np.uint8)

        for point in self.scan_data:
            x, y, z = point
            proj_point = self.P @ np.array([-y, 0, x, 1])
            proj_point = 1 / proj_point[2] * proj_point

            norm = 255 / (self.laser_range_max - self.laser_range_min)
            C = 40
            color = [255 - norm * z - C, 0, norm * z + C]
            x, y = int(proj_point[0]), int(proj_point[1])
            if x < 0 or x >= 640 or y < 0 or y >= 480:
                continue

            for i in range(-10, 10):
                for j in range(-10, 10):
                    y_i = y + i
                    x_j = x + j
                    if x_j < 0 or x_j >= 640 or y_i < 0 or y_i >= 480:
                        continue
                    np_img[y + i, x + j] = color

        
        self.display(np_img)
        
    def camera_info_callback(self, m : CameraInfo):
        if not self.proj_init:
            self.proj_init = True
            self.P = np.reshape(m.p, (3, 4))
            print(self.P)
 
    def display(self, img : np.ndarray):
        cv2.imshow("camera view", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)
        pass
        
    def scan_callback(self, m : LaserScan):
        self.scan_data.clear()
        self.laser_range_max = m.range_max
        self.laser_range_min = m.range_min
        depths = np.array(m.ranges)
        
        angles = np.arange(m.angle_min, m.angle_max, m.angle_increment)
        for angle, R in zip(angles, depths):
            if R != np.inf:
                self.scan_data.append(np.array([R * np.cos(angle), R * np.sin(angle), R]))

              
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def listener_callback(self, msg):
        pass

    def stop_vehicle(self, ):
        self.send_vel(0.0, 0.0)    
    
    def wrap_to_pi(self, x):
        x = np.array([x])
        xwrap = np.remainder(x, 2*np.pi)
        mask = np.abs(xwrap)>np.pi
        xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
        return xwrap[0]
    
    def timer_callback(self, ):

        if self.time_utilized > self.duration:
            self.stop_vehicle()
            self.end_controller = True
            print('Finished due timeout')
            return
        self.time_utilized += self.Ts

        msg = Path()
        msg.header.frame_id = "odom"
        for i in range(len(self.control_points)):
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.pose.position.x = self.control_points[i, 0]
            pose.pose.position.y = self.control_points[i, 1]
            msg.poses.append(pose)

        self.path_publisher.publish(msg)

        msg = Path()
        msg.header.frame_id = "odom"
        for i in range(len(self.qs)):
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.pose.position.x = self.qs[i][0]
            pose.pose.position.y = self.qs[i][1]
            msg.poses.append(pose)

        self.car_path_publisher.publish(msg)

        self.reference_path_follower_one_step()

        return 
    
    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        if(self.set_q_init is None):
            self.set_q_init = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw + np.pi / 2]
            self.q = np.array(self.set_q_init)
            self.rqs.append(self.set_q_init)
            self.qs.append(self.set_q_init)
        else:
            self.rqs.append([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
            self.qs.append(list(self.q))

    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher_.publish(msg)

        dq = np.array([self.v * np.cos(self.q[2]), self.v * np.sin(self.q[2]), self.v * np.tan(self.delta) / self.L]) * self.Ts
        self.q += dq
        self.q[2] = self.wrap_to_pi(np.fmod(self.q[2], 2 * np.pi))

        self.v = v
        self.delta = w

    def reference_path_follower(self, duration, max_v, control_points, k_p, k_theta, ref_theta):
        self.time_utilized = 0
        self.duration = duration
        self.control_points = control_points
        self.k_p = k_p
        self.k_theta = k_theta
        self.current_index = -1
        self.dmin = 1
        self.prev_index = -1
        self.k_r = 0.5
        self.v = 0.0
        self.delta = 0.0
        self.ref_theta = ref_theta
        self.max_v = max_v

        self.stanley_controller.tv = self.max_v

    def reference_path_follower_one_step(self):
        # Check whether we are initialized in space
        if self.set_q_init is None or self.end_controller == True:
            return

        if self.current_index == -1:
            for i in range(len(self.control_points - 1)):
                d1 = np.dot(self.q[0:2] - self.control_points[i], self.control_points[self.current_index+1] - self.control_points[i])
                d2 = np.dot(self.control_points[i+1] - self.control_points[i], self.control_points[i+1] - self.control_points[i])
                if d1 / d2 < 1:
                    self.current_index = i
                    break
        if self.current_index != len(self.control_points) - 2:
            d1 = np.dot(self.q[0:2] - self.control_points[self.current_index], 
                        self.control_points[self.current_index+1] - self.control_points[self.current_index])
            d2 = np.dot(self.control_points[self.current_index+1] - self.control_points[self.current_index],
                        self.control_points[self.current_index+1] - self.control_points[self.current_index])
            if np.abs(d1) / np.abs(d2) > 1:
                self.current_index += 1

        elif self.current_index == len(self.control_points) - 2:
            d1 = np.dot(self.q[0:2] - self.control_points[self.current_index], 
                        self.control_points[self.current_index+1] - self.control_points[self.current_index])
            d2 = np.dot(self.control_points[self.current_index+1] - self.control_points[self.current_index],
                        self.control_points[self.current_index+1] - self.control_points[self.current_index])
            if np.abs(d1) / np.abs(d2) > 1:
                for i in range(100):
                    self.stop_vehicle()
                print("Reach to the goal pose")
                self.end_controller = True
                return

        else:
            d = np.sqrt((self.control_points[self.current_index+1][1]-self.q[1])**2 + (self.control_points[self.current_index+1][0]-self.q[0])**2)
            if d < self.dmin:
                for i in range(100):
                    self.stop_vehicle()
                print("Reach to the goal pose")
                self.end_controller = True
                return

        if self.current_index != self.prev_index:
            print('Following now: ', self.current_index, 'th segment')
            self.prev_index = self.current_index

        pos = self.q - np.array([np.cos(self.q[2]), np.sin(self.q[2]), 0]) * 1.1
        ref = np.array([self.control_points[self.current_index + 1][0], self.control_points[self.current_index + 1][1], self.ref_theta[self.current_index + 1]])
        theta, v = 0.0, 0.0
        
        theta, v = self.stanley_controller.step(self.v, pos, ref)
        
        self.send_vel(v, theta)

def main(args=None):
    rclpy.init(args=args)
    control_strategy = ControlStrategy(delta_t=0.03)
    x = np.linspace(0, 10, 10)
    y = 5 * np.sqrt(x)
    p = np.array([x, y]).T
    ref_path_x, ref_path_y = x, y
    ref_path_theta = [np.arctan2(ref_path_y[i + 1] - ref_path_y[i], ref_path_x[i + 1] - ref_path_x[i]) for i in range(0, len(ref_path_y) - 1, 1)]
    ref_path_theta.append(ref_path_theta[-1])
    p = np.array([ref_path_x, ref_path_y]).T

    data = {'cp': p, 'max_v': 1, 'ktheta': 10, 'ref_theta': ref_path_theta, 'kp': 5}

    control_strategy.reference_path_follower(50, data['max_v'], control_points=np.array(data['cp'], dtype=np.float64), 
                                                        k_p=data['kp'], k_theta=data['ktheta'], ref_theta=ref_path_theta)

    while rclpy.ok():
        try:
            rclpy.spin_once(control_strategy)
        except KeyboardInterrupt:
            break
    control_strategy.destroy_node()
    data['cp'] = data['cp'].astype(float).tolist()
    data['qs'] = control_strategy.qs
    data['rqs'] = control_strategy.rqs
    data['ref_x'] = ref_path_x.astype(float).tolist()
    data['ref_y'] = ref_path_y.astype(float).tolist()
    data['ref_theta'] = ref_path_theta
    json.dump(data, open(f'./logs/data{data["max_v"]}-{data["controller"]}.json', 'w'))
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# To reset ros2 service call /reset_simulation std_srvs/srv/Empty
