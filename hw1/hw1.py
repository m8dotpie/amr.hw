import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time 
import numpy as np 
import sympy as sp
import os

import matplotlib.pyplot as plt

class ControlStrategy(Node):
    def __init__(self, delta_t, r=0.04, L=0.08, duration=2):
        super().__init__('control_strategy')
        self.publisher_ = self.create_publisher(Twist, '/hagen/cmd_vel', 30)
        self.vel_sub = self.create_subscription(Twist, '/hagen/cmd_vel', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/hagen/odom", self.set_pose, 20)
        self.vel_sub
        self.odom_sub
        self.i = 0
        self.set_q_init = None
        self.q = None 
        self.r = r # Wheel radius
        self.L = L # Axle length
        self.D = 0.07 # Distance between the front front whell and rear axle
        self.duration = duration
        self.Ts = delta_t # Sampling time
        self.velocity = [0, 0]
        self.state = 0
        self.t = np.arange(0, self.duration, self.Ts) # Simulation time
        self.end_controller = False
        self.timer = self.create_timer(self.Ts, self.timer_callback)

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
    
    def diff_drive_init(self, duration=10):
        self.duration = duration
        self.wL = 12 # Left wheel velocity
        self.wR = 12 # Right wheel velocity
        self.time_utilized = 0.0
        
    def inter_point_diff_drive_init(self, duration=10, r_distance=1.3
                    , refPose=np.array([3,6,0]), k_p=0.5, k_w=0.7, dmin=0.7):
        
        self.duration = duration
        self.r_distance = r_distance
        self.refPose = refPose
        self.k_p = k_p 
        self.k_w = k_w
        self.dmin = dmin
        self.time_utilized = 0.0
        self.xT = self.refPose[0]  - self.r_distance*np.cos(self.refPose[2])
        self.yT = self.refPose[1]  - self.r_distance*np.sin(self.refPose[2])
        self.state = 0
        
    def inter_direction_diff_drive_init(self, duration=10, r_distance=1.3
                    , refPose=np.array([3,6,0]), k_p=0.5, k_w=0.7, dmin=0.7):
        self.duration = duration
        self.r_distance = r_distance
        self.refPose = refPose
        self.k_p = k_p 
        self.k_w = k_w
        self.dmin = dmin
        self.time_utilized = 0.0
    
    def inter_direction_diff_drive(self, ):
        if(self.q is not None):
            if(self.duration < self.time_utilized):
                self.send_vel(0.0, 0.0)
                self.end_controller = True

            self.D = np.sqrt((self.q[0]-self.refPose[0])**2 
                                    + (self.q[1]-self.refPose[1])**2)

            if(self.D < self.dmin):
                self.send_vel(0.0, 0.0)
                self.end_controller = True

            beta = np.arctan(self.r_distance/self.D)
            phiR = np.arctan2(self.refPose[1]-self.q[1], self.refPose[0]-self.q[0])
            alpha = self.wrap_to_pi(phiR-self.refPose[2])
            
            if(alpha <0):
                beta = -beta
            ##Controller
            if(np.abs(alpha) < np.abs(beta)):
                ePhi = self.wrap_to_pi(phiR - self.q[2] + alpha) 
            else:
                ePhi = self.wrap_to_pi(phiR - self.q[2] + beta) 
                
            v = self.k_p*self.D
            w = self.k_w*ePhi
            dq = np.array([v*np.cos(self.q[2]+self.Ts*w/2)
                            , v*np.sin(self.q[2]+self.Ts*w/2), w])
            self.q = self.q + self.Ts*dq # Integration
            self.q[2] = self.wrap_to_pi(self.q[2]) # Map orientation angle to [-pi, pi]
            self.send_vel(v, w)
            self.time_utilized  =  self.time_utilized + self.Ts    
        
    def inter_point_diff_drive(self, ):
        if(self.q is not None):
            if(self.duration < self.time_utilized):
                self.stop_vehicle()
                self.end_controller = True

            self.D = np.sqrt((self.q[0]-self.refPose[0])**2 + (self.q[1]-self.refPose[1])**2)

            if(self.D < self.dmin):
                self.stop_vehicle()
                self.end_controller = True

            if self.state == 0:
                d = np.sqrt((self.yT-self.q[1])**2 + (self.xT-self.q[0])**2)
                if(d < self.dmin):
                    self.state = 1
                self.phiT = np.arctan2(self.yT-self.q[1], self.xT-self.q[0])
                self.ePhi = self.phiT - self.q[2]
            else:
                self.ePhi = self.refPose[2] - self.q[2]
            
            v = self.k_p*self.D
            w = self.k_w*self.ePhi
            dq = np.array([v*np.cos(self.q[2]+self.Ts*w/2), v*np.sin(self.q[2]+self.Ts*w/2), w])
            self.q = self.q + self.Ts*dq # Integration
            self.q[2] = self.wrap_to_pi(self.q[2]) # Map orientation angle to [-pi, pi]
            self.send_vel(v, w)
            self.time_utilized  =  self.time_utilized + self.Ts 

    def perform_action_diff_drive_one_step(self):
        v = self.r/2*(self.wR+self.wL) # Robot velocity
        w = self.r/self.L*(self.wR-self.wL) # Robot angular velocity
        dq = np.array([v*np.cos(self.q[2]+self.Ts*w/2), v*np.sin(self.q[2]+self.Ts*w/2), w])
        self.q = self.q + self.Ts*dq # Integration
        self.q[2] = self.wrap_to_pi(self.q[2]) # Map orientation angle to [-pi, pi]
        self.send_vel(v, w)
        # rate.sleep()
        # time.sleep(self.Ts)
        self.time_utilized  =  self.time_utilized + self.Ts


    def timer_callback(self, ):
        # self.perform_action_diff_drive_one_step()
        # self.inter_direction_diff_drive()
        # self.inter_point_diff_drive()
        self.constant_velocity_one_step()
        return 

    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.q_dump.append(np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]))
        if(self.set_q_init is None):
            self.set_q_init = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
            self.q = self.set_q_init
            self.ref_q_dump.append(self.set_q_init)

    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher_.publish(msg)

    def constant_velocity_init(self, vel):
        self.q = [0, 0, 0]
        self.q_dump = []
        self.ref_q_dump = []
        self.time_utilized = 0
        if vel[2]:
            self.wL = vel[0]
            self.wR = vel[1]
            self.velocity[0] = self.r/2*(self.wR+self.wL) # Robot velocity
            self.velocity[1] = self.r/self.L*(self.wR-self.wL) # Robot angular velocity
        else:
            self.velocity = vel[0:2]

    def constant_velocity_one_step(self, ):
        if self.time_utilized > self.duration:
            self.stop_vehicle()
            self.end_controller = True
            return

        v = self.velocity[0]
        w = self.velocity[1]
        self.send_vel(v, w)

        dq = np.array([v*np.cos(self.q[2]+self.Ts*w/2), v*np.sin(self.q[2]+self.Ts*w/2), w])
        self.q = self.q + self.Ts*dq # Integration

        self.ref_q_dump.append(self.q)
        self.time_utilized  =  self.time_utilized + self.Ts

def sim_vehicle(args=None, delta_t=0.01, r=0.1, L=0.5, duration=10, velocity=[0.5, 0, False]):
    rclpy.init(args=args)
    control_strategy = ControlStrategy(delta_t=delta_t, r=r, L=L, duration=duration)
    control_strategy.constant_velocity_init(velocity)
    while control_strategy.end_controller is False and rclpy.ok():
        try:
            rclpy.spin_once(control_strategy)
        except KeyboardInterrupt:
            break
    dump = control_strategy.q_dump
    ref = control_strategy.ref_q_dump
    for _ in range(100):
        control_strategy.stop_vehicle()
    time.sleep(1)
    os.system('ros2 service call /reset_simulation std_srvs/srv/Empty "{}"')
    time.sleep(1)
    control_strategy.destroy_node()
    rclpy.shutdown()
    return dump, ref

def analytical_solution(velocity, delta_t, duration, r, L):
    v = velocity[0]
    w = velocity[1]
    q = np.array([0, 0, 0])

    if velocity[2]:
        v = r/2*(velocity[1]+velocity[0]) # Robot velocity
        w = r/L*(velocity[1]-velocity[0]) # Robot angular velocity
        
    t = sp.Symbol('t')
    dq = sp.Matrix([v*sp.cos(w*t), v*sp.sin(w*t), w*t])
    q = sp.integrate(dq, t)

    ts = np.arange(0, duration, delta_t)
    q = np.array([np.array(q.subs(t, t_).evalf()).astype(np.float64).reshape(1, 3).squeeze() for t_ in ts])
    return q
    

def main(args=None):
    vel_args = [[0.5, 0.0, 0],
                [1.0, 2.0, 0],
                [0.0, 2.0, 0],
                [20.0, 18.0, 1]]
    delta_t = 0.033
    r = 0.14
    L = 0.035
    duration = 5
    dumps = []
    ref = []
    analytical = []

    for velocity in vel_args:
        q_dump, ref_dump = sim_vehicle(args=args, delta_t=delta_t, r=r, L=L, duration=duration, velocity=velocity)
        dumps.append(q_dump)
        ref.append(ref_dump)
        analytical.append(analytical_solution(velocity, delta_t, duration, r, L))
        
    dumps = np.array(dumps)
    ref = np.array(ref)
    analytical = np.array(analytical)

    fig, ax = plt.subplots(2, 2)
    ax = [ax[0][0], ax[0][1], ax[1][0], ax[1][1]]

    for i in range(4):
        dump = np.array(dumps[i])
        ref_traj = np.array(ref[i])
        an = np.array(analytical[i])
        if i == 0:
            ax[i].plot(dump[:, 0], dump[:, 1], label='Real')
            ax[i].plot(ref_traj[:, 0], ref_traj[:, 1], label='Reference')
            ax[i].plot(an[:, 0], an[:, 1], label='Analytical')
        else:
            ax[i].plot(dump[:, 0], dump[:, 1])
            ax[i].plot(ref_traj[:, 0], ref_traj[:, 1])
            ax[i].plot(an[:, 0], an[:, 1])
        ax[i].set_xlabel('x (m)')
        ax[i].set_ylabel('y (m)')
    fig.legend()
    fig.tight_layout()
    plt.savefig('trajectories.png')
    
    ts = np.arange(0, duration, delta_t)

    fig = plt.figure()
    for i in range(4):
        ax = fig.add_subplot(2, 2, i + 1, projection='3d')
        dump = np.array(dumps[i])
        ref_traj = np.array(ref[i])
        an = np.array(analytical[i])
        ln = min(len(dump), len(ref_traj), len(an), len(ts))
        dump = dump[:ln]
        ref_traj = ref_traj[:ln]
        an = an[:ln]
        ts = ts[:ln]
        if i == 0:
            ax.plot3D(dump[:, 0], dump[:, 1], ts, label='Real')
            ax.plot3D(ref_traj[:, 0], ref_traj[:, 1], ts, label='Reference')
            ax.plot3D(an[:, 0], an[:, 1], ts, label='Analytical')
        else:
            ax.plot3D(dump[:, 0], dump[:, 1], ts)
            ax.plot3D(ref_traj[:, 0], ref_traj[:, 1], ts)
            ax.plot3D(an[:, 0], an[:, 1], ts)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_zlabel('t (s)')
    fig.legend()
    fig.tight_layout()
    plt.savefig('3dtrajectories.png')



if __name__ == '__main__':
    main()

# To reset ros2 service call /reset_simulation std_srvs/srv/Empty
