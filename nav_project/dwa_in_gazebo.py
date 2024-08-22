#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
import numpy as np
import math
import random
import os
import time

robot_radius = 0.2
max_speed = 0.22
min_speed = -0.22
max_omega = 300.72 * math.pi / 180.0 # 최대 각속도
max_a = 100.0
max_delta_omega = 600.72 * math.pi / 180.0 # 최대 각가속도
dt = 0.2

future_time = 4.0

# 가중치
alpha = 2.0 # 위치 가중치
beta = 0.01 # 충돌 가중치
gamma = 0.01 # 속도 가중치


def distance(a, b):
    return math.sqrt((a[0] - b[0])**2+(a[1] - b[1])**2) 

    
def generate_velocity_range(u, dt): # u = [v,w]
    return np.array([u[0]-max_a*dt, u[0]+max_a*dt, u[1]-max_delta_omega*dt ,u[1]+max_delta_omega*dt])


def velocity_sampling(dw): 
    i = 0
    j = 0
    sampling_number = 10
    sample_list = []
    while i <= sampling_number:
        while j <=sampling_number:
            sample_list.append((dw[0]+(dw[1]-dw[0])*i/sampling_number,dw[2]+(dw[3]-dw[2])*j/sampling_number))
            j+=1
        i+=1
        j=0
        
    return sample_list

class DWANode(Node):
    def __init__(self):
        super().__init__('dwa_node')
                # 파라미터 선언 및 기본값 설정
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')


        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting again...')
        while not self.delete_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Delete service not available, waiting again...')
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        # self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.goal_subscriber = self.create_subscription(PoseStamped, 'goal', self.goal_callback, 10)
        self.subscription = self.create_subscription(LaserScan,'scan',self.laser_callback,10)

# Best Effort QoS 설정
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,depth=10)
        self.time_subscriber = self.create_subscription(Clock,'clock',self.clock_callback,qos_profile)
        self.sim_time = None
        self.robot_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # [x, y, v, angle, omega]
        self.goal = np.array([0.0, -0.5])
        self.obstacles = []
        self.obstacle_radius = 0.15
        self.dt = dt
        self.spawn_request()
        self.previous_time = self.get_clock().now()
        self.timer = self.create_timer(0.01, self.timer_callback)

    def laser_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # 유효한 거리 필터링
        valid_indices = np.isfinite(ranges)
        close_indices = np.where(ranges[valid_indices] < 1.0)[0]
        ranges = ranges[valid_indices][close_indices]
        angles = angles[valid_indices][close_indices]
        #print(ranges)
        # (x, y) 좌표로 변환
        #if ranges <= 1.2:
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        #print(x)
        points = np.vstack((x, y)).T

        # 클러스터링 수행
        self.cluster_points(points)
        
        
    def cluster_points(self, points): # 클러스터링 원리 알기.
        self.obstacles = []
        db = DBSCAN(eps=0.2, min_samples=5).fit(points)
        labels = db.labels_

        # 노이즈 포인트는 -1로 라벨링됨
        unique_labels = set(labels)
        clusters = [points[labels == label] for label in unique_labels if label != -1]

        #self.get_logger().info(f'Number of clusters found: {len(clusters)}')

        # 각 클러스터의 중심 좌표 출력
        for i, cluster in enumerate(clusters):
            centroid = np.mean(cluster, axis=0)
            #self.get_logger().info(f'Cluster {i}: Centroid at {(centroid[0]+self.robot_state[0],centroid[1]+self.robot_state[1])}') 
            angle = self.robot_state[3]

            self.obstacles.append((math.cos(angle)*centroid[0]-math.sin(angle)*centroid[1]+self.robot_state[0],
                                   math.sin(angle)*centroid[0]+math.cos(angle)*centroid[1]+self.robot_state[1]))       

                


    def clock_callback(self, msg):
        self.sim_time = msg.clock
        #print(self.sim_time)


    def spawn_request(self):
        self.get_logger().info('Spawning circle...')
        request = SpawnEntity.Request()

        # SDF 파일의 경로 설정  
        pkg_share = get_package_share_directory('nav_project')
        sdf_file_path = os.path.join(pkg_share, 'models/circle.sdf')

        with open(sdf_file_path, 'r') as f:
            sdf_content = f.read()

        request.name = 'circle'
        request.xml = sdf_content
        request.robot_namespace = ''
        request.initial_pose.position.x = self.goal[0]
        request.initial_pose.position.y = self.goal[1]
        request.initial_pose.position.z = 0.0
        request.initial_pose.orientation.x = 0.0
        request.initial_pose.orientation.y = 0.0
        request.initial_pose.orientation.z = 0.0
        request.initial_pose.orientation.w = 1.0

        self.spawn_future = self.spawn_cli.call_async(request)
    def delete_request(self):
        self.get_logger().info('Deleting circle...')
        request = DeleteEntity.Request()
        request.name = 'circle'
        self.delete_future = self.delete_cli.call_async(request)    

        
    def odom_callback(self, msg):
        self.robot_state[0] = msg.pose.pose.position.x
        self.robot_state[1] = msg.pose.pose.position.y
        # Orientation to yaw conversion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_state[3] = math.atan2(siny_cosp, cosy_cosp)
        self.robot_state[2] = msg.twist.twist.linear.x
        self.robot_state[4] = msg.twist.twist.angular.z



    def goal_callback(self, msg):
        a = 1
        #self.goal[0] = msg.pose.position.x
        #self.goal[1] = msg.pose.position.y

    def timer_callback(self):

        best_vw = self.DWA(self.goal)

        cmd_vel = Twist()
        cmd_vel.linear.x = best_vw[0]# check it!
        cmd_vel.linear.y = 0.0
        cmd_vel.angular.z = best_vw[1]
        self.publisher.publish(cmd_vel)
        
        if distance((self.robot_state[0],self.robot_state[1]),self.goal) < robot_radius: # 새 goal 생성 부분
            print("goal reached.")
            self.delete_request()
            time.sleep(0.2)
            self.goal = np.array((random.uniform(-0.8,0.8),random.uniform(-0.8,0.8)))
            self.spawn_request()
            time.sleep(0.2)
            while self.goal[0] >=-0.2 and self.goal[0] <=0.2 and self.goal[1] >=-0.2 and self.goal[1] <=0.2: 
                self.delete_request()
                time.sleep(0.2)
                self.goal = np.array((random.uniform(-0.8,0.8),random.uniform(-0.8,0.8)))
                self.spawn_request()
                time.sleep(0.2)

        
    def create_dynamic_window(self,robot_states): # 일단은 장애물 말고 동적 제약조건만 가정
        velocity_constraint = [min_speed, max_speed, -max_omega, max_omega]
        u = [robot_states[2],robot_states[4]]
        velocity_range = generate_velocity_range(u,dt)
        dw = [max(velocity_constraint[0],velocity_range[0]), # 최대/최소 속도 제약조건 넘지 않도록 설정.
            min(velocity_constraint[1],velocity_range[1]),
            max(velocity_constraint[2],velocity_range[2]),
            min(velocity_constraint[3],velocity_range[3])]
        return dw
        


    def motion(self,x, u, dt):
        # x = [x, y, v, angle, omega]
        x[3] += u[1] * dt
        x[0] += u[0] * math.cos(x[3]) * dt
        x[1] += u[0] * math.sin(x[3]) * dt
        x[2] = u[0]
        x[4] = u[1]
        return x


    def calc_trajectory(self,robot_states, v, omega):
        trajectory = np.array(robot_states)
        delta_t = self.dt 
        time = delta_t
        while time <= future_time:
            robot_states = self.motion(np.copy(robot_states), [v, omega], delta_t)
            trajectory = np.vstack((trajectory, robot_states))
            time += self.dt
        return trajectory 
        


    def objective_function(self,trajectory, end): # 평가 함수. 목적함수가 최소가 되는 v,w쌍 찾기
        heading = distance(trajectory[-1],end)
        clearance = 0
        dist_list = []
        #print(self.obstacles)
        for ob in self.obstacles:
            for point in trajectory:
                pointxy = [point[0],point[1]]
                dist_list.append(distance(pointxy,ob))
                if min(dist_list) <= self.obstacle_radius:
                    return float('inf')  #궤적이 충돌하면 비용 크게
                clearance += 1.0 / min(dist_list)
            dist_list = []
        velocity = -trajectory[-1,2]
        ob = alpha * heading + beta * clearance + gamma * velocity
        
        return ob   

    def DWA(self, end):
        min_val = float("inf")
        temp_state = np.copy(self.robot_state)
        dw = self.create_dynamic_window(temp_state)
        sample_list = velocity_sampling(dw)
        best_vw = []
        is_collied = True
        for sample in sample_list:
            trajectory = self.calc_trajectory(temp_state,sample[0],sample[1])
            ob = self.objective_function(trajectory, end)
            #print(trajectory[-1][0],trajectory[-1][1])
            if min_val > ob:
                is_collied = False
                min_val = ob
                best_trajectory = trajectory
                best_vw = [sample[0],sample[1]]
            elif is_collied:

                best_vw = [-0.22, 0.0]
        #print(best_trajectory[-1][0],best_trajectory[-1][1])
        #print(robot_states,", ",best_vw)  
        #robot_states = self.motion(robot_states, best_vw, dt)
        #past_trajectory = np.vstack((past_trajectory, robot_states))
        
        return best_vw

def main(args=None):
    rclpy.init(args=args)
    node = DWANode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
