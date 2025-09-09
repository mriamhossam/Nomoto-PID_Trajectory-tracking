import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math

class NomotoModel:
    def __init__(self):
        self.x = 0.0      # 船舶x位置
        self.y = 0.0      # 船舶y位置
        self.psi = 0.0    # 航向角
        self.r = 0.0      # 角速度
        self.u = 4.0      # 船速 (m/s)
        
        # 模型参数
        self.T = 2.0      # 时间常数
        self.K = 1.0      # 增益
        
    def update(self, delta, dt):
        # 更新角速度和航向角
        r_dot = (-self.r + self.K * delta) / self.T
        self.r += r_dot * dt
        self.psi += self.r * dt
        
        # 更新位置
        self.x += self.u * math.cos(self.psi) * dt
        self.y += self.u * math.sin(self.psi) * dt

class Controller:
    def __init__(self):
        # PID控制参数
        self.Kp = 2.0
        self.Ki = 0.005
        self.Kd = 4.0
        
        # 状态变量
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_desired_psi = None
        
        # 限制参数
        self.max_integral = 0.1
        self.max_yaw_rate = 0.15
        self.max_heading_change = math.pi/12
        
        # 前视参数
        self.look_ahead_min = 15.0
        self.look_ahead_max = 30.0
        
        # 航向控制参数
        self.global_direction = None
        self.is_turning = False
        self.forward_angle_threshold = math.pi/2
        
    def is_point_forward(self, ship_x, ship_y, ship_psi, point_x, point_y):
        angle = math.atan2(point_y - ship_y, point_x - ship_x)
        relative_angle = abs(self.normalize_angle(angle - ship_psi))
        return relative_angle < self.forward_angle_threshold
        
    def find_target(self, current_idx, target_x, target_y, ship_x, ship_y, ship_psi):
        idx = current_idx
        max_points = 20
        best_idx = current_idx
        min_angle = float('inf')
        
        current_dir = math.atan2(target_y[current_idx+1] - target_y[current_idx],
                                target_x[current_idx+1] - target_x[current_idx])
        
        while idx < len(target_x) - 1 and idx - current_idx < max_points:
            if self.is_point_forward(ship_x, ship_y, ship_psi, target_x[idx], target_y[idx]):
                if idx < len(target_x) - 1:
                    next_dir = math.atan2(target_y[idx+1] - target_y[idx],
                                        target_x[idx+1] - target_x[idx])
                    angle_diff = abs(self.normalize_angle(next_dir - current_dir))
                    
                    if angle_diff < min_angle:
                        min_angle = angle_diff
                        best_idx = idx
            idx += 1
        
        return best_idx

    def calc_control(self, target_x, target_y, next_x, next_y, ship_x, ship_y, ship_psi, dt):
        if self.global_direction is None:
            self.global_direction = ship_psi
        
        path_angle = math.atan2(next_y - target_y, next_x - target_x)
        current_angle = math.atan2(target_y - ship_y, target_x - ship_x)
        self.is_turning = abs(self.normalize_angle(path_angle - current_angle)) > math.pi/4
        
        # 计算横向误差
        dx = ship_x - target_x
        dy = ship_y - target_y
        cross_track_error = -math.sin(path_angle) * dx + math.cos(path_angle) * dy
        
        # 计算期望航向
        if self.is_turning:
            look_ahead = self.look_ahead_max
            preview_dist = look_ahead * 1.5
            preview_x = target_x + preview_dist * math.cos(path_angle)
            preview_y = target_y + preview_dist * math.sin(path_angle)
            desired_psi = math.atan2(preview_y - ship_y, preview_x - ship_x)
        else:
            look_ahead = self.look_ahead_min
            desired_psi = path_angle + 0.5 * math.atan2(cross_track_error, look_ahead)
        
        # 限制航向变化率
        if self.prev_desired_psi is not None:
            heading_change = self.normalize_angle(desired_psi - self.prev_desired_psi)
            max_change = self.max_yaw_rate * dt
            if abs(heading_change) > max_change:
                heading_change = max_change if heading_change > 0 else -max_change
                desired_psi = self.prev_desired_psi + heading_change
        
        self.prev_desired_psi = desired_psi
        error = self.normalize_angle(desired_psi - ship_psi)
        
        # PID控制
        if not self.is_turning and abs(error) < math.pi/6:
            self.integral = max(min(self.integral + error * dt, self.max_integral), -self.max_integral)
        else:
            self.integral = 0
        
        derivative = 0.3 * (error - self.prev_error) / dt
        self.prev_error = error
        
        # 根据转向状态调整PID参数
        Kp = self.Kp * (0.7 if self.is_turning else 1.0)
        Ki = 0 if self.is_turning else self.Ki
        Kd = self.Kd * (1.2 if self.is_turning else 1.0)
        
        # 计算舵角
        delta = Kp * error + Ki * self.integral + Kd * derivative
        max_rudder = math.pi/6 if self.is_turning else math.pi/12
        
        return max(min(delta, max_rudder), -max_rudder)
    
    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

def main():
    # 读取轨迹数据
    df = pd.read_csv('history4s.csv')
    sampling_interval = 2
    target_x = df['x'].values[::sampling_interval]
    target_y = df['y'].values[::sampling_interval]
    
    # 初始化系统
    ship = NomotoModel()
    controller = Controller()
    ship.x, ship.y = target_x[0], target_y[0]
    ship.psi = math.radians(90)
    
    # 仿真参数
    dt = 0.1
    actual_x, actual_y = [ship.x], [ship.y]
    time_points = [0]  # 记录时间点
    psi_history = [math.degrees(ship.psi)]  # 记录航向角历史（转换为角度）
    delta_history = [0]  # 记录舵角历史（转换为角度）
    target_idx = 0
    
    # 设置图形
    plt.ion()
    fig = plt.figure(figsize=(15, 10))
    
    # 轨迹子图
    ax_traj = fig.add_subplot(211)
    target_line, = ax_traj.plot(df['x'], df['y'], 'b-', label='Target Path')
    actual_line, = ax_traj.plot(actual_x, actual_y, 'r--', label='Actual Path')
    ship_point, = ax_traj.plot([], [], 'k^', markersize=10, label='Ship Position')
    
    ax_traj.grid(True)
    ax_traj.set_xlabel('X Position (m)')
    ax_traj.set_ylabel('Y Position (m)')
    ax_traj.set_title('Ship Trajectory Tracking')
    ax_traj.legend()
    ax_traj.axis('equal')
    
    # 航向角和舵角子图
    ax_angles = fig.add_subplot(212)
    psi_line, = ax_angles.plot(time_points, psi_history, 'b-', label='Heading (deg)')
    delta_line, = ax_angles.plot(time_points, delta_history, 'r--', label='Rudder (deg)')
    
    ax_angles.grid(True)
    ax_angles.set_xlabel('Time (s)')
    ax_angles.set_ylabel('Angle (deg)')
    ax_angles.set_title('Heading and Rudder Angles')
    ax_angles.legend()
    
    plt.tight_layout()
    
    try:
        while target_idx < len(target_x) - 2:
            # 寻找目标点
            target_idx = controller.find_target(target_idx, target_x, target_y, 
                                             ship.x, ship.y, ship.psi)
            next_idx = min(target_idx + 1, len(target_x) - 1)
            
            # 计算控制量并更新状态
            delta = controller.calc_control(target_x[target_idx], target_y[target_idx],
                                         target_x[next_idx], target_y[next_idx],
                                         ship.x, ship.y, ship.psi, dt)
            ship.update(delta, dt)
            
            # 记录数据
            current_time = time_points[-1] + dt
            time_points.append(current_time)
            psi_history.append(math.degrees(ship.psi))  # 转换为角度
            delta_history.append(math.degrees(delta))   # 转换为角度
            actual_x.append(ship.x)
            actual_y.append(ship.y)
            
            # 更新目标点
            dist = math.sqrt((ship.x - target_x[target_idx])**2 + 
                           (ship.y - target_y[target_idx])**2)
            
            if dist < (8.0 if controller.is_turning else 4.0):
                next_idx = target_idx + 1
                if next_idx < len(target_x) and controller.is_point_forward(
                    ship.x, ship.y, ship.psi, target_x[next_idx], target_y[next_idx]):
                    target_idx = next_idx
            
            # 更新图形
            actual_line.set_data(actual_x, actual_y)
            ship_point.set_data([ship.x], [ship.y])
            psi_line.set_data(time_points, psi_history)
            delta_line.set_data(time_points, delta_history)
            
            # 自动调整航向角和舵角图的显示范围
            ax_angles.relim()
            ax_angles.autoscale_view()
            
            plt.pause(0.01)
            
    except KeyboardInterrupt:
        print("\nSimulation stopped")
    
    # 保存数据到Excel文件
    data = {
        'Time(s)': time_points,
        'X(m)': actual_x,
        'Y(m)': actual_y,
        'Heading(deg)': psi_history,
        'Rudder(deg)': delta_history
    }
    df_result = pd.DataFrame(data)
    df_result.to_excel('history4s_control.xlsx', index=False)
    print("\n数据已保存到 history4s_control.xlsx")
    
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main() 