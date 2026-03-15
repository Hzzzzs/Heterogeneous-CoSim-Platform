"""
Module: ugv_chassis_control.py
Description: 基于双轮差速运动学的 UGV 底盘控制器 (将全局速度解算为电机转速)
Author: Weicheng Li (李蔚澄)
"""
import numpy as np
import pybullet as p

class UGVController:
    def __init__(self, ugv_id, physics_client):
        self.ugv_id = ugv_id
        self.CLIENT = physics_client
        
        # Husky 越野车的近似物理参数
        self.wheel_radius = 0.165  # 车轮半径 (米)
        self.track_width = 0.55    # 左右轮距 (米)
        self.max_torque = 50.0     # 电机最大扭矩
        
        # 自动识别左右车轮的关节索引
        self.left_wheels = []
        self.right_wheels = []
        self._identify_wheel_joints()

    def _identify_wheel_joints(self):
        """遍历 URDF，找到并分类左右车轮的 Joint ID"""
        num_joints = p.getNumJoints(self.ugv_id, physicsClientId=self.CLIENT)
        for j in range(num_joints):
            info = p.getJointInfo(self.ugv_id, j, physicsClientId=self.CLIENT)
            joint_name = info[1].decode('utf-8')
            if 'front_left' in joint_name or 'rear_left' in joint_name:
                self.left_wheels.append(j)
            elif 'front_right' in joint_name or 'rear_right' in joint_name:
                self.right_wheels.append(j)

    def compute_and_apply_control(self, current_quat, target_vel_world):
        """
        核心控制逻辑：将全局期望速度向量转化为左右轮电机指令并下发
        """
        # 1. 获取当前车头朝向 (Yaw角)
        rpy = p.getEulerFromQuaternion(current_quat)
        current_yaw = rpy[2]

        # 2. 从全局速度向量中提取目标参数
        vx, vy = target_vel_world[0], target_vel_world[1]
        target_speed = np.sqrt(vx**2 + vy**2) # 期望线速度大小
        
        # 如果期望速度极小，说明已经到了位置，直接刹车防震荡
        if target_speed < 0.05:
            self._apply_motor_control(0.0, 0.0)
            return

        # 计算目标航向角 (向量的夹角)
        target_yaw = np.arctan2(vy, vx)

        # 3. 计算航向误差，并将其归一化到 [-pi, pi] 之间
        yaw_error = target_yaw - current_yaw
        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi

        # 4. 运动学解算 (P 控制器)
        Kp_yaw = 3.0  # 转向增益：越大转弯越猛
        Kp_v = 1.0    # 速度增益

        # 🌟 核心非完整约束逻辑：航向误差越大，越要减速先转弯
        # 使用 cos 函数作为减速因子，如果误差超过 90 度，车辆甚至会先原地打转或倒车
        speed_factor = np.cos(yaw_error) 
        
        v_cmd = Kp_v * target_speed * speed_factor  # 最终底盘线速度
        w_cmd = Kp_yaw * yaw_error                  # 最终底盘角速度

        # 5. 双轮差速运动学逆解
        # 左轮速度 = 线速度 - (角速度 * 轮距 / 2)
        # 右轮速度 = 线速度 + (角速度 * 轮距 / 2)
        v_left = v_cmd - (w_cmd * self.track_width / 2.0)
        v_right = v_cmd + (w_cmd * self.track_width / 2.0)

        # 将线速度 (m/s) 转化为车轮角速度 (rad/s)
        w_left_wheel = v_left / self.wheel_radius
        w_right_wheel = v_right / self.wheel_radius

        # 6. 下发指令到物理引擎
        self._apply_motor_control(w_left_wheel, w_right_wheel)

    def _apply_motor_control(self, w_left, w_right):
        """调用 PyBullet 底层 API 驱动关节"""
        for left_joint in self.left_wheels:
            p.setJointMotorControl2(self.ugv_id, left_joint, p.VELOCITY_CONTROL, 
                                    targetVelocity=w_left, force=self.max_torque, physicsClientId=self.CLIENT)
        for right_joint in self.right_wheels:
            p.setJointMotorControl2(self.ugv_id, right_joint, p.VELOCITY_CONTROL, 
                                    targetVelocity=w_right, force=self.max_torque, physicsClientId=self.CLIENT)