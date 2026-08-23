# -*- coding: utf-8 -*-
"""机器人差速驱动与导航控制模块

Pioneer P3-DX 为两轮差速驱动，运动学关系：
    线速度 v = (vl + vr) * r / 2
    角速度 w = (vr - vl) * r / L
其中 r 为轮半径、L 为轮距、vl/vr 为左右轮角速度(rad/s)。
答辩亮点：可现场推导差速运动学，并说明比例控制的参数整定。
"""
import math
import time


def normalize_angle(a):
    """角度归一化到 [-pi, pi]"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class DifferentialDrive:
    """差速驱动底盘封装：底层轮速控制 + 上层导航"""

    def __init__(self, sim, left_handle, right_handle, nav_cfg):
        self.sim = sim
        self.left = left_handle
        self.right = right_handle
        self.cfg = nav_cfg

    # ---------------- 底层控制 ----------------
    def set_wheel_speeds(self, vl, vr):
        """设置左右轮目标角速度 rad/s"""
        self.sim.setJointTargetVelocity(self.left, vl)
        self.sim.setJointTargetVelocity(self.right, vr)

    def stop(self):
        """停车"""
        self.set_wheel_speeds(0.0, 0.0)

    def set_twist(self, v, w):
        """给定线速度 v(m/s) 与角速度 w(rad/s)，换算为左右轮速"""
        r = self.cfg["wheel_radius"]
        L = self.cfg["wheel_base"]
        vl = (v - w * L / 2.0) / r
        vr = (v + w * L / 2.0) / r
        self.set_wheel_speeds(vl, vr)

    # ---------------- 位姿 ----------------
    def get_pose(self, robot_name):
        """返回机器人位姿 (pos[x,y,z], yaw)。yaw 取欧拉角 gamma（绕 Z 轴）。"""
        pos = self.sim.getObjectPosition(robot_name, -1)
        ori = self.sim.getObjectOrientation(robot_name, -1)
        return pos, ori[2]

    # ---------------- 导航 ----------------
    def go_to_point(self, robot_name, target_pos, target_yaw=None,
                    step=0.05, max_steps=10000):
        """驶向目标点(米)，可选朝向对准。返回 (是否到达, 步数)。

        控制策略（简单可靠，符合职教定位）：
          1) 计算目标方向角，若朝向偏差大则原地转向对准；
          2) 对准后前进，速度与剩余距离成比例；
          3) 到达位置容差后，若指定了目标朝向再做原地对准。
        """
        nav = self.cfg
        kp_ang = 2.0   # 角速度比例增益
        kp_lin = 1.5   # 线速度比例增益

        for i in range(max_steps):
            pos, yaw = self.get_pose(robot_name)
            dx = target_pos[0] - pos[0]
            dy = target_pos[1] - pos[1]
            dist = math.hypot(dx, dy)

            # 位置已到达
            if dist < nav["pos_tolerance"]:
                if target_yaw is not None:
                    err = normalize_angle(target_yaw - yaw)
                    if abs(err) > nav["angle_tolerance"]:
                        w = max(-nav["max_angular_speed"],
                                min(nav["max_angular_speed"], kp_ang * err))
                        self.set_twist(0.0, w)
                        time.sleep(step)
                        continue
                self.stop()
                return True, i

            # 目标方向角
            target_angle = math.atan2(dy, dx)
            err = normalize_angle(target_angle - yaw)

            if abs(err) > 0.15:
                # 先原地转向对准
                w = max(-nav["max_angular_speed"],
                        min(nav["max_angular_speed"], kp_ang * err))
                self.set_twist(0.0, w)
            else:
                # 对准后前进，角速度微调保持方向
                v = max(0.0, min(nav["max_linear_speed"], kp_lin * dist))
                self.set_twist(v, kp_ang * err * 0.5)
            time.sleep(step)

        self.stop()
        return False, max_steps
