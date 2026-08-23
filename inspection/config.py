# -*- coding: utf-8 -*-
"""巡检任务配置文件

所有场景对象命名、点位、导航/感知参数、输出路径集中在此。
答辩亮点：体现"配置驱动"——改配置不改代码即可调整任务。
搭建场景时，务必让场景对象名与此处命名约定一致（见 README.md）。
"""
import os

# ==================== 场景对象命名约定 ====================
# 机器人（Pioneer P3-DX 底座 + 加装传感器）
ROBOT_NAME = "/P3DX"
LEFT_MOTOR = "/P3DX/leftMotor"        # 左轮关节
RIGHT_MOTOR = "/P3DX/rightMotor"      # 右轮关节
VISION_SENSOR = "/P3DX/VisionSensor"  # 视觉传感器（云台相机）
LIDAR = "/P3DX/Hokuyo"                # fast Hokuyo 激光雷达
PROX_FRONT = "/P3DX/proxFront"        # 前方接近传感器（急停保护）

# 巡检点位（用 dummy 对象标记，按运行顺序排列）
WAYPOINTS = ["/waypoint_01", "/waypoint_02", "/waypoint_03"]

# 目标对象（按类别分组）
# 注：仪表/指示灯/按钮/开关已改为「配电柜前面板的安装件」（柜体子对象），
# 命名用完整层级路径；alias 已设为简单名，故 /meter_01 等短路径仍可解析。
TARGETS = {
    "cabinet": ["/cabinet_01", "/cabinet_02", "/cabinet_03"],   # 配电柜（主体）
    "meter":   ["/cabinet_02/meter_01"],                        # 指针仪表（装在 cabinet_02 面板）
    # 指示灯：一排 3 绿（运行）+ 1 红（告警），真实配电柜风格
    "lamp_green": ["/cabinet_03/lamp_green_1",
                   "/cabinet_03/lamp_green_2",
                   "/cabinet_03/lamp_green_3"],
    "lamp_red":   ["/cabinet_03/lamp_red"],                     # 红色告警灯
    "sign":   ["/sign_01"],                                     # 安全警示标识（挂背墙）
    # 按钮：绿色启动 + 红色停止；旋钮开关
    "button": ["/cabinet_01/btn_start", "/cabinet_01/btn_stop"],
    "switch": ["/cabinet_01/switch_01"],                        # 旋钮开关（装在 cabinet_01）
    "obstacle": ["/obstacle_01", "/obstacle_02"],               # 通道障碍物
}

# ==================== 导航参数 ====================
NAV = {
    "max_linear_speed": 0.3,     # 最大线速度 m/s
    "max_angular_speed": 0.8,    # 最大角速度 rad/s
    "pos_tolerance": 0.05,       # 到达位置容差 m
    "angle_tolerance": 0.1,      # 到达朝向容差 rad
    "wheel_radius": 0.0975,      # P3-DX 轮半径 m（以实际模型为准）
    "wheel_base": 0.331,         # P3-DX 轮距 m（实测左右轮中心 y 间距 ±0.165）
}

# ==================== 障碍感知参数 ====================
OBSTACLE = {
    "slow_distance": 0.8,        # 进入减速的距离 m
    "stop_distance": 0.35,       # 急停距离 m
    "sidestep_distance": 0.6,    # 侧向绕行判定距离 m
}

# ==================== 结果输出 ====================
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
RESULT_JSON = os.path.join(OUTPUT_DIR, "inspection_result.json")
RESULT_CSV = os.path.join(OUTPUT_DIR, "inspection_result.csv")
SHOT_DIR = os.path.join(OUTPUT_DIR, "shots")   # 现场截图目录


def ensure_output_dirs():
    """确保输出目录存在"""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    os.makedirs(SHOT_DIR, exist_ok=True)
