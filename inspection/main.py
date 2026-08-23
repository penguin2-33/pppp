# -*- coding: utf-8 -*-
"""工业配电房智能巡检 - 主流程

M1 最小闭环目标：打通
  ZMQ 连接 -> 启动仿真 -> 环境检查 -> 抓帧 -> 基本导航 -> 结果记录
后续 M2~M7 在此框架上扩展：多点位巡检 / YOLO 识别 / 状态判读 /
异常提示 / 障碍避让 / 完整结果输出。

运行前提：
  1. CoppeliaSim 已打开对应场景并处于可连接状态（默认 ZMQ 端口 23000）。
  2. 场景对象命名符合 config.py 的命名约定（见 README.md）。
"""
import json
import os
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import config
from robot import DifferentialDrive
import sensors
import env_check


def main():
    # 0. 准备输出目录
    config.ensure_output_dirs()

    # 1. 连接 CoppeliaSim (ZMQ Remote API)
    print("[1/6] 连接 CoppeliaSim ...")
    client = RemoteAPIClient()
    sim = client.require("sim")
    print("      连接成功")

    # 2. 启动仿真
    print("[2/6] 启动仿真 ...")
    sim.startSimulation()
    time.sleep(1.0)

    # 3. 环境检查
    print("[3/6] 仿真环境检查 ...")
    report = env_check.check(sim)
    env_check.print_report(report)

    # 4. 视觉传感器抓帧测试
    print("[4/6] 视觉传感器抓帧测试 ...")
    vis = sim.getObject(config.VISION_SENSOR)
    img = sensors.grab_image(sim, vis)
    shot = os.path.join(config.SHOT_DIR, "m1_frame_test.png")
    sensors.save_image(sim, vis, shot)
    print(f"      抓帧成功，图像尺寸 {img.shape[1]}x{img.shape[0]}，已保存 {shot}")

    # 5. 基本导航测试：驶向点位1
    print("[5/6] 导航到点位1 ...")
    left = sim.getObject(config.LEFT_MOTOR)
    right = sim.getObject(config.RIGHT_MOTOR)
    robot_base = sim.getObject(config.ROBOT_NAME)
    robot = DifferentialDrive(sim, left, right, config.NAV)

    wp1_handle = sim.getObject(config.WAYPOINTS[0])
    wp1 = sim.getObjectPosition(wp1_handle, -1)
    print(f"      目标点位坐标: ({wp1[0]:.3f}, {wp1[1]:.3f})")
    reached, steps = robot.go_to_point(robot_base, wp1[:2])
    print(f"      到达状态: {'成功' if reached else '未达容差'}, 控制步数: {steps}")

    # 6. 记录结果
    print("[6/6] 记录结果 ...")
    result = {
        "task": "M1 最小闭环测试",
        "env_check_passed": report["_all_ok"],
        "frame_size": [img.shape[1], img.shape[0]],
        "waypoint_1_reached": reached,
        "control_steps": steps,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    with open(config.RESULT_JSON, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(f"      结果已保存 {config.RESULT_JSON}")

    print("\nM1 最小闭环测试完成")
    print("仿真仍在运行，可在 CoppeliaSim 界面观察机器人；按 Ctrl+C 退出。")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n运行异常: {e}")
