# -*- coding: utf-8 -*-
"""复位：停止仿真 + 机器人回到起点 + 轮关节归零。"""
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

print("[1/3] 正在尝试连接 CoppeliaSim 的 ZMQ 接口...")
print("      （如果卡在这里，说明 CoppeliaSim 没开，或者弹窗挡住了！）")

# 连接（如果 CoppeliaSim 没运行，这里会一直等）
sim = RemoteAPIClient().require("sim")
print("[2/3] 连接成功，正在停止仿真...")

sim.stopSimulation()
time.sleep(0.5)

robot = sim.getObject("/P3DX")
if robot is None:
    print("      [错误] 找不到 /P3DX，请检查场景！")
else:
    sim.setObjectPosition(robot, -1, [-2.0, 1.0, 0.2])
    sim.setObjectOrientation(robot, -1, [0.0, 0.0, 0.0])
    for p in ["/P3DX/leftMotor", "/P3DX/rightMotor"]:
        j = sim.getObject(p)
        if j is not None:
            sim.setJointPosition(j, 0.0)

print("[3/3] [OK] 已复位：仿真停止，机器人回到起点 (-2,1,0.2)")