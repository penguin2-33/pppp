# -*- coding: utf-8 -*-
"""复位：停止仿真 + 机器人回到起点 + 轮关节归零。"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

sim = RemoteAPIClient().require("sim")
sim.stopSimulation()

robot = sim.getObject("/P3DX")
sim.setObjectPosition(robot, -1, [-2.0, -1.0, 0.2])
sim.setObjectOrientation(robot, -1, [0.0, 0.0, 0.0])
for p in ["/P3DX/leftMotor", "/P3DX/rightMotor"]:
    sim.setJointPosition(sim.getObject(p), 0.0)

print("[OK] 已复位：仿真停止，机器人回到起点 (-2,-1,0.2)")
