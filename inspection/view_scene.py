# -*- coding: utf-8 -*-
"""停仿真 + 复位机器人 + 把默认相机对准场景，让 GUI 直接看到全景。"""
import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

sim = RemoteAPIClient().require("sim")

# 1. 停仿真
sim.stopSimulation()

# 2. 复位机器人到起点
robot = sim.getObject("/P3DX")
sim.setObjectPosition(robot, -1, [-2.0, -1.0, 0.2])
sim.setObjectOrientation(robot, -1, [0.0, 0.0, 0.0])
for p in ["/P3DX/leftMotor", "/P3DX/rightMotor"]:
    sim.setJointPosition(sim.getObject(p), 0.0)

# 3. 找默认相机
cam = None
for o in sim.getObjectsInTree(sim.handle_scene, sim.handle_all):
    if sim.getObjectName(o) == "DefaultCamera":
        cam = o
        break

if cam is not None:
    # 相机放到场景斜前方，看向场景中心 (3, -0.3, 0.9)
    P = (3.0, -5.0, 3.2)
    T = (3.0, -0.3, 0.9)
    dx, dy, dz = T[0] - P[0], T[1] - P[1], T[2] - P[2]
    L = math.sqrt(dx * dx + dy * dy + dz * dz)
    dx, dy, dz = dx / L, dy / L, dz / L
    # 相机观察方向为 -Z：R*e_z = -f，f=(dx,dy,dz)
    beta = math.asin(max(-1.0, min(1.0, -dx)))
    alpha = math.atan2(dy, -dz)
    gamma = 0.0
    sim.setObjectPosition(cam, -1, list(P))
    sim.setObjectOrientation(cam, -1, [alpha, beta, gamma])
    print(f"[OK] 相机已对准场景 位置={P} 朝向={[round(alpha,3), round(beta,3), 0]}")
else:
    print("[警告] 未找到 DefaultCamera")

print("[OK] 已停仿真并复位机器人")
print("场景对象数:", len(sim.getObjectsInTree(sim.handle_scene, sim.handle_all)))
