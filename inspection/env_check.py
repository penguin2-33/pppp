# -*- coding: utf-8 -*-
"""仿真环境检查模块

任务运行前检查：机器人模型、传感器接口、通信链路、场景资源。
对应评分项"仿真环境检查与规范操作"。
"""
from config import (ROBOT_NAME, LEFT_MOTOR, RIGHT_MOTOR, VISION_SENSOR,
                    LIDAR, PROX_FRONT, WAYPOINTS, TARGETS)


def check(sim):
    """检查关键对象句柄是否有效，返回检查报告 dict"""
    report = {}

    def _chk(name, label):
        try:
            h = sim.getObject(name)
            ok = h is not None and h >= 0
            report[label] = (ok, name, h)
            return ok
        except Exception as e:
            report[label] = (False, name, str(e))
            return False

    ok = True
    # 机器人及传感器
    ok &= _chk(ROBOT_NAME, "机器人底座")
    ok &= _chk(LEFT_MOTOR, "左轮关节")
    ok &= _chk(RIGHT_MOTOR, "右轮关节")
    ok &= _chk(VISION_SENSOR, "视觉传感器")
    ok &= _chk(LIDAR, "激光雷达")
    ok &= _chk(PROX_FRONT, "前方接近传感器")

    # 巡检点位
    for wp in WAYPOINTS:
        ok &= _chk(wp, f"巡检点位 {wp}")

    # 目标对象
    for cat, objs in TARGETS.items():
        for o in objs:
            ok &= _chk(o, f"目标 {cat}:{o}")

    report["_all_ok"] = ok
    return report


def print_report(report):
    """打印检查报告"""
    print("=" * 56)
    print("仿真环境检查报告")
    print("=" * 56)
    for k, v in report.items():
        if k == "_all_ok":
            continue
        ok, name, h = v
        status = "OK  " if ok else "FAIL"
        print(f"  [{status}] {k:<18} -> {name} (handle={h})")
    print("-" * 56)
    print("总体状态:", "通过" if report["_all_ok"] else "存在缺失，请核对场景对象命名")
    print("=" * 56)
    return report["_all_ok"]
