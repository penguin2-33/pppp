# -*- coding: utf-8 -*-
"""工业配电房智能巡检 - 主流程（M1-M5 + M7 完整版）"""
import json
import os
import time
import math
import csv
import cv2

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import config
from robot import DifferentialDrive
import sensors
import env_check
import vision


def main():
    config.ensure_output_dirs()
    # M5：确保异常文件夹存在
    alarms_dir = os.path.join(config.SHOT_DIR, "alarms")
    os.makedirs(alarms_dir, exist_ok=True)

    print("[1/4] 连接 CoppeliaSim ...")
    client = RemoteAPIClient()
    sim = client.require("sim")
    print("      连接成功")

    print("[2/4] 启动仿真 ...")
    sim.startSimulation()
    time.sleep(1.0)

    print("[3/4] 仿真环境检查 ...")
    report = env_check.check(sim)
    env_check.print_report(report)

    print("[4/4] 开始按顺序行驶并执行 M3/M4/M5/M7 ...")
    left = sim.getObject(config.LEFT_MOTOR)
    right = sim.getObject(config.RIGHT_MOTOR)
    robot_base = sim.getObject(config.ROBOT_NAME)
    robot = DifferentialDrive(sim, left, right, config.NAV)

    # 严格按 1,4,5,2,3,6,7,8,9 的顺序走
    waypoint_order = [
        "waypoint_01", "waypoint_04", "waypoint_05",
        "waypoint_02", "waypoint_03", "waypoint_06",
        "waypoint_07", "waypoint_08", "waypoint_09"
    ]

    points_coords = {}
    for wp_name in waypoint_order:
        wp_handle = sim.getObject("/" + wp_name)
        if wp_handle is None or wp_handle < 0:
            print(f"      [错误] 找不到点位 {wp_name}！")
            return
        pos = sim.getObjectPosition(wp_handle, -1)
        points_coords[wp_name] = pos[:2]

    # 恢复4号点的检测：面向配电柜1
    face_targets = {
        "waypoint_04": "/cabinet_01",
        "waypoint_05": "/cabinet_02",
        "waypoint_06": "/cabinet_03",
        "waypoint_07": "/transformer",
    }

    vis = sim.getObject(config.VISION_SENSOR)

    STOP_DURATION = 0.5
    TURN_DURATION = 0.3
    FACE_DURATION = 1.0

    print("      开始巡检...")
    status_log = []
    anomaly_count = 0

    for i, wp_name in enumerate(waypoint_order):
        cur_pos = points_coords[wp_name]

        # 8号点直接穿过，不停顿，直接去9
        if wp_name == "waypoint_08":
            next_pos = points_coords["waypoint_09"]
            print("      经过 waypoint_08（不做停留），直接前往 waypoint_09 ...")
            robot.go_to_point(robot_base, cur_pos)
            robot.go_to_point(robot_base, next_pos)
            robot.stop()
            break

        # 注：4号点现在正常参与检测，不再“直接穿过”，因此删除了原有的跳过逻辑。

        print(f"      正在直线前往 {wp_name}，坐标 ({cur_pos[0]:.2f}, {cur_pos[1]:.2f}) ...")
        robot.go_to_point(robot_base, cur_pos)
        robot.stop()

        time.sleep(STOP_DURATION)
        print(f"      已到达 {wp_name}，停顿0.5秒")

        if wp_name in face_targets:
            face_obj_name = face_targets[wp_name]
            print(f"      [检测] 面向 {face_obj_name} 停1秒...")

            face_handle = sim.getObject(face_obj_name)
            if face_handle is not None:
                face_pos = sim.getObjectPosition(face_handle, -1)
                target_yaw = math.atan2(face_pos[1] - cur_pos[1], face_pos[0] - cur_pos[0])

                robot.go_to_point(robot_base, cur_pos, target_yaw=target_yaw)
                robot.stop()
                time.sleep(FACE_DURATION)
                print(f"      已面向箱子并停留1秒。")

                # ======== M3 识别 + M4 判读 + M5 异常 ========
                img = sensors.grab_image(sim, vis)
                shot_path = os.path.join(config.SHOT_DIR, f"检测_{wp_name}_{face_obj_name.split('/')[-1]}.png")
                sensors.save_image(sim, vis, shot_path)
                print(f"      [拍照] 原图已保存: {shot_path}")

                detections = vision.detect_objects(img)
                print(f"      [YOLO] 识别到 {len(detections)} 个物体:")
                obj_names = []
                for det in detections:
                    print(f"            - {det['class']} (置信度: {det['conf']})")
                    obj_names.append(f"{det['class']}({det['conf']})")

                annotated_img = vision.draw_detections(img, detections)
                yolo_path = shot_path.replace(".png", "_yolo.png")
                vision.save_image_safe(annotated_img, yolo_path)
                print(f"      [YOLO] 识别结果图已保存: {yolo_path}")

                # M4 状态判读逻辑
                state_info = vision.analyze_colors(img)
                status = "正常"
                detail_msg = ""

                if wp_name == "waypoint_04":  # cabinet_01 (判断绿灯)
                    if state_info['green_ratio'] > 0.002:
                        detail_msg = "电源指示灯亮起（绿色）"
                    else:
                        status = "异常"
                        detail_msg = "电源指示灯熄灭！"

                elif wp_name == "waypoint_05":  # cabinet_02
                    if state_info['red_ratio'] > 0.05:
                        status = "异常"
                        detail_msg = "仪表指针或指示灯呈红色！"
                    else:
                        detail_msg = "仪表无异常，无红色报警"

                elif wp_name == "waypoint_06":  # cabinet_03
                    if state_info['red_ratio'] > 0.002:
                        status = "异常"
                        detail_msg = "检测到红色告警灯亮起！"
                    else:
                        detail_msg = "无红色告警"

                elif wp_name == "waypoint_07":  # transformer
                    if state_info['red_ratio'] > 0.005:
                        status = "异常"
                        detail_msg = "变压器检测到过热/火情/红色故障灯！"
                    elif state_info['green_ratio'] < 0.01 and state_info['yellow_ratio'] < 0.01:
                        status = "异常"
                        detail_msg = "变压器外观异常（绿色外壳或黄色警示标识缺失）！"
                    else:
                        status = "正常"
                        detail_msg = "变压器正常（绿色外观完整，黄色标识清晰）"

                log_entry = {
                    "point": wp_name,
                    "target": face_obj_name.split('/')[-1],
                    "detected_objects": " | ".join(obj_names),
                    "status": status,
                    "detail": detail_msg,
                    "red_ratio": state_info['red_ratio'],
                    "green_ratio": state_info['green_ratio'],
                    "yellow_ratio": state_info['yellow_ratio'],
                    "ocr_text": "",
                    "image_path": shot_path,
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
                }

                # OCR
                texts = vision.read_numbers(img)
                if texts:
                    log_entry["ocr_text"] = " | ".join(texts)
                    print(f"      [OCR] 识别到文字: {texts}")
                else:
                    print(f"      [OCR] 未识别到文字")

                status_log.append(log_entry)

                if status == "异常":
                    anomaly_count += 1
                    print("\n" + "=" * 50)
                    print(f"      ！！【M5异常警报】！！")
                    print(f"      点位: {wp_name}，设备: {face_obj_name}")
                    print(f"      异常详情: {detail_msg}")
                    print("=" * 50 + "\n")

                    alarm_name = f"异常_{face_obj_name.split('/')[-1]}.png"
                    alarm_path = os.path.join(alarms_dir, alarm_name)
                    vision.save_image_safe(cv2.imread(shot_path), alarm_path)
                    print(f"      [M5] 异常现场已归档至: {alarm_path}")
                else:
                    print(f"      [状态判读] {detail_msg} -> 状态：正常")

            else:
                print(f"      [警告] 找不到对象 {face_obj_name}，无法拍照。")

        if i < len(waypoint_order) - 1:
            next_name = waypoint_order[i + 1]
            next_pos = points_coords[next_name]

            dx = next_pos[0] - cur_pos[0]
            dy = next_pos[1] - cur_pos[1]
            target_yaw = math.atan2(dy, dx)

            print(f"      原地旋转（旋向）对准 {next_name} ...")
            robot.go_to_point(robot_base, cur_pos, target_yaw=target_yaw)
            robot.stop()
            time.sleep(TURN_DURATION)

    print("      全部点位行驶完毕，任务完成。")

    # ================= M7：完整结果输出 (JSON + CSV) =================
    result = {
        "task": "M1-M5+M7 全流程完整巡检",
        "waypoints_followed": waypoint_order,
        "state_log": status_log,
        "anomaly_count": anomaly_count,
        "final_stop_reached": True,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    # 保存 JSON
    with open(config.RESULT_JSON, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(f"      [M7] 结果已保存: {config.RESULT_JSON}")

    # 保存 CSV (供 Excel 直接查看)
    try:
        with open(config.RESULT_CSV, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.writer(f)
            writer.writerow(["巡检点位", "目标设备", "识别物体", "状态", "异常描述",
                             "红色比例", "绿色比例", "黄色比例", "OCR文字", "截图路径", "时间戳"])
            for row in status_log:
                writer.writerow([row["point"], row["target"], row["detected_objects"],
                                 row["status"], row["detail"], row["red_ratio"],
                                 row["green_ratio"], row["yellow_ratio"], row["ocr_text"],
                                 row["image_path"], row["timestamp"]])
        print(f"      [M7] 数据表已保存: {config.RESULT_CSV}")
    except Exception as e:
        print(f"      [警告] 保存 CSV 失败: {e}")

    print("\n" + "=" * 40)
    print("      巡检任务全部完成！")
    print(f"      发现异常数: {anomaly_count}")
    print("=" * 40 + "\n")

    print("\n按 Ctrl+C 退出。")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n运行异常: {e}")