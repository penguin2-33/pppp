# -*- coding: utf-8 -*-
"""传感器封装：Vision Sensor / LiDAR / 接近传感器

答辩亮点：三级感知分工——
  视觉(Vision Sensor) 负责"是什么"（语义识别），
  激光(LiDAR) 负责"多远"（精确测距），
  接近传感器 负责"最后急停防线"。
"""
import numpy as np
import cv2


# ---------------- 视觉传感器 ----------------
def grab_image(sim, vision_handle):
    """Vision Sensor 抓帧 -> RGB numpy 数组 (H, W, 3)

    关键：Vision Sensor 在「显式处理」模式下，必须先 sim.handleVisionSensor()
    主动触发渲染，否则 getVisionSensorCharImage 可能返回陈旧或全零缓冲。
    每次抓帧前都调用一次，保证拿到最新画面。
    """
    try:
        sim.handleVisionSensor(vision_handle)
    except Exception:
        # 退化处理：老版 API / 非显式模式下 handleVisionSensor 可能失败，
        # 不应阻塞抓帧，忽略异常
        pass
    img, resX, resY = sim.getVisionSensorCharImage(vision_handle)
    arr = np.frombuffer(bytes(img), dtype=np.uint8).reshape(resY, resX, 3)
    return arr  # RGB 顺序


def save_image(sim, vision_handle, path):
    """抓帧并保存为图片文件（用于结果记录 / 异常现场截图）"""
    arr = grab_image(sim, vision_handle)
    bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    ok = cv2.imwrite(path, bgr)
    if not ok:
        # 中文路径下 cv2.imwrite 在某些 OpenCV 版本会静默失败，
        # 此时降级用 numpy + 二进制写入
        import numpy as _np
        # 再次尝试：先写到 tmp，再移动
        import os, tempfile
        tmp = tempfile.NamedTemporaryFile(suffix=".png", delete=False).name
        cv2.imwrite(tmp, bgr)
        try:
            import shutil
            shutil.move(tmp, path)
            print(f"  [信息] 经临时文件写入成功: {path}")
        except Exception as e:
            print(f"  [警告] 写入 {path} 失败: {e}；像素已抓取但未落盘")
    return arr


# ---------------- 激光雷达 ----------------
def read_lidar(sim, lidar_handle):
    """读取 LiDAR 点云 -> numpy 数组 (N, 3)，单位米。

    注：不同 CoppeliaSim 版本激光读取接口可能为 handleLidar / readLidar，
    此处做兼容处理，按实际版本二选一。
    """
    try:
        pts = sim.handleLidar(lidar_handle)
    except Exception:
        pts = sim.readLidar(lidar_handle)
    return np.asarray(pts, dtype=np.float32)


def lidar_front_distance(sim, lidar_handle, fov_deg=30.0):
    """从 LiDAR 数据计算正前方 ±fov/2 扇区内最近障碍距离(米)"""
    pts = read_lidar(sim, lidar_handle)
    if pts.size == 0:
        return float("inf")
    angles = np.arctan2(pts[:, 1], pts[:, 0])
    half = np.deg2rad(fov_deg / 2.0)
    front = pts[np.abs(angles) < half]
    if front.size == 0:
        return float("inf")
    dists = np.linalg.norm(front[:, :2], axis=1)
    return float(np.min(dists))


# ---------------- 接近传感器 ----------------
def read_proximity(sim, prox_handle):
    """读接近传感器，返回 (是否检测到, 最近距离)"""
    state, dist, *_ = sim.readProximitySensor(prox_handle)
    return bool(state), dist
