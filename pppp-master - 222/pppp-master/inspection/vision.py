# -*- coding: utf-8 -*-
"""视觉识别模块 (M3+M4 完整版)
M3: 目标检测 + OCR
M4: 状态判读 (颜色占比分析)
"""
import os
import numpy as np
import cv2

# 尝试导入库
try:
    from ultralytics import YOLO

    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

try:
    from paddleocr import PaddleOCR

    OCR_AVAILABLE = True
except ImportError:
    OCR_AVAILABLE = False

_yolo_model = None
_ocr_model = None


# ================= M3：目标检测 =================
def detect_by_color(image_rgb):
    image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    detections = []

    # 1. 绿色 (绝缘垫/绿灯)
    mask_green = cv2.inRange(hsv, (35, 80, 80), (85, 255, 255))
    if np.count_nonzero(mask_green) > 2000:
        detections.append({"class": "green_area/green_light", "conf": 0.9, "bbox": [0, 0, 0, 0]})

    # 2. 红色 (灭火器/红灯/指针)
    mask_red1 = cv2.inRange(hsv, (0, 80, 80), (10, 255, 255))
    mask_red2 = cv2.inRange(hsv, (170, 80, 80), (180, 255, 255))
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    if np.count_nonzero(mask_red) > 500:
        detections.append({"class": "red_object/fire_ext/red_light", "conf": 0.9, "bbox": [0, 0, 0, 0]})

    # 3. 深灰色 (配电柜)
    mask_gray = cv2.inRange(hsv, (0, 0, 50), (180, 30, 120))
    if np.count_nonzero(mask_gray) > 5000:
        detections.append({"class": "cabinet_gray_box", "conf": 0.85, "bbox": [0, 0, 0, 0]})

    # 4. 深绿色 (变压器)
    mask_darkgreen = cv2.inRange(hsv, (35, 50, 30), (85, 255, 80))
    if np.count_nonzero(mask_darkgreen) > 5000:
        detections.append({"class": "transformer_green", "conf": 0.85, "bbox": [0, 0, 0, 0]})

    # 5. 黄色 (通道线/标识)
    mask_yellow = cv2.inRange(hsv, (20, 80, 80), (35, 255, 255))
    if np.count_nonzero(mask_yellow) > 2000:
        detections.append({"class": "yellow_line/sign", "conf": 0.9, "bbox": [0, 0, 0, 0]})

    return detections


def detect_objects(image_rgb):
    detections = detect_by_color(image_rgb)
    if len(detections) > 0:
        return detections
    model = get_yolo()
    if model is None: return []
    image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    results = model(image_bgr, verbose=False)
    for r in results:
        for box in r.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            xyxy = box.xyxy[0].cpu().numpy().astype(int)
            detections.append({"class": model.names[cls_id], "conf": round(conf, 2), "bbox": xyxy.tolist()})
    return detections


def get_yolo(model_path="yolov8n.pt"):
    global _yolo_model
    if not YOLO_AVAILABLE: return None
    if _yolo_model is None:
        _yolo_model = YOLO(model_path)
    return _yolo_model


# ================= M3：OCR识别（已修复兼容） =================
def read_numbers(image_rgb):
    ocr = get_ocr()
    if ocr is None: return []
    image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    try:
        result = ocr.ocr(image_bgr)
    except Exception as e:
        # 忽略底层报错，防止崩溃
        return []
    texts = []
    if result and result[0]:
        for line in result[0]:
            try:
                texts.append(line[1][0])
            except Exception:
                continue
    return texts


def get_ocr():
    global _ocr_model
    if not OCR_AVAILABLE: return None
    if _ocr_model is None:
        try:
            _ocr_model = PaddleOCR(use_angle_cls=True, lang='en')
        except Exception:
            return None
    return _ocr_model


def draw_detections(image_rgb, detections):
    image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        if x1 == 0 and y1 == 0 and x2 == 0 and y2 == 0:
            cv2.putText(image_bgr, det["class"], (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            label = f"{det['class']} {det['conf']:.2f}"
            cv2.rectangle(image_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image_bgr, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)


def save_image_safe(image_bgr, path):
    try:
        cv2.imencode('.png', image_bgr)[1].tofile(path)
        return True
    except Exception as e:
        return False


# ================= M4：状态判读模块 =================
def analyze_colors(image_rgb):
    """分析图像中红、绿、黄的占比，用于状态判断"""
    image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

    # 红色掩码
    mask_red1 = cv2.inRange(hsv, (0, 80, 80), (10, 255, 255))
    mask_red2 = cv2.inRange(hsv, (170, 80, 80), (180, 255, 255))
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # 绿色掩码
    mask_green = cv2.inRange(hsv, (35, 80, 80), (85, 255, 255))

    # 黄色掩码
    mask_yellow = cv2.inRange(hsv, (20, 80, 80), (35, 255, 255))

    total_pixels = image_bgr.shape[0] * image_bgr.shape[1]
    if total_pixels == 0:
        return {"red_ratio": 0, "green_ratio": 0, "yellow_ratio": 0}

    red_ratio = np.count_nonzero(mask_red) / total_pixels
    green_ratio = np.count_nonzero(mask_green) / total_pixels
    yellow_ratio = np.count_nonzero(mask_yellow) / total_pixels

    return {
        "red_ratio": round(red_ratio, 3),
        "green_ratio": round(green_ratio, 3),
        "yellow_ratio": round(yellow_ratio, 3)
    }