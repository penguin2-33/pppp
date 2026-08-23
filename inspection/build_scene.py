# -*- coding: utf-8 -*-
"""配电房场景生成脚本（程序化搭建，保证可复现）

用法：
  1. 打开 CoppeliaSim 4.10.0 (Edu)，保持运行。
  2. 运行：python build_scene.py
  3. 脚本会先清空当前场景，再自动创建 机器人 + 3点位 + 6类目标 + 2障碍物，
     并保存为 .ttt。可重复运行（幂等，自动清场）。

本版（v2）相对 v1 的关键修复与扩展：
  - VisionSensor 位置/朝向修正（相对机器人 [0.15, 0, 0.30]，绕 Y 轴 +π/2，
    真正朝机器人 +X 前进方向）。原版 [0, -π/2, 0] 实际是朝后下。
  - 仪表指针改为表盘的子对象（避免变成孤儿导致 setObjectOrientation 失效）。
  - 4 类目标细节补全：
      配电柜：加前面板 + 浅色编号牌（OCR 用）
      仪表：加刻度圆环 + 旋转指针（可独立转动，便于异常模拟）
      警示标识：加黑黄条纹边框（提升 YOLO 识别特征）
      开关/旋钮：加指示线（ON/OFF 位置判读）
  - 清理残留 box / Floor 父子关系。

已按本机实际 API 核对（4.10.0）：
  - 形状：sim.createPrimitiveShape(primitiveshape_cuboid/spheroid/cylinder, size, 0)
    尺寸为全尺寸（球=直径）
  - 上色：sim.setShapeColor(h, "", sim.colorcomponent_ambient_diffuse, [r,g,b])
  - 碰撞/静态：sim.setObjectInt32Param(h, sim.shapeintparam_respondable/static, 1)
  - 模型路径：robots/mobile/pioneer p3dx.ttm / components/sensors/Hokuyo URG 04LX UG01_Fast.ttm
  - 相机/接近传感器：sim.createVisionSensor / sim.createProximitySensor
  - 视觉抓帧前置：需先 sim.handleVisionSensor(h) 触发一帧（显式处理模式）
"""
import math
import os

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ==================== 配置区 ====================
COPPELIA_PATH = r"C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu"
# 注意：CoppeliaSim 的 saveScene 在中文 Windows 上会把非 ASCII 路径按 GBK 误解，
# 导致中文路径变乱码。故保存到纯英文路径，再用 PowerShell 复制到中文项目目录。
SCENE_SAVE_PATH = r"D:\inspection_scene.ttt"

# ==================== 布局常量（可复现：所有坐标集中在此） ====================
# 设备沿 X 轴排成一排（Y=0，靠墙），机器人在 Y=-1.0 的通道行进，+X 为机器人前进方向
# 机器人在 waypoint 处停靠后，整体转向设备侧（−Y 反方向，即朝 +Y 看向设备）
LAYOUT = {
    # 房间：地面 12m(X)×6m(Y)，X∈[-4,8]，Y∈[-3,3]；设备靠后墙(y=2.6)，机器人沿 y=1.0 通道行进
    "robot_start": (-2.0, 1.0, 0.2),   # 抬高起始高度，避免初始嵌入地板被物理引擎弹穿
    "waypoints": {
        "waypoint_01": (0.0, 1.0, 0.0),    # 看 cabinet_01（按钮/开关）
        "waypoint_02": (1.6, 1.0, 0.0),    # 看 cabinet_02（仪表）
        "waypoint_03": (3.2, 1.0, 0.0),    # 看 cabinet_03（指示灯）
    },
    # 3 个配电柜主体（靠房间后墙，柜体中心 y=2.6）
    "cabinets": {
        "cabinet_01": (0.0, 2.6, 0.8),    # 进线柜：编号牌 + 按钮 + 开关
        "cabinet_02": (1.6, 2.6, 0.8),    # 仪表柜：编号牌 + 指针仪表
        "cabinet_03": (3.2, 2.6, 0.8),    # 指示柜：编号牌 + 3绿灯 + 红灯
    },
    # 变压器（靠后墙，大设备）
    "transformer": (5.3, 2.6, 0.7),
    # 母线桥架（天花板下，沿设备方向）
    "bus_duct": (2.75, 2.6, 2.35),
    # 灭火器（通道远侧靠墙，2 个红色罐体）
    "fire_extinguishers": [
        (-2.5, 0.0, 0.2),
        (6.5, 0.0, 0.2),
    ],
    # 安全标识（挂墙，多种类型）
    "signs": {
        "sign_01":         (1.0, 2.85, 2.1),   # 黄色警示（黑边斜杠）
        "sign_highvoltage":(2.0, 2.85, 2.1),   # 高压危险（黄底黑闪电）
        "sign_no_fire":    (3.0, 2.85, 2.1),   # 禁止烟火（白底红圈斜杠）
        "sign_ground":     (4.0, 2.85, 2.1),   # 必须接地（蓝底）
        "sign_exit":       (7.85, 0.5, 2.1),   # 安全出口（绿底，右墙内侧）
    },
    "obstacles": {
        "obstacle_01": (2.4, 1.0, 0.25),   # 静态障碍（通道上，需绕行）
        "obstacle_02": (6.8, 0.3, 0.25),   # 静态障碍（备用）
    },
    # 房间结构（后墙 + 左右墙，正面敞开便于展示；墙厚 0.1、高 2.8）
    "room": {
        "back_wall":    {"pos": (2.0, 2.95, 1.4), "size": (12.0, 0.1, 2.8)},
        "left_wall":    {"pos": (-3.95, 0.0, 1.4), "size": (0.1, 6.0, 2.8)},
        "right_wall":   {"pos": (7.95, 0.0, 1.4), "size": (0.1, 6.0, 2.8)},
    },
    # 地板：12m × 6m，覆盖整个房间
    "floor": {
        "pos": (2.0, 0.0, -0.05),          # 地板中心（z=-0.05 使顶面在 z=0）
        "size": (12.0, 6.0, 0.1),          # 长(X) 12m × 宽(Y) 6m × 厚 0.1m
    },
    # 通道边界黄线（巡检通道：y=1.6 近设备 / y=0.3 远侧）
    "corridor_lines": [
        {"name": "corridor_line_near", "pos": (2.0, 1.6, 0.005), "size": (8.5, 0.05, 0.01)},
        {"name": "corridor_line_far",  "pos": (2.0, 0.3, 0.005), "size": (8.5, 0.05, 0.01)},
    ],
    # 黄黑警示带（设备前地面，交替黄黑段）
    "warning_strip": {"y": 2.25, "x_start": -0.6, "x_end": 6.2, "width": 0.14},
    # 绝缘垫（设备前操作区，绿色橡胶垫）
    "insulation_mat": {"pos": (2.8, 1.85, 0.005), "size": (6.8, 0.6, 0.015)},
}

COLORS = {
    "cabinet":       (0.30, 0.30, 0.36),   # 深灰柜体
    "cabinet_panel": (0.65, 0.65, 0.70),   # 浅灰前面板
    "id_plate":      (0.92, 0.92, 0.88),   # 编号牌（接近白色，便于 OCR）
    "dial":          (0.95, 0.95, 0.95),   # 表盘白
    "dial_mark":     (0.10, 0.10, 0.10),   # 刻度黑
    "needle":        (0.90, 0.10, 0.10),   # 指针红
    "green":         (0.00, 0.80, 0.10),
    "red":           (0.90, 0.05, 0.05),
    "sign_yellow":   (0.95, 0.85, 0.10),   # 警示牌黄底
    "sign_black":    (0.05, 0.05, 0.05),   # 警示牌黑边
    "switch":        (0.20, 0.20, 0.25),   # 开关主体深色
    "switch_indicator": (0.95, 0.95, 0.10), # 开关指示线亮黄
    "obstacle":      (0.20, 0.45, 0.80),
    "wall":          (0.72, 0.72, 0.75),
    "floor":         (0.70, 0.70, 0.73),   # 水泥地面
    "lamp_off":      (0.22, 0.22, 0.25),   # 熄灭的指示灯（暗色灯罩，状态判读用）
    "door_seam":     (0.12, 0.12, 0.15),   # 柜门缝
    "handle":        (0.70, 0.70, 0.74),   # 柜门把手（金属）
    "grille":        (0.10, 0.10, 0.12),   # 散热格栅
    "yellow_line":   (0.90, 0.82, 0.10),   # 通道边界黄线
    # v4 新增：房间/设备/地面
    "transformer":   (0.22, 0.36, 0.28),   # 变压器深绿
    "metal":         (0.62, 0.62, 0.66),   # 金属（母线桥架）
    "fire_ext":      (0.85, 0.10, 0.08),   # 灭火器红
    "insulation":    (0.15, 0.45, 0.20),   # 绝缘垫深绿
    "window":        (0.10, 0.16, 0.28),   # 柜门观察窗玻璃（深蓝）
    "door_color":    (0.45, 0.38, 0.30),   # 房门（深棕）
    "sign_blue":     (0.10, 0.30, 0.70),   # 必须接地标识蓝底
    "sign_exit_green": (0.10, 0.60, 0.25), # 安全出口标识绿底
    "white":         (0.95, 0.95, 0.95),   # 白
}


def main():
    print("=" * 60)
    print("配电房场景生成脚本 v2")
    print("=" * 60)
    print("前提：CoppeliaSim 已打开。脚本会自动清空当前场景后重建。\n")

    client = RemoteAPIClient()
    sim = client.require("sim")
    print("[连接] 成功\n")

    # 清空当前场景（幂等：可重复运行；不用 closeScene，避免 Edu 版弹注册窗）
    def clear_scene():
        # 注：Floor/box 也从 keep 中移除，改为自建大地板（默认 Floor 仅 5×5m，太小，
        # 机器人驶到 x>2.5 会坠落）。故清场时一并删除。
        keep = {"DefaultCamera", "DefaultLights", "DefaultLightA", "DefaultLightB",
                "DefaultLightC", "DefaultLightD", "XYZCameraProxy",
                "DefaultNXViewCamera", "DefaultNYViewCamera", "DefaultNZViewCamera",
                "DefaultXViewCamera", "DefaultYViewCamera", "DefaultZViewCamera"}
        all_objs = sim.getObjectsInTree(sim.handle_scene, sim.handle_all)
        to_remove = [o for o in all_objs if sim.getObjectName(o) not in keep]
        # 深度优先（先删子对象再删父对象），避免残留孤儿
        def depth(o):
            d = 0
            while sim.getObjectParent(o) != -1:
                o = sim.getObjectParent(o)
                d += 1
            return d
        to_remove.sort(key=depth, reverse=True)
        if to_remove:
            sim.removeObjects(to_remove)
        return len(to_remove)

    try:
        n = clear_scene()
        print(f"[清场] 已移除 {n} 个残留对象\n")
    except Exception as e:
        print(f"[警告] 清场失败（继续，可能残留对象）: {e}\n")

    # 形状类型常量（新 API createPrimitiveShape 使用）
    CUBOID = getattr(sim, "primitiveshape_cuboid", 3)
    SPHERE = getattr(sim, "primitiveshape_spheroid", 4)
    CYLINDER = getattr(sim, "primitiveshape_cylinder", 5)
    # 圆环/环面（用于仪表刻度盘）
    try:
        DISC = getattr(sim, "primitiveshape_disc", 6)
    except Exception:
        DISC = CYLINDER  # 退化到圆柱

    manual_todo = []

    # ---------- 工具函数 ----------
    def set_name_alias(h, name):
        """同时设置 name 与 alias（sim.getObject('/路径') 依赖 alias 解析）"""
        sim.setObjectName(h, name)
        sim.setObjectAlias(h, name)

    def load_model(rel_path, name):
        full = os.path.join(COPPELIA_PATH, "models", rel_path)
        if not os.path.exists(full):
            print(f"  [警告] 模型文件不存在: {full}")
            manual_todo.append(name)
            return None
        try:
            h = sim.loadModel(full)
            set_name_alias(h, name)
            print(f"  [OK] 加载 {name}")
            return h
        except Exception as e:
            print(f"  [警告] 加载 {name} 失败: {e}")
            manual_todo.append(name)
            return None

    def rename_child(root, keyword, new_name):
        """在模型树中按关键词找子对象并重命名"""
        try:
            objs = sim.getObjectsInTree(root, sim.handle_all)
        except Exception:
            objs = []
        for o in objs:
            try:
                if keyword.lower() in sim.getObjectName(o).lower():
                    set_name_alias(o, new_name)
                    return o
            except Exception:
                continue
        return None

    def make_shape(ptype, name, pos, size, color, respondable=True, parent=-1):
        h = sim.createPrimitiveShape(ptype, size, 0)
        set_name_alias(h, name)
        if parent != -1:
            sim.setObjectParent(h, parent, True)
        # parent=-1 用世界坐标；parent=对象则用相对坐标
        sim.setObjectPosition(h, parent, list(pos))
        sim.setShapeColor(h, "", sim.colorcomponent_ambient_diffuse, list(color))
        if respondable:
            sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 1)
            sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
        return h

    def make_cuboid(name, pos, size, color, respondable=True, parent=-1):
        return make_shape(CUBOID, name, pos, size, color, respondable, parent)

    def make_cylinder(name, pos, size, color, respondable=True, parent=-1):
        return make_shape(CYLINDER, name, pos, size, color, respondable, parent)

    def make_sphere(name, pos, diameter, color, respondable=True, parent=-1):
        return make_shape(SPHERE, name, pos, [diameter, diameter, diameter], color, respondable, parent)

    def make_dummy(name, pos, parent=-1):
        h = sim.createDummy(0.05, None)
        set_name_alias(h, name)
        if parent != -1:
            sim.setObjectParent(h, parent, True)
        sim.setObjectPosition(h, parent, list(pos))
        return h

    # ============ 1. 机器人 P3-DX ============
    print("[机器人]")
    p3dx = load_model(os.path.join("robots", "mobile", "pioneer p3dx.ttm"), "P3DX")
    if p3dx:
        sim.setObjectPosition(p3dx, -1, list(LAYOUT["robot_start"]))
        l = rename_child(p3dx, "left", "leftMotor")
        r = rename_child(p3dx, "right", "rightMotor")
        if l is None or r is None:
            print("  [警告] 未自动识别左右轮关节，请手动重命名为 leftMotor/rightMotor")
            manual_todo.append("P3DX 左右轮关节重命名")

    # ============ 2. 激光雷达 Hokuyo ============
    print("[传感器]")
    hokuyo = load_model(os.path.join("components", "sensors", "Hokuyo URG 04LX UG01_Fast.ttm"), "Hokuyo")
    if hokuyo and p3dx:
        sim.setObjectParent(hokuyo, p3dx, False)
        sim.setObjectPosition(hokuyo, p3dx, [0.0, 0.0, 0.25])

    # ============ 3. 视觉传感器（API 创建透视相机） ============
    # options=3: bit0(显式处理) + bit1(透视模式)
    # intParams: [分辨率X, 分辨率Y, 0, 0]
    # floatParams: [近裁剪面, 远裁剪面, 视角(deg), 传感器尺寸X, 0,0, null像素RGB, 0,0]
    # 修正：v1 的朝向 [0, -π/2, 0] 实际让相机朝机器人 -X 后方；改为 [0, +π/2, 0] 朝前。
    try:
        vis = sim.createVisionSensor(
            3, [640, 480, 0, 0],
            [0.01, 3.0, 60.0, 0.05, 0.0, 0.0, 0.5, 0.5, 0.5, 0.0, 0.0])
        set_name_alias(vis, "VisionSensor")
        if p3dx:
            sim.setObjectParent(vis, p3dx, False)
            # 前方 0.15、高 0.30（相对 P3-DX 模型原点的轮子底部）
            sim.setObjectPosition(vis, p3dx, [0.15, 0.0, 0.30])
            # 相机默认观察方向是局部 +Z；绕 Y 转 +π/2 把 +Z 旋转到 +X（机器人前方）
            sim.setObjectOrientation(vis, p3dx, [0.0, math.pi / 2.0, 0.0])
        print("  [OK] 创建 VisionSensor（透视相机 640x480，朝前 +X）")
    except Exception as e:
        print(f"  [警告] 创建 VisionSensor 失败: {e}")
        manual_todo.append("VisionSensor（菜单 Add > Vision sensor > Perspective type）")

    # ============ 4. 接近传感器（API 创建锥形） ============
    try:
        prox = sim.createProximitySensor(
            sim.proximitysensor_cone, 16, 1,
            [8, 8, 8, 8, 0, 0, 0, 0],
            [0.0, 0.4, 0.05, 0.05, 0.1, 0.1, 0.0, 0.05, 0.1, 30.0, 0.0, 0.0, 0.02, 0.0, 0.0])
        set_name_alias(prox, "proxFront")
        if p3dx:
            sim.setObjectParent(prox, p3dx, False)
            sim.setObjectPosition(prox, p3dx, [0.25, 0.0, 0.15])
            # 接近传感器默认观察 -Z 轴；绕 Y +π/2 让它指向机器人 +X 前进方向
            sim.setObjectOrientation(prox, p3dx, [0.0, math.pi / 2.0, 0.0])
        print("  [OK] 创建 proxFront（锥形接近传感器，检测 0.4m）")
    except Exception as e:
        print(f"  [警告] 创建 proxFront 失败: {e}")
        manual_todo.append("proxFront（菜单 Add > Proximity sensor > Cone type）")

    # ============ 5. 房间结构：地板 + 三面墙（后+左右，正面敞开）+ 地面细节 ============
    print("[房间]")
    # 地板（12m × 6m，覆盖整个房间）
    fl = LAYOUT["floor"]
    make_cuboid("floor", fl["pos"], fl["size"], COLORS["floor"])
    # 三面墙（respondable，机器人不能穿墙）
    for wname, w in LAYOUT["room"].items():
        make_cuboid(wname, w["pos"], w["size"], COLORS["wall"])
    # 通道边界黄线（视觉标线，非碰撞体，避免机器人撞线）
    print("[通道标线]")
    for ln in LAYOUT["corridor_lines"]:
        make_cuboid(ln["name"], ln["pos"], ln["size"], COLORS["yellow_line"], respondable=False)
    # 黄黑警示带（设备前地面，交替黄黑段）
    ws = LAYOUT["warning_strip"]
    seg = 0.3
    n = int((ws["x_end"] - ws["x_start"]) / seg) + 1
    for i in range(n):
        cx = ws["x_start"] + i * seg + seg / 2.0
        color = COLORS["yellow_line"] if i % 2 == 0 else COLORS["sign_black"]
        make_cuboid(f"warnstrip_{i}", [cx, ws["y"], 0.005], [seg, ws["width"], 0.01], color, respondable=False)
    # 绝缘垫（设备前操作区，绿色橡胶垫）
    im = LAYOUT["insulation_mat"]
    make_cuboid("insulation_mat", im["pos"], im["size"], COLORS["insulation"], respondable=False)

    # ============ 6. 巡检点位 ============
    print("[巡检点位]")
    for name, pos in LAYOUT["waypoints"].items():
        make_dummy(name, pos)

    # ============ 7. 目标对象 ============
    # 核心原则：仪表/指示灯/开关都是「配电柜前面板的安装件」（柜体的子对象），
    # 柜体才是主体；警示标识单独挂背墙。符合真实配电房布局。
    print("[目标对象]")

    # 柜体前面板相对柜体中心的偏移：柜深 0.4，-y 面在 y=-0.2，面板再突出 0.005
    PANEL_OFF = -0.205   # 面板中心相对柜体中心的 y 偏移（面板厚 0.01，表面在 y≈-0.21）
    FACE_OFF = -0.215    # 安装件表面相对柜体中心的 y（贴到面板外表面外一点）

    # 7.1 配电柜 cabinet_01~03（柜体 + 前面板 + 编号牌）
    cab_handles = {}
    for idx, name in enumerate(["cabinet_01", "cabinet_02", "cabinet_03"], start=1):
        pos = LAYOUT["cabinets"][name]
        # 主体：深灰柜体 0.8(宽) x 0.4(深) x 1.6(高)
        cab = make_cuboid(name, pos, [0.8, 0.4, 1.6], COLORS["cabinet"])
        cab_handles[name] = cab
        # 前面板：略小、浅灰，贴在 -y 一面（朝通道）
        panel = make_cuboid(name + "_panel", [0.0, PANEL_OFF, 0.0], [0.74, 0.01, 1.4], COLORS["cabinet_panel"],
                            respondable=False, parent=cab)
        # 编号牌：浅色小方块，挂在前面板中央偏上（面板本地 +z 0.3 处）
        plate = make_cuboid(name + "_idplate", [0.0, FACE_OFF - PANEL_OFF, 0.3], [0.5, 0.005, 0.20], COLORS["id_plate"],
                            respondable=False, parent=panel)
        # 编号牌上的"数字"——用 2 个深色小方块模拟两位数字（01/02/03）
        for k in range(2):
            digit_pos = [-0.10 + k * 0.20, -0.003, 0.0]
            make_cuboid(name + f"_d{k+1}", digit_pos, [0.14, 0.005, 0.14], COLORS["dial_mark"],
                        respondable=False, parent=plate)

        # 柜门细节（提升真实感）：门缝 + 把手 + 顶部散热格栅 + 柜顶警示条
        # 门缝：右侧竖直深色细线（单开门右门缝）
        make_cuboid(name + "_seam", [0.30, -0.007, -0.05], [0.006, 0.002, 0.9],
                    COLORS["door_seam"], respondable=False, parent=panel)
        # 把手：左侧竖直金属拉手
        make_cuboid(name + "_handle", [-0.28, -0.03, -0.15], [0.02, 0.04, 0.12],
                    COLORS["handle"], respondable=False, parent=panel)
        # 散热格栅：顶部 3 条水平深色缝
        for gi in range(3):
            make_cuboid(name + f"_grille_{gi}", [0.0, -0.007, 0.50 + gi * 0.055],
                        [0.34, 0.002, 0.015], COLORS["grille"], respondable=False, parent=panel)
        # 柜顶警示条：顶部一条黄黑相间色带
        make_cuboid(name + "_warnstripe", [0.0, -0.007, 0.67], [0.74, 0.002, 0.03],
                    COLORS["yellow_line"], respondable=False, parent=panel)
        # 柜门观察窗：深色玻璃（编号牌下方，中上部）
        make_cuboid(name + "_window", [0.0, -0.008, 0.0], [0.5, 0.002, 0.18],
                    COLORS["window"], respondable=False, parent=panel)

    # 7.2 指针仪表 meter_01 —— 装在 cabinet_02 面板中央（电压表/电流表样式）
    # 简化设计：白色表盘（dial）+ 12 个深色刻度块（直接贴表面）+ 红色指针 + 黑轴帽
    cab2 = cab_handles["cabinet_02"]
    dial = make_cylinder("meter_01", [0.0, FACE_OFF, 0.0], [0.18, 0.18, 0.03], COLORS["dial"],
                         respondable=False, parent=cab2)
    sim.setObjectOrientation(dial, cab2, [math.pi / 2.0, 0.0, 0.0])
    # 弧形刻度：-135° ~ +135°（270° 量程），每 22.5° 一根；主刻度（每 45°）更长，
    # 作为读数锚点（数字刻度值在 M5 读数算法里用 角度→数值 映射，表盘太小不宜放字块）
    for i in range(13):
        ang = math.radians(-135.0 + i * 22.5)
        is_major = (i % 2 == 0)
        r_mid = 0.065
        dx = r_mid * math.sin(ang)   # dial 本地 +X（旋转后 = 世界 +X）
        dy = r_mid * math.cos(ang)   # dial 本地 +Y（旋转后 = 世界 +Z 向上）
        tick_len = 0.034 if is_major else 0.022
        tick_w = 0.012 if is_major else 0.008
        make_cuboid(f"meter_01_tick_{i}",
                    [dx, dy, 0.020],                 # 突出 dial 朝通道表面 0.005m
                    [tick_w, tick_len, 0.006],
                    COLORS["dial_mark"], respondable=False, parent=dial)
    # 指针：用 dummy 枢轴挂在表盘中心，保证指针绕中心旋转（而非绕自身中心）
    pivot = make_dummy("meter_01_pivot", [0.0, 0.0, 0.022], parent=dial)
    make_cuboid("meter_01_needle", [0.0, 0.04, 0.0], [0.012, 0.08, 0.006], COLORS["needle"],
                respondable=False, parent=pivot)
    # 异常「仪表超限」：绕枢轴转 +150°（超出量程最大刻度 +135°）
    sim.setObjectOrientation(pivot, dial, [0.0, 0.0, math.radians(150.0)])
    # 中心轴帽
    make_sphere("meter_01_axis", [0.0, 0.0, 0.025], 0.020, COLORS["cabinet"],
                respondable=False, parent=dial)

    # 7.3 指示灯 —— 分布在 3 个柜子，用「真实灯光组件」模拟亮灭
    # 可见灯罩球体 + 复制默认点光源（亮=发光，灭=不发光）；自发光让灯球"点亮"
    light_template = None
    for o in sim.getObjectsInTree(sim.handle_scene, sim.handle_all):
        if sim.getObjectName(o) == "DefaultLightA":
            light_template = o
            break

    def make_lamp(cab, name, lx, lz, color, lit):
        """在柜子面板装一个指示灯：底座 + 灯球(亮则自发光) + 低强度点光源"""
        ly = FACE_OFF
        make_cylinder(name + "_base", [lx, ly, lz], [0.05, 0.05, 0.02], COLORS["switch"],
                      respondable=False, parent=cab)
        bulb = make_sphere(name, [lx, ly - 0.035, lz], 0.08, color,
                           respondable=False, parent=cab)
        if lit:
            # 自发光（emission）：让灯球本身看起来"点亮"，不产生全局光照
            try:
                sim.setShapeColor(bulb, "", getattr(sim, "colorcomponent_emission", 2), list(color))
            except Exception:
                pass
            if light_template is not None:
                try:
                    lh = sim.copyPasteObjects([light_template], 0)[0]
                    set_name_alias(lh, name + "_light")
                    sim.setObjectParent(lh, cab, True)
                    sim.setObjectPosition(lh, cab, [lx, ly - 0.06, lz])
                    dim = [c * 0.06 for c in color]   # 强度降到 6%
                    sim.setLightParameters(lh, 1, None, dim, dim)
                except Exception as e:
                    print(f"  [警告] 创建 {name} 光源失败: {e}")

    # 各柜指示灯布局：
    #   cabinet_01 电源指示(亮) / cabinet_02 工作指示(亮)
    #   cabinet_03 回路运行(灭，状态判读) + 故障告警(亮，异常)
    make_lamp(cab_handles["cabinet_01"], "lamp_green_1", 0.0, 0.45, COLORS["green"], True)
    make_lamp(cab_handles["cabinet_02"], "lamp_green_2", 0.0, 0.45, COLORS["green"], True)
    make_lamp(cab_handles["cabinet_03"], "lamp_green_3", -0.12, 0.45, COLORS["lamp_off"], False)
    make_lamp(cab_handles["cabinet_03"], "lamp_red", 0.12, 0.45, COLORS["red"], True)

    # 7.4 按钮 —— 装在 cabinet_01 面板上（绿色"启动" + 红色"停止"）
    cab1 = cab_handles["cabinet_01"]
    btn_y = FACE_OFF
    # 绿色启动按钮（左）
    make_cylinder("btn_start", [-0.15, btn_y, -0.3], [0.07, 0.07, 0.03], COLORS["green"],
                  respondable=False, parent=cab1)
    # 红色停止按钮（右）
    make_cylinder("btn_stop", [0.15, btn_y, -0.3], [0.07, 0.07, 0.03], COLORS["red"],
                  respondable=False, parent=cab1)
    # 旋钮开关 switch_01 —— 装在 cabinet_01 面板中间（旋转式切换开关）
    sw = make_cuboid("switch_01", [0.0, FACE_OFF, -0.05], [0.08, 0.08, 0.10], COLORS["switch"],
                     respondable=False, parent=cab1)
    # 指示线：亮黄，从旋钮中心指向左 = OFF 位置（状态判读用）
    make_cuboid("switch_01_indicator", [-0.03, -0.05, 0.0], [0.06, 0.004, 0.015], COLORS["switch_indicator"],
                respondable=False, parent=sw)

    # 7.5 安全标识阵列 —— 挂墙，多种类型（黄警示/高压/禁止烟火/接地/安全出口）
    print("[安全标识]")
    signs_cfg = LAYOUT["signs"]

    # 黄色警示牌 sign_01（黑边 + 斜杠）
    sp = signs_cfg["sign_01"]
    sign = make_cuboid("sign_01", sp, [0.30, 0.03, 0.30], COLORS["sign_yellow"])
    for label, relpos, size in [
        ("top",    [0.0,  -0.02,  0.13], [0.30, 0.005, 0.04]),
        ("bottom", [0.0,  -0.02, -0.13], [0.30, 0.005, 0.04]),
        ("left",   [-0.13, -0.02, 0.0],  [0.04, 0.005, 0.26]),
        ("right",  [ 0.13, -0.02, 0.0],  [0.04, 0.005, 0.26]),
    ]:
        make_cuboid(f"sign_01_border_{label}", relpos, size, COLORS["sign_black"],
                    respondable=False, parent=sign)
    slash = make_cuboid("sign_01_slash", [0.0, -0.02, 0.0], [0.30, 0.005, 0.04], COLORS["sign_black"],
                        respondable=False, parent=sign)
    sim.setObjectOrientation(slash, sign, [0.0, 0.0, math.pi / 4.0])

    # 高压危险 sign_highvoltage（黄底 + 黑色闪电）
    sp = signs_cfg["sign_highvoltage"]
    hv = make_cuboid("sign_highvoltage", sp, [0.30, 0.03, 0.30], COLORS["sign_yellow"])
    bolt = make_cuboid("sign_highvoltage_bolt", [0.0, -0.02, 0.0], [0.08, 0.005, 0.22], COLORS["sign_black"],
                       respondable=False, parent=hv)
    sim.setObjectOrientation(bolt, hv, [0.0, 0.0, math.pi / 4.0])

    # 禁止烟火 sign_no_fire（白底 + 红边框 + 红斜杠）
    sp = signs_cfg["sign_no_fire"]
    nf = make_cuboid("sign_no_fire", sp, [0.30, 0.03, 0.30], COLORS["white"])
    for label, relpos, size in [
        ("top",    [0.0,  -0.02,  0.13], [0.30, 0.005, 0.03]),
        ("bottom", [0.0,  -0.02, -0.13], [0.30, 0.005, 0.03]),
        ("left",   [-0.13, -0.02, 0.0],  [0.03, 0.005, 0.26]),
        ("right",  [ 0.13, -0.02, 0.0],  [0.03, 0.005, 0.26]),
    ]:
        make_cuboid(f"sign_no_fire_border_{label}", relpos, size, COLORS["red"],
                    respondable=False, parent=nf)
    nf_slash = make_cuboid("sign_no_fire_slash", [0.0, -0.02, 0.0], [0.30, 0.005, 0.03], COLORS["red"],
                           respondable=False, parent=nf)
    sim.setObjectOrientation(nf_slash, nf, [0.0, 0.0, math.pi / 4.0])

    # 必须接地 sign_ground（蓝底 + 白色接地横线）
    sp = signs_cfg["sign_ground"]
    gd = make_cuboid("sign_ground", sp, [0.30, 0.03, 0.30], COLORS["sign_blue"])
    for gi, gz in enumerate([-0.08, 0.0, 0.08]):
        make_cuboid(f"sign_ground_line_{gi}", [0.0, -0.02, gz], [0.18, 0.005, 0.02], COLORS["white"],
                    respondable=False, parent=gd)

    # 安全出口 sign_exit（绿底 + 白色箭头，右墙内侧，绕 Z 转 90° 面向房间内）
    sp = signs_cfg["sign_exit"]
    ex = make_cuboid("sign_exit", sp, [0.34, 0.03, 0.16], COLORS["sign_exit_green"])
    sim.setObjectOrientation(ex, -1, [0.0, 0.0, math.pi / 2.0])
    make_cuboid("sign_exit_arrow", [0.06, -0.02, 0.0], [0.16, 0.005, 0.04], COLORS["white"],
                respondable=False, parent=ex)
    make_cuboid("sign_exit_arrow_head", [0.14, -0.02, 0.0], [0.05, 0.005, 0.05], COLORS["white"],
                respondable=False, parent=ex)

    # 7.6 典型设备扩充：变压器 + 母线桥架 + 灭火器
    print("[典型设备]")
    # 变压器（靠后墙大设备 + 散热鳍片 + 警示标签）
    tp = LAYOUT["transformer"]
    tf = make_cuboid("transformer", tp, [0.9, 0.6, 1.4], COLORS["transformer"])
    for fi in range(4):
        make_cuboid(f"transformer_fin_{fi}", [-0.3 + fi * 0.2, -0.31, 0.0], [0.03, 0.02, 1.2],
                    COLORS["metal"], respondable=False, parent=tf)
    make_cuboid("transformer_label", [0.0, -0.31, 0.4], [0.4, 0.005, 0.25], COLORS["sign_yellow"],
                respondable=False, parent=tf)
    # 母线桥架（天花板下金属槽）
    bd = LAYOUT["bus_duct"]
    make_cuboid("bus_duct", bd, [5.5, 0.3, 0.15], COLORS["metal"])
    # 灭火器（红色罐体 + 顶部喷口）
    for i, fe in enumerate(LAYOUT["fire_extinguishers"], start=1):
        make_cylinder(f"fire_ext_{i:02d}", fe, [0.12, 0.12, 0.4], COLORS["fire_ext"])
        make_cuboid(f"fire_ext_{i:02d}_nozzle", [fe[0], fe[1], fe[2] + 0.22], [0.05, 0.05, 0.06], COLORS["metal"])

    # ============ 8. 障碍物 ============
    print("[障碍物]")
    for name, pos in LAYOUT["obstacles"].items():
        make_cuboid(name, pos, [0.4, 0.3, 0.5], COLORS["obstacle"])

    # ============ 9. 保存场景 ============
    print("[保存]")
    try:
        sim.saveScene(SCENE_SAVE_PATH)
        print(f"  [OK] 场景已保存: {SCENE_SAVE_PATH}")
    except Exception as e:
        print(f"  [警告] 保存失败: {e}（可手动 File > Save scene as）")

    # ============ 10. 总结 ============
    print("\n" + "=" * 60)
    print("场景生成完成（v2）")
    print("=" * 60)
    if manual_todo:
        print("需手动补充：")
        for t in manual_todo:
            print(f"  - {t}")
    else:
        print("所有对象已自动生成。")
    print("\n下一步：运行 python inspect_scene.py 核对对象，再运行 python main.py 验证 M1。")
    print("注：M1 抓帧会调用 sim.handleVisionSensor() 显式触发一帧，避免 0 缓冲。")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n[异常] {e}")
        print("请检查：CoppeliaSim 是否打开、COPPELIA_PATH 是否正确。")
