# 工业配电房智能巡检 - 代码运行说明（M1）

## 一、环境准备

1. **CoppeliaSim 4.10.0**：已安装并打开配电房场景。
2. **Python 3.8+**。
3. 安装依赖：

   ```bash
   pip install -r requirements.txt
   ```

   > `coppeliasim-zmqremoteapi-client` 若 pip 安装失败，可从 CoppeliaSim 安装目录
   > `programming/zmqRemoteApi/clients/python/src/` 下拷贝 `coppeliasim_zmqremoteapi_client`
   > 文件夹到本项目 `inspection/` 目录即可。

## 二、场景生成（build_scene.py）

场景**由脚本程序化生成**（可复现，答辩亮点），无需手动搭：

```bash
# 前提：CoppeliaSim 已打开（空场景或任意场景均可，脚本会先自动清场）
python build_scene.py
```

脚本会自动：清场 → 加载 P3-DX 机器人 + Hokuyo 雷达 → 创建相机/接近传感器 →
建背墙、3 个巡检点位、6 类目标（柜×3、仪表、红绿灯、标识、开关）、2 个障碍物 →
保存场景。

- 保存路径：`D:\inspection_scene.ttt`（英文路径，规避 CoppeliaSim 中文路径乱码问题）
- 项目内已另存一份：`D:\具身智能\配电房巡检场景.ttt`
- 可重复运行（幂等，每次自动清场重建）

## 三、场景对象命名约定（build_scene.py 已自动设置）

脚本已按下表自动命名并设置 alias（`sim.getObject('/路径')` 依赖 alias）。若手动改场景，需保持命名一致：

| 对象 | 命名 | 说明 |
|---|---|---|
| 机器人底座 | `/P3DX` | Pioneer P3-DX |
| 左轮关节 | `/P3DX/leftMotor` | 差速左轮 |
| 右轮关节 | `/P3DX/rightMotor` | 差速右轮 |
| 视觉传感器 | `/P3DX/VisionSensor` | 云台相机（识别用） |
| 激光雷达 | `/P3DX/Hokuyo` | fast Hokuyo URG-04LX |
| 前方接近传感器 | `/P3DX/proxFront` | 急停保护 |
| 巡检点位1~3 | `/waypoint_01` ~ `/waypoint_03` | dummy 对象 |
| 配电柜 | `/cabinet_01` ~ `/cabinet_03` | 目标 |
| 指针仪表 | `/meter_01` | 目标 |
| 绿色指示灯 | `/lamp_green` | 目标（正常） |
| 红色告警灯 | `/lamp_red` | 目标（异常） |
| 警示标识 | `/sign_01` | 目标 |
| 开关/旋钮 | `/switch_01` | 目标 |
| 障碍物 | `/obstacle_01`、`/obstacle_02` | 静态 + 动态 |

## 四、运行步骤

1. 打开 CoppeliaSim，运行 `python build_scene.py` 生成场景（若已生成可跳过）。
2. 运行：

   ```bash
   python main.py
   ```

3. 观察终端输出：连接 → 启动仿真 → 环境检查报告 → 抓帧（保存
   `output/shots/m1_frame_test.png`）→ 导航到点位1 → 结果记录
   （`output/inspection_result.json`）。

### 辅助脚本

- `python inspect_scene.py`：转储当前场景对象树（核对命名/位置）。
- `python reset_state.py`：停止仿真并把机器人复位到起点。

> 注意：CoppeliaSim Edu 版会周期性弹出 "Registration" 注册窗，弹出时 ZMQ 会阻塞，
> 需手动关闭该弹窗后再运行脚本。

## 五、M1 验收标准

- [ ] 终端打印环境检查报告，全部 OK
- [ ] `output/shots/m1_frame_test.png` 是机器人相机视角画面
- [ ] 机器人自主驶向点位1并停靠
- [ ] `output/inspection_result.json` 记录成功

## 六、后续里程碑（框架已预留）

- M2 多点位巡检与停靠对准
- M3 YOLO 目标识别 + OCR 编号
- M4 状态判读（指示灯/开关/通道）
- M5 异常提示（仪表超限/红灯亮起）
- M6 障碍避让（LiDAR 绕行 + 急停）
- M7 完整结果输出（JSON/CSV/截图）+ 连续稳定运行
