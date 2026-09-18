# 正常停止返回 P0 与独立复位

本功能只用于正式 RTDE 扫描。模拟器不会连接或移动 UR7e。

## 状态流程

正常停止：

```text
任意扫描状态
→ safe_stop_motion
→ STOP_SCAN
→ 保存 P_stop、F/T 和扫描状态
→ RETURN_TO_START
→ 沿 UR Base +Z 相对抬升 0.030 m
→ 在安全高度移动到 P0 的 XY 上方并恢复 P0 姿态
→ 垂直下降到 P0
→ 检查位置和姿态误差
→ STOP
```

返回路径不会执行 `current_pose → P0` 的单段直线运动。
每一段进入容差后都会先停止并复查停稳位置，再允许开始下一段；独立手动返回还会在读取
P_stop 前强制尝试 `stopL` 和 `speedStop`，避免依赖新进程并不了解的旧运动状态。

紧急停止：

```text
ESC / Ctrl+C / RTDE failure / PX6D failure / protective stop / safety threshold
→ 立即停止
→ 保存当前数据、当前 TCP 和 P0
→ 不执行自动返回
```

## 记录初始标定点 P0

先在示教器中安全地把探针尖端 active TCP 放到扫描起点并保持静止，然后运行只读标定工具：

```bash
python3 run_calibration.py
```

程序显示当前 TCP pose/speed。只有输入：

```text
SAVE_P0
```

才会把六维 TCP pose 写入 `scan_calibration.yaml`。该工具不创建 RTDE 控制接口，不移动机器人。

## 必须现场填写的安全返回参数

`config.yaml`：

```yaml
safe_return:
  return_lift_distance: 0.030
  startup_bias_sample_count: 100
  return_speed: 0.015
  return_vertical_speed: 0.009
  return_acceleration: 0.030
  return_force_limit: 8.0
  return_torque_limit: 0.7
  return_position_tolerance: 0.0005
  return_orientation_tolerance: 0.0087
  return_segment_timeout_sec: 60.0
  auto_return_after_normal_stop: true
```

`return_lift_distance` 是沿 UR Base +Z 的相对抬升距离。程序在停止后计算
`safe_z = P_stop.z + 0.030`。当前 `workspace.enabled: false`，不检查软件 workspace；仍须确认
垂直退出、顶部横移和下降三段的整套工具与线缆扫掠空间安全。

如果返回中断时已经达到 `P0.z + 0.030` 或更高位置，下一次返回保持当前高度，不会重复累加
3 cm。

启动扫描命令触发的自动返回发生在正式 baseline 之前。程序会先在当前位置采集 100 个临时
静止偏置样本，以增量 processed force/torque 检查返回过程；到达 P0 后重新采集正式扫描
baseline。absolute raw F/T 后备阈值不会被临时偏置取消。

示例速度和阈值不是现场确认值。工程不使用布尔确认门锁，但返回参数仍必须是完整有效的数值。

## 正式扫描操作键

正式执行进入扫描循环后：

- `Q`：NORMAL STOP，停止扫描并尝试三段安全返回；
- `ESC`：EMERGENCY STOP，立即停止且不返回；
- `Ctrl+C`：EMERGENCY STOP，立即停止且不返回。

正式扫描的 SEARCH 必须从已标定 P0 开始。执行 `main.py --sensor real --execute` 后，程序比较
当前 TCP 和 P0；如果不在容差内，会先执行同一套三段安全返回。成功到达 P0 后仍需输入
`START` 才开始扫描；启动返回失败则保持停止并拒绝扫描。

## 返回过程的监控

每个返回段启动前以及运动期间持续检查：

- processed PX6D force/torque 返回阈值；
- absolute raw force/torque 后备阈值；
- PX6D timeout、CRC 和串口状态；
- RTDE receive/control 连接；
- emergency stop、protective stop；
- TCP speed；
- 每段 timeout；
- 最终 P0 位置和 SO(3) 姿态误差。

任一检查失败会调用停止并输出：

```text
SAFE RETURN ABORTED: ...
```

程序保存 `return_status=aborted` 和具体原因，不会强行继续后面的路径段。

## 异常中断后的独立手动返回

先用示教器把机器人放到任意一个适合作为复位终点的位置，然后只读保存一次：

```bash
python3 save_reset_pose.py
# 输入 SAVE_RESET_POSE
```

以后在现场人工检查机器人、目标、颗粒、探针、线缆、安全状态和返回路径后运行：

```bash
python3 return_to_reset.py
```

程序首先只读当前位置，并显示当前位置、TCP 绑定复位点、安全高度和三段路径。只有输入：

```text
RETURN_TO_RESET
```

才会打开 PX6D 和 RTDE 控制接口并开始返回。复位点保存时会同时记录 active TCP；当前 TCP
与该绑定不一致时拒绝运动。未输入完全一致的确认词时不会运动。

不要在 protective stop、emergency stop、通信故障未排除、PX6D 不可用或返回路径不明确时使用
手动返回工具。

## 输出文件

正式扫描目录增加：

```text
full_log.csv
scan_stop_snapshot.json
return_status.json
contour_result.png
```

原有 `samples.csv` 和 `boundary_points.csv` 保留。`full_log.csv` 同时包含扫描周期和
`RETURN_TO_START` 周期。成功回到 P0 后打印 `RETURN TO START COMPLETE`，保存并尝试打开
`contour_result.png`。

## 限制

三段安全返回是明确、可监控的笛卡尔路径，但不是碰撞规划。它不会检查 UR 机器人本体、夹具、
探针、线缆、箱体或环境的三维碰撞，因此必须由现场风险评估确认整条路径。
