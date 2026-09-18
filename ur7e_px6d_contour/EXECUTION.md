# 分阶段执行与验证

下面的顺序特意把“代码验证、PX6D、RTDE 只读、真实传感器 dry-run、低速运动”分开。
完成前一阶段并保存结果后再评估下一阶段；本文不会要求直接运行真机运动。
当前目录、数据流和状态图见 [ARCHITECTURE.md](ARCHITECTURE.md)。根目录 `main.py` 是兼容入口，
实际系统调度在 `app/main.py`，原运行命令保持不变。

## 0. 环境准备

本机系统 Python 3.14 没有可用的 `ur-rtde` wheel。使用已经创建的 Python 3.12 环境，并清除
ROS 2 Lyrical 写入的 `PYTHONPATH`：

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour
source /home/user-linux/robotics_cs/.venv312/bin/activate
unset PYTHONPATH
~/.local/bin/uv pip install -r requirements.txt
```

确认 `python --version` 显示 Python 3.12.x。虚拟环境实际位于
`/home/user-linux/robotics_cs/.venv312`，不在 `ur7e_px6d_contour` 内；不要使用相对路径
`source .venv312/bin/activate`。

## 1. 离线单元测试

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
```

预期全部通过。测试不连接 PX6D 和 UR，也不发运动命令，覆盖：

- 官方命令字节、实测 PX6D 数据帧、CRC 拒绝；
- bias、EMA、旋转及 `r × F` 力矩变换；
- force feature 及其诊断记录；
- contact hold、同 anchor 返回、局部 PCA、boundary recovery/confirmation 和 safety STOP；
- dry-run XY 积分、workspace 和真机门锁。

## 2. 正方形完整一圈（离线）

```bash
python3 run_simulation.py --no-gui
xdg-open simulation_outputs/full_loop_square.png
```

默认离线入口现在验证一个中心位于原点的 100 mm × 100 mm 正方形。菜单 2（无窗口）、菜单 6（实时动画）和 `python3 main.py --sensor mock` 使用同一个几何闭环。
力模型为确定性弹簧接触：自由空间为零，接触力沿目标面的局部法向关系生成；noise、drag、friction 均为零。

所有动作来自 `policy/rule_policy.py` 中的真实共享策略；目标几何只提供接触力和事后评估，不传给 policy。
闭环成功要求：四条边都有接触、四次边界恢复确认、足够边界路径、至少 0.95 圈同向环绕、回到首次接触附近、切向一致。
`simulation/loop_completion.py` 只在 simulation 中评分和停止，不用于真机默认停止逻辑。

实时动画用 `python3 run_simulation.py`。图中只显示目标、TCP、执行轨迹和接触点。
成功自动停止；LOST、超时或提前停止明确输出 FAILED 和失败原因，记录最后状态及恢复序号。
旧 `python main.py` 默认 mock 也运行此验证。
真实传感器 dry-run 仍是 `python main.py --sensor real`，需要真实串口。

## 3. PX6D 单独只读检查

先确认没有 ROS 节点或其他程序占用串口，再确认设备稳定路径存在：

```bash
ls -l /dev/serial/by-id/
python tools/check_px6d.py --samples 500
```

该工具只发送版本和数据帧请求，**不会发送 PX6D 硬件清零命令**。保存固件版本、实际读取率、
六维 mean/std/min/max。若有 timeout、CRC、权限或掉线错误，先解决，不进入后续阶段。

建议分别采集并留档：

1. 已安装、空气中静止、无外部接触；
2. 探针处于目标实验姿态；
3. 固定插入深度、颗粒中但确认没有目标；
4. 从已知 `+X_base/-X_base/+Y_base/-Y_base` 方向做非常轻微的人工受力。

这些数据用来区分 bias/重力/颗粒 baseline，并标定传感器轴到 base XY 的映射。不要根据肉眼
猜测 `rotation_sensor_to_base`。新 policy 不用力方向推导边界，但坐标旋转仍必须经轻触验证。

## 4. UR RTDE 单独只读检查

确保 PC 与 `192.168.1.10` 同网段、UR 已上电且现场条件允许只读连接，然后只运行：

```bash
python tools/check_rtde.py
```

该脚本只构造 `RTDEReceiveInterface`，读取 actual TCP pose/speed、TCP offset、robot mode 和
safety mode；不构造 control interface，不调用 `speedL`。将 active TCP 与实测探针尖端 TCP
比较，并记录结果。

## 5. 真实 PX6D + 策略 dry-run

在**机器人保持静止**时，可运行：

```bash
python main.py --sensor real
```

没有 `--execute`，所以仍使用本地模拟 TCP，不连接机器人、不发送运动。它用于验证真实数据能否
驱动阈值和状态机。启动时会按 `preprocessing.baseline` 采集软件 bias；必须先保证传感器处于
该字段声明的物理状态。若要保留原值而不采集，设置 `capture_on_start: false`。

注意：此阶段的模拟 TCP 会移动，但真实探针不动，所以它只能验证数据流、规则、日志和手动施力
触发，不能证明轮廓策略的物理方向正确。

## 6. 参数分析与真机门锁

在考虑任何运动前，至少完成 README 的“必须现场确认的参数”表，并执行以下配置工作：

- 用实测刚体关系填写 sensor-to-base rotation 和力矩臂；
- 明确 bias 是否已包含工具重力，避免重复扣除；
- 用同一姿态、同一插入深度的数据填写 granular baseline；
- 从噪声分布和接触实验确定 contact/hold/safety 阈值，而非沿用示例值；
- 用双点示教标定 P0→P1 初始 SEARCH 方向；确认 probe sign 的后续探测运动确实指向目标，
  retract 确实远离目标；
- 当前按现场检查结果设置 `workspace.enabled: false`；人工检查 TCP、探针、PX6D 与线缆扫掠体；
- 填写真实 active TCP offset，确认 payload/CoG 及 UR 自身安全设置；
- 将首次运动的距离、速度和加速度降到风险评估允许的值，并安排监护人员用 Q 正常结束扫描。
- 用只读工具保存 P0、P1 和初始方向，逐段示教确认垂直退出、顶部横移和下降路径；填写并确认
  `safe_return` 的高度、速度、力矩阈值和误差容差。

先运行：

```bash
python3 run_calibration.py
```

按下面顺序完成双点标定：

1. 示教器移动到扫描起点 P0 并保持静止，输入 `SAVE_P0`；
2. 沿希望正式 SEARCH 的 XY 方向移动一小段到 P1，保持静止；
3. 输入 `SAVE_DIRECTION`；
4. 程序检查 P0/P1 最小 XY 距离和最大 Z 差，计算并保存 P0→P1 单位向量；
5. 运行只读预览：

```bash
python3 tools/check_scan_calibration.py
```

检查终端的 P0、P1、`scan_direction_xy` 以及生成的 `scan_calibration_preview.png`。P1 只定义
初始方向，不是轮廓采样点，也不是机器人正式扫描必须经过的第二个点。如果 P1 太近或 Z 变化
过大，程序不会保存方向并要求重新示教；若中途取消，标定保持未完成，execute 模式会被拒绝。

工程不再使用各项 `confirmed`、`execution.allow_robot_motion` 或风险口令作为启动门锁。但
当前 workspace 软件限位关闭；TCP、扫描标定和安全返回参数仍必须有效，否则程序会在创建
RTDE 控制接口之前拒绝执行。

## 7. 将来的受控低速真机阶段

真机命令被刻意设计为必须同时满足配置门锁和显式确认短语：

```bash
python main.py --sensor real --execute
```

这不是当前阶段的运行要求。到达该阶段时仍需现场监护、急停可触达、先在自由空间验证方向、
逐项审查配置，并以最短搜索距离和最少边界点开始。程序没有碰撞检测或自动避障，软件阈值也
不能替代 UR 安全系统。

程序会要求标定文件同时包含 `start_tcp_pose`、`direction_reference_tcp_pose` 和
`scan_direction_xy`，并检查方向与 P0→P1 一致。若当前 TCP 不在 P0，会先按“Base +Z 抬升
0.030 m → P0 上方 → 下降到 P0”自动返回；通过后会在 `START` 前打印 P0、P1、初始方向、
固定 Z 和固定姿态。必须再次核对方向并输入 `START` 才会
开始。第一次接触前，TARGET_SEARCH 沿标定的初始方向直线扫描；稳定接触后先返回
搜索 anchor，再用附近平行 probes 的 TCP contact poses 估计局部 tangent。扫描中按
`Q` 属于正常停止，程序保存停止点并按“垂直上升 → 安全高度平移 → 垂直下降”返回 P0；按
`ESC` 或 `Ctrl+C` 属于紧急停止，不会自动返回。完整参数和异常后的手动返回方式见
[SAFE_RETURN.md](SAFE_RETURN.md)。

## 故障行为

- `Q`：停止扫描，保存 P_stop，然后在全部安全检查持续有效时自动返回 P0；
- `ESC` / `Ctrl+C`：立即停止并留在当前位置，不自动返回；
- PX6D 串口/CRC/timeout：立即停止，按紧急停止处理，不自动返回；
- RTDE read/control/disconnect 或 protective stop：立即尝试停止，不自动返回；
- processed 或 absolute raw F/T 超限：策略直接 `STOP`；
- Z、姿态、TCP speed 超限：控制器拒绝下一指令并停止；workspace 检查当前已关闭；
- PROBE 超距：先回该 probe anchor，再进入同一 recovery anchor 的递进局部角域；
  候选接触必须通过默认三点 `BOUNDARY_CONFIRMATION`，搜索预算耗尽则 `STOP`；
- 任何扫描阶段的 `Fxy` 正向上升率超过 `force_rate_limit`：立即安全停止；
- 返回期间 F/T、RTDE、机器人安全状态或段超时异常：打印 `SAFE RETURN ABORTED`，
  保存原因并停止后续返回段；
- SEARCH 超过 1.5 m：正常停止并按配置执行安全返回；正式轮廓扫描不再按运行时间或边界点数
  自动结束，保持循环直到用户按 Q。
