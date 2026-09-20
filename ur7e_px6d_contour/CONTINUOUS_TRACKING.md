# 连续轻接触跟踪实验（第一版）

独立入口 `run_continuous_tracking.py`，默认完全离线。原 `run_project.py` / `policy/rule_policy.py` 保持原功能。这个实验验证实时 F/T 速度反馈，不输出完整轮廓重建，也不声称解决颗粒摩擦。

## 文件与复用关系

- `policy/continuous_tracking.py`：纯状态机、方向 EMA、速度反馈、接触持续时间与局部重捕获；使用 `core.models.PolicyCommand/PolicyWaypoint`、原 `force_guard`、原 `handed_tangent` 约定。
- `run_continuous_tracking.py`：连接已有 `URRTDEController/PX6DReader/WrenchPreprocessor`；复用配置加载、扫描标定、TCP 绑定、键盘、`ExperimentLogger/TerminationRecorder`。不包含另一套设备驱动。
- `experiment_logging/data_logger.py`：增加可选扩展列和可选 workspace exporter 开关，原调用默认行为不变。连续入口关闭旧 workspace exporter，其真实标定投影不适用于合成坐标，并避免结束时自动绘图。
- `tools/visualize_continuous_run.py`：只读取已有日志和配置快照的离线同步双视图。
- `simulation/scene_continuous.yaml`：复用已有 circle geometry、`SimulatedRobot/SimulatedForceSensor` 的近距离调试场景，不创建新模拟器。
- `config.yaml`：仅追加 `continuous_tracking` 配置，原机器人、离散 policy、安全阈值均未提高。
- `tests/test_continuous_tracking.py`、`tests/test_continuous_run.py`：状态机、闭环、故障停止、日志和回放测试。

## 状态与首次接触

`READY → TARGET_SEARCH → FIRST_CONTACT → CONTINUOUS_TRACKING`

`CONTINUOUS_TRACKING → CONTACT_LOST → LOCAL_REACQUIRE → CONTINUOUS_TRACKING`

任意安全故障、输入异常、用户停止或预算耗尽：`STOP`，不可自动恢复。

真机初始搜索方向只来自已验证的 P0→P1 标定。必须先把机器人放到 P0；此入口不自动从其他位置返回 P0。沿 `search_speed` 搜索。`Fxy >= policy.contact_threshold` 连续达到 `policy.contact_hold_time` 才接受接触；从接触候选开始即停止推进，稳定后记录 `initial_contact` 和 `FIRST_CONTACT` 事件。下一周期直接执行连续反馈，无回撤、三点初始化或 PCA。

沿用原接触阈值 1 N、稳定时间 0.05 s、控制频率 100 Hz。阈值候选期间如果力再次降到接触阈值以下，确认计时清零，搜索继续。

## 每周期控制

先进行有限值/时间顺序、processed force/torque、raw absolute force/torque、force-rate 检查，然后才能生成运动。

令处理后的 Base 平面力为 `f = [Fx, Fy]`：

```text
Fxy = norm(f)
f_filtered[k] = alpha * f_filtered[k-1] + (1-alpha) * f[k]
filtered_Fxy = norm(f_filtered)
n = force_direction_sign * normalize(f_filtered)
```

`alpha` 是旧样本权重，与项目现有 EMA 约定相同；该方向滤波在现有 wrench preprocessing 之后执行。没有足够信号时保留上一次接触方向，滤波幅值仍逐周期更新。

`n` 只是接触方向估计。正号究竟指向目标必须现场测定，不能由真实 PX6D 型号推断。`force_direction_sign` 可取 `+1/-1`，需使 `n` 指向目标；模拟模型约定 `+1` 指向目标。

复用已有沿边手性约定（n 为朝内方向）：

```text
CLOCKWISE:        t = [-n_y, n_x]
COUNTERCLOCKWISE: t = [ n_y,-n_x]
```

例如在物体左侧 `n=[1,0]`，CLOCKWISE 向 +Y，COUNTERCLOCKWISE 向 -Y。方向每周期由滤波力更新，不是一次接触后锁定。

```text
e = F_ref - Fxy
v_n = 0                                      if abs(e) <= force_deadband
v_n = clip(Kf * e, -v_n_max, +v_n_max)        otherwise
v_xy = v_t * t + v_n * n
```

力偏小时朝目标靠近，力偏大时离开目标。在最后一步将总速度缩放到 `robot.max_tcp_speed` 内，日志中的 v_t/v_n 也同步缩放。Z 与角速度由原 RTDE 接口强制为零，实际固定 Z/姿态漂移继续由原控制器检测。这里只实现速度比例反馈，没有完整 M-B-K admittance。

## 丢失与局部重捕获

只有 `Fxy < contact_lost_threshold` **连续**超过 `contact_lost_hold_sec` 才触发 CONTACT_LOST。单帧下降不会触发；低力确认窗口内仍执行有界跟踪反馈。最后可靠姿态、tangent 和接触方向只在 `Fxy >= policy.contact_threshold` 时更新，因此低信号不会覆盖恢复依据。

触发时停止，下一周期进入 LOCAL_REACQUIRE 并继续保持零速一个周期。随后以最后接触方向为中心，用最后 tangent 确定先扫的一侧，执行 `0 → +A → -A → +A …` 的三角波 heading 扫描。保持低速、不返回旧接触点、不沿旧 tangent 正常前进，不分析任何拐角类型。

heading 限制在 `±reacquire_max_angle_deg`，转动速率为 `reacquire_angular_speed_deg_s`。同时受最大时间及以 `last_contact_pose` 为中心的距离半径限制；预测下一步超过半径也停止。力重新超过接触阈值就暂停扫描等待稳定，稳定后记录 `REACQUIRED` 并回到跟踪。当次命令为停止，下一周期恢复反馈。稳定条件未满足即到预算边界时，优先 STOP。

## 新配置（所有值 MUST CONFIRM ON SITE）

| 参数 | 默认值 | 含义 |
|---|---:|---|
| enabled | true | 仅供独立入口启用；不改变旧入口 |
| tangential_speed | 0.001 m/s | 连续切向速度 |
| force_reference | 1.5 N | 目标平面交互力幅值 |
| force_gain | 0.0005 (m/s)/N | 法向比例增益 |
| force_deadband | 0.15 N | 力误差死区 |
| normal_speed_limit | 0.0005 m/s | 法向速度绝对值上限 |
| force_direction_sign | +1 | 仿真约定；真机必须确认 |
| force_direction_filter_alpha | 0.8 | 方向 EMA 旧值权重 |
| search_speed | 0.001 m/s | 初始搜索速度 |
| search_max_distance | 0.10 m | 从起点算的搜索距离预算 |
| search_max_time_sec | 110 s | 初始搜索时间预算 |
| contact_lost_threshold | 0.5 N | 低力阈值 |
| contact_lost_hold_sec | 0.15 s | 持续低力窗口 |
| reacquire_speed | 0.0005 m/s | 局部扫描速度 |
| reacquire_max_angle_deg | 60° | heading 半角 |
| reacquire_angular_speed_deg_s | 30°/s | heading 扫描速度 |
| reacquire_max_time_sec | 8 s | 每次局部搜索预算 |
| reacquire_max_distance | 0.004 m | 最后可靠接触点周围半径 |
| max_runtime_sec | 120 s | 本次实验总时限 |
| visualization_fps | 10 | 离线回放默认帧率 |

`follow_hand`、稳定接触判定、全部力/力矩/变化率阈值与控制周期仍读取原 `policy` 配置。force-rate 与旧 policy 一致：保护 **正向上升速率**，不把接触释放造成的负向下降当冲击。新配置会拒绝非有限数、无效 sign、不合理的阈值关系、超过原搜索速度/距离及机器人上限的配置。

## 离线运行与回放

在项目目录执行（也可换为已有依赖的 `python3`）：

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour
../.venv312/bin/python run_continuous_tracking.py --dry-run --duration 30
# 完整默认 120 秒模拟时间；无实时等待，完全离线：
../.venv312/bin/python run_continuous_tracking.py --dry-run
# 可选择已有场景；算法不知道目标几何：
../.venv312/bin/python run_continuous_tracking.py --dry-run --scene simulation/scene_3_circle.yaml
```

默认近距离圆形场景约 10 秒开始接触。使用原始远距离场景会有更长的低速初始搜索。模拟采用 continuous 的 100 Hz 周期、真实配置的安全阈值和控制增益；只使用场景的环境、合成力模型和合成 Base-frame 坐标变换，保留主配置 wrench EMA。不会把现场传感器外参/重力再次施加到合成 Base-frame 力上。配置快照保留这些选择。

`--duration` 只能缩短配置中的总时限，不能绕过它。程序输出实际 run 目录；可用 `--output` 指定日志根目录。正常用户停止或实验总时限结束返回 0，安全故障及搜索/重捕获预算失败返回 1。达到时限只代表本次运行结束，不代表完成整圈。

```bash
../.venv312/bin/python tools/visualize_continuous_run.py data/run_实际时间戳 --fps 10
# 只保存最终静态总结：
../.venv312/bin/python tools/visualize_continuous_run.py data/run_实际时间戳 --format none
```

输出 `continuous_summary.png` 和 `continuous_replay.mp4`（ffmpeg 可用时），否则使用 GIF。GIF 为限制内存最多 240 帧，长记录自动降低有效帧率并保留真实时长；编码失败仍保留 PNG。没有数千张中间 PNG。左图 `set_aspect("equal")`，XY 用米且保持真实比例；方向箭头按同一 8 mm 显示长度绘制，不表示力或速度量纲。显示实际 TCP、轨迹、接触/丢失/重捕获事件，已知容器来自配置快照。右图 Fx/Fy/Fxy/F_ref 与同一时间游标同步。按实际时间抽样，兼容真机控制抖动。

## 日志

继续写入项目原有 `samples.csv` / `full_log.csv`、`policy_waypoints.csv`、配置快照、summary、stop snapshot、termination JSON。连续模式不生成离散探测点；`boundary_points.csv` 等兼容文件只有表头。

已有列保留六维 raw / processed wrench、完整 TCP pose/speed、状态和接触标志。扩展列提供 `filtered_fxy`、未追加方向 EMA 的 processed `force_direction_x/y`、滤波且经 sign 修正的 `contact_direction_x/y`、`force_reference`、`force_error`、`v_t/v_n`、`command_vx/vy/speed`、`contact_lost_timer`、`follow_hand`、`force_direction_sign`、`force_rate` 和 `reacquire_heading_deg`。已有 `tangent_x/y` 记录当前方向；时间为 `monotonic_sec` 和 UTC。

事件包括 FIRST_CONTACT、CONTACT_LOST、LOCAL_REACQUIRE、REACQUIRED、SAFETY_STOP、USER_STOP、BUDGET_STOP。summary 保存 initial_contact 和 last_contact_pose。原始与处理后力并存，不把交互力解释为纯物体法向力。

## 真机启动与首次实验观察

1. 先完成原项目 PX6D 只读检查、RTDE/TCP 检查、P0/P1 标定及空载静止零偏准备。确认 `rotation_sensor_to_base`、TCP offset、固定 Z/姿态与现场一致。当前配置的旋转是占位 identity，必须现场确认。
2. 用已知安全操作将 probe 放到标定 P0，保持静止且零偏采样时不接触目标。这个入口不执行自动回位或自动离开接触点；所有停止均停在原位。
3. 核对 sign，使 `n` 朝目标；核对上述保守参数与现有安全阈值。现有配置 `workspace.enabled: false` 会原样保留，软件笛卡尔范围限制仍关闭；离线图里的容器边框不构成安全限位。需要软件范围检查时先填入实际有效范围再启用。
4. 启动：

```bash
../.venv312/bin/python run_continuous_tracking.py --execute --duration 30
```

设备连接后仍由原控制器执行 active TCP 身份校验、固定姿态/Z、已启用 workspace、实际速度及 RTDE emergency/protective stop 检查。P0 位置不符直接拒绝运动。确认终端中的 P0、搜索方向、sign、参考力后输入 `START`。Q、ESC、Ctrl+C 都立即请求停止，不自动返回 P0；传感器/控制器/日志异常也停止，停止后才写最终诊断。未执行硬件零点命令。

首次最应观察：Fx/Fy 与真实方向是否一致；接触后 Fxy 是否在参考力附近；v_n 在力过低/过高时符号是否正确；是否长期饱和；tangent 是否抖动或滞后；是否出现误 CONTACT_LOST 或反复恢复；实际 TCP 速度/Z/姿态；force-rate 是否接近保护上限。利用日志时间差检查实际循环频率，Python、串口、RTDE 和文件写入不能保证硬实时 100 Hz。

当前尚无真机验证：力符号/外参、增益和滤波延迟、真实接触稳定性、颗粒阻力污染、粗糙表面/急变边界、局部扫描能否重新找到目标、真实 RTDE 与急停响应时延。模拟是可解释的合成调试模型，不是颗粒真实物理证明。

## 测试

```bash
MPLCONFIGDIR=/tmp/continuous-mpl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  ../.venv312/bin/python -m pytest -q
```

新增测试覆盖首次接触直入跟踪、低/高力与死区、速度限制、手性/sign、EMA 连续更新、单帧与持续低力、停止旧方向、稳定重捕获、恢复超时/距离预算、全部活动状态的 force/torque/raw/rate 优先级、配置拒绝、入口硬件替身故障、Q/ESC/Ctrl+C、日志字段、离线采样/比例/游标、GIF 与编码失败降级。原离散测试一并运行。

本次离线验证记录：默认场景运行 120 s，12,001 个控制周期，9.68 s 首次稳定接触；11,031 个连续跟踪周期的 Fxy 为 1.09–1.66 N，平均 1.64 N，最大指令速度 0.001021 m/s。该场景未触发丢失接触；丢失/恢复分支由确定性单元测试覆盖。当前环境缺少 ffmpeg，已实际生成静态 PNG 和 240 帧 GIF（120 s，降级为 2 fps）；MP4 编码路径尚未在当前环境成功验证。

最终全量测试结果：`612 passed in 107.47s`，包括原有 543 项和新增 69 项；`git diff --check` 通过。所有验证均在离线或设备替身中完成，没有连接 UR7e/PX6D。
