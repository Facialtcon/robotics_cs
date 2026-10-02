# 当前架构

## 职责

| 模块 | 职责 |
|---|---|
| `run_project.py` | 兼容 1～20 菜单，子进程启动，明确选 run 后回放 |
| `app/continuous_runtime.py` | 连续采样、policy、执行、watchdog、异步日志和统一收尾 |
| `app/discrete_runtime.py` | 原离散 policy 的采样、执行、正常返回和统一收尾 |
| `app/return_runtime.py` | UR-only 返回目标选择、预检、路径确认和返回日志 |
| `app/scan_startup.py` | 两种扫描共用的静止零偏、START 确认和启动返回 |
| `app/configuration.py` | 离线标定身份检查、工作空间和速度边界推导 |
| `robot/rtde_controller.py` | 唯一 Control owner；Receive、运动边界、stop 与停稳 |
| `sensor/` | PX6D 协议、预处理和力特征 |
| `policy/` | 保留的连续/离散数值算法及其必要依赖 |
| `safety/safe_return.py` | 唯一四段返回轨迹/执行器，安全高度对正姿态；可选 `PX6DForceMonitor` |
| `experiment_logging/paths.py` | 唯一 run 路径分配与 metadata 来源 |
| `experiment_logging/` | 按需 CSV、连续异步 writer、终止诊断 |
| `workspace/` | 原标定几何及离线投影 |
| `simulation/` | 原环境动力学、离散仿真和连续预演/绘图 |

根目录 `main.py`、`run_continuous_tracking.py`、`return_to_reset.py`、`return_to_start.py` 是不足十行的兼容 wrapper；实际实现位于 `app/`。保留的标定/仿真入口直接服务已有菜单，未增加空占位模块。

## RTDE 生命周期

每个真实任务创建一个 `URRTDEController`，由它管理一个 Receive 和至多一个 Control。`connect()` 仅创建 Receive；所有运动都要在现场确认后经 `activate_control(confirmed=True)` 创建 Control，并验证 active TCP。禁止该 owner 重新连接 Control。返回、正式扫描和收尾共用这一个对象。

Receive 负责 TCP pose/speed、设备 timestamp、机器人/安全状态。当前安装的 ur-rtde 1.6.5 的 Receive 类没有 `getTCPOffset`；只读标定工具通过 UR 的 **30012 只读 Secondary** 状态流解析 Cartesian Info 中的 TCP offset，不上传脚本，不创建 Control。运动任务直接从已确认创建的唯一 Control 验证 TCP。

协议依据：[UR Primary and Secondary Interfaces](https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/primary-secondary-guide.html)。此协议读取实现有分包测试，尚未连接现场端口验收。读取失败就报错，不退回创建 Control。

## stop

连续扫描与共享返回使用 `request_stop(nonblocking=True)` → `STOPPING` → 每周期 `poll_stop()` → `STOPPED`。首次请求只发送一次制动指令；后续周期读取新鲜速度并累计静止时间，达到有限 `confirmation_timeout_sec` 仍未停稳则进入 `FAILED` / `STOP_MOTION_ERROR`。SEARCH_LIMIT 和 FIRST_CONTACT 共用此实现，前者停稳后正常退出，后者才允许继续接触/方向确认并进入跟踪。

ur-rtde 1.6.5 的 `speedStop(a)` 没有异步参数，其控制脚本同步执行 `stopl(a)`。因此速度模式先发送一次 `speedL([0]*6, stop_deceleration, 0.01)`，使用原停车减速度把持续速度目标设为零；跨周期确认停稳后，才调用一次 `speedStop` 退出速度模式。返回 moveL 使用 SDK 原有异步 `stopL(a, True)`。所有 Control 调用仍在同一线程，没有以并发喂狗绕过 SDK 的线程安全约束。报告另记 `braking_method` / `braking_return_value`，区分零速度制动与速度模式退出的结果。

接口依据：[SDU Robotics 接口说明](https://sdurobotics.gitlab.io/ur_rtde/introduction/introduction.html)、[控制脚本](https://gitlab.com/sdurobotics/ur_rtde/-/blob/master/scripts/rtde_control.script)。停稳后的 SDK 模式退出仍有通信往返开销，继续记录周期时长；真实 continuous 的 30 ms 现在是诊断阈值，单次超过不触发 STOP。

停车确认要求设备 timestamp 向前更新、实际速度持续低于阈值。所有 motion braking/settling（包括 return 每段、FIRST_CONTACT 和最终 STOP）使用线速度 0.1 mm/s、角速度 0.005 rad/s、hold 80 ms 及有限停车超时。只有真实 continuous 运动前的启动预检/bias/确认使用 1 mm/s、0.01 rad/s。`wait_for_standstill(startup=False)` 始终严格；`startup=True` 只适用于运动前的预检，不能放宽已开始的扫描或任何 stop request。`_return_mode` 不再授权宽松停稳；返回段间继续非阻塞地 request_stop/poll_stop，扫描循环不增加阻塞等待。重复读取同一缓存包不能累计静止时间。

`speedStop == False` 或 SDK 异常只产生 API anomaly。若新鲜速度证明已停稳，记录 `physical_stop=confirmed`；若持续运动超时则报 `STOP_MOTION_ERROR`，若观测失效则记录观测错误。API 返回值不替代物理结论。下一次运动不会改写前一次 stop 报告。

连续模式在制动观测期间继续读取 PX6D、TCP 并记录诊断；确认持续停稳后结束自己上传的脚本，再断开。默认 `continuous_tracking.continuous_require_watchdog: false`，启动、bias、返回、扫描、停车均不调用 setWatchdog / kickWatchdog / watchdog health check。显式开启时保留原 watchdog 路径。UR 自带 emergency/protective stop 仍终止运动。Control close 幂等，断开失败不会跳过其他资源释放。

## 真实搜索范围

`safety/search_geometry.py` 从扫描 P0 沿已保存的 `scan_direction_xy`，对原始沙箱四角的真实边逐条求交，取前方最近交点。它支持顺/逆时针凸四边形，拒绝无效顺序、非有限数据、零方向、起点不在内部或没有前方交点；不使用 AABB 或 P1 距离作为搜索预算。

真实预算为几何距离减去原有 `search_boundary_margin`，保存在运行快照 `continuous_search_geometry`，启动连接设备前打印三项距离。YAML 的 `search_max_distance: 0.10` 保留供仿真使用；真实分支不再用它封顶。搜索预测从标定 P0 起算，下一步进入停车余量前正常触发 `STOP_SEARCH_LIMIT`。真实 continuous 默认 `max_runtime_sec: null`，即使载入旧的有限预算也只 warning；`--duration` 仅用于仿真/预演。过去的周期抖动不扩大下一条 10 ms 命令的几何预测步长。

斜边的垂直净距或更严格的工作空间仍可让搜索提前结束；这些执行边界也按 SEARCH_LIMIT 正常停车。实际制动允许消耗停车余量，原始 polygon / 工作空间外边界仍为硬限制。力安全、速度阶段限制与停车减速度保持原值。

## 真实 continuous 的 warning 与 STOP

初始搜索不运行 tracking / direction / reacquire 的规则。运动硬停止条件为操作者停止、UR emergency/protective stop、设备通信/API 错误、NaN/Inf、全局 TCP 超速、标定 polygon 搜索终点、raw force >= 60 N 或 raw torque >= 5 Nm，以及明显异常的 Z/姿态偏移。Fxy >= 1 N 是正常 FIRST_CONTACT 转移，共用原停车状态机。

真实 FIRST_CONTACT 中，接触和实际停稳都确认成功后进入 CONTINUOUS_TRACKING。若停稳后 Fxy 掉到 1 N 以下，或停稳但原 confirmation_timeout_sec 内未能确认接触，则清空本次确认窗口、记录 `FIRST_CONTACT_RETRY_SEARCH` / warning，并返回 TARGET_SEARCH。转换当周期保持零速，下一周期仍先检查接触与原 polygon 搜索预算；搜索方向、P0 起点和累计空间预算不重置。策略停稳 hold 与执行层停稳确认缺一不可，未停稳不能重新搜索。仿真维持原确认失败行为。

processed force 12 N / torque 1 Nm、force-rate、方向跳变/反转/相干度/置信度、阶段 1.2x/1.5x 速度检查、30 ms 周期、sample gap、observation age、RTDE stagnation、日志延迟/积压以及 startup/bias 抖动，在真实模式只记录 WARNING / diagnostic。实际 Z 漂移超过原 1 mm、姿态超过原约 1° 时 warning；超过 5 mm / 5° 仍 STOP。速度命令始终为 `[vx, vy, 0, 0, 0, 0]`，未修改目标 Z/TCP 或标定。方向估计、导纳和切向/法向速度公式保留。方向重确认超时继续等待估计；重启无进展暂停一周期后继续使用实时估计。overload stall 只 warning、切向速度置零，保留原向外法向修正。

`capture_stationary_bias()` 在建立零偏前仅采集 raw，检查 finite、60 N/5 Nm 原始上限和基本静止；完成 `set_zero_bias(samples)` 后才开始 process/processed/contact/force-rate。15 N 恒定原始载荷可完成零偏采集。

真实 continuous 的 bias 允许线速度 1 mm/s、角速度 0.01 rad/s、位置漂移 1 mm；超过旧阈值但未超过新阈值只 warning。P0 启动检查也采用 1 mm，不改保存坐标。明显运动、P0 不满足启动容差或配置/标定/TCP 不匹配仍拒绝启动。

真实默认启用现有 local reacquire：低力先停车，持续 0.15 s 后进入 CONTACT_LOST，实际停稳后执行原局部圆弧，重新接触后停稳确认再恢复 tracking。记忆过期、置信度、重复失联无进展、reacquire 时间预算和跟随误差只 warning；缺少记忆或显式关闭 recovery 时原地等待接触。原圆弧角度、累计路径和位移空间预算耗尽且未接触才 STOP；接触到达预算边缘时先停车确认。没有新增恢复算法。仿真保留原严格 guard 行为。

## return

唯一 `SafeReturnExecutor` 负责轨迹、异步 moveL、每段停稳和最终误差检查。

四段依次为 `VERTICAL_RETREAT → ALIGN_PROBE_ORIENTATION → MOVE_ABOVE_START → DESCEND_TO_START`。第一段沿 Base +Z 抬升，保持实际起始姿态。到达原 safe_z 并严格停稳后，在实测 TCP 位置只对正姿态；随后保持目标姿态平移和下降。姿态误差已在 `return_orientation_tolerance` 内则打印 skipped 并不发 ALIGN moveL。抬升未完成、低于安全高度或停稳失败不得旋转；ALIGN 失败不得平移或下降。返回水平速度为 27 mm/s，垂直仍为 18 mm/s，全局 TCP 上限仍为 30 mm/s。

扫描所称“探针垂直向下”直接定义为已保存的 `scan_calibration.yaml.fixed_orientation`，不重新推算 rotation vector；加载器保证它等于完整 P0 pose 的姿态。手动返回复用同一路径与执行器，RESET 使用保存的 reset 姿态，P0 回退使用扫描姿态；均保留完整目标 pose。两者仅在有无 PX6D monitor 等已有任务配置上区分，不复制路径实现。

真实 FIRST_CONTACT 成功转为 tracking 时，runtime 消费该转换事件并打印一次 Fxy、控制 normal n、tangent t、follow hand 和 force_direction_sign；算法公式及正常周期输出不变。`tools/check_force_direction.py` 独立使用 PX6D 与现有预处理，只展示配置换算后的 Fx/Fy/Fxy、未乘 sign 的单位力方向和乘 sign 的候选 n，不导入/连接机器人驱动、不修改配置。

真实 continuous startup return 的 processed 8 N / 0.7 Nm 只 warning，raw >= 60 N / 5 Nm 和非有限值仍终止；PX6D samples、warning 和 return_status 均保留。返回段软件时间预算只 warning，保持观测并接受人工停止；返回末尾仍必须满足 1 mm 启动位置容差。手动返回和 discrete 保留原行为。

- manual：`force_monitor=None`，不导入 PX6D 驱动，不采零偏，不创建扫描 policy。
- continuous/discrete startup：注入 `PX6DForceMonitor`，静止临时零偏只用于返回；到 P0 后重新采扫描零偏。
- 离散正常结束后返回：同一执行器使用当次扫描预处理器和传感器。

路径按旧几何规则取 +Z 安全高度，再到目标上方，最后下降；已在足够高的位置时不叠加抬升。预先验证全部端点和已启用的工作空间边界，段间必须停稳。完整姿态也显示给操作者，水平段可能包含目标姿态变化。它没有障碍物碰撞规划。

## 日志

`create_run(mode, strategy, config_source)` 在采样前创建带身份的 run。metadata 包含模式、策略、UTC 时间、Git branch/commit/dirty、配置绝对路径；混合模式“真实 PX6D + 模拟机器人”另记 devices。

CSV 按有记录才建立，保留 samples/full_log、有效 policy waypoints、边界/探测记录、配置快照、停止快照、summary 和 termination。手动返回只记录其实际 TCP、目标、停止请求和返回结果。连续写盘线程不调用设备；真实模式的积压/写盘异常只记 warning。队列仍有固定容量，满时丢弃新记录并累计 dropped_records，不阻塞设备线程或转成 STOP。software_warnings 同时写入样本、summary、独立 termination 和结束时的终端输出。

连续样本增加 `runtime_stop_state`、`actual_xyz_speed_mps`、`stop_api_anomaly`。policy 已输出 STOP 时，runtime 的 STOPPING 样本仍逐周期写入，直到物理停稳；正常耗尽搜索预算的退出码为 0，真实停车失败为 1，并记录 STOP_MOTION_ERROR。

菜单 19/20 及回放器读取 `metadata.json` 判断模式和策略；缺少元数据直接报错，不选“最新目录”或推测历史 schema。旧实验仍由旧工程查看；本轮没有旧日志迁移。

## 迁移文件清单

依赖审查后按文件选择，未整体复制。下列清单不包含新写的 runtime、paths、return executor、测试和文档。

### 原样复制的文件

- `core/`：`__init__.py`, `models.py`。
- `config/`：`__init__.py`。
- `sensor/`：`__init__.py`, `force_preprocess.py`, `force_features.py`, `px6d_reader.py`。
- `policy/`：`probe_episode.py`, `__init__.py`, `boundary_estimation.py`, `local_recovery.py`, `local_tracking.py`, `rule_policy.py`。
- `calibration/`：`__init__.py`, `scan_calibration.py`, `reset_pose.py`。
- `workspace/`：`__init__.py`, `workspace_visualizer.py`, `workspace_calibrator.py`, `workspace_transform.py`。
- `simulation/`：`continuous_session.py`, `__init__.py`, `simulated_force_sensor.py`, `continuous_view.py`, `simulated_robot.py`, `top_view.py`, `physical_validation.py`, `geometry.py`, `visualization.py`, `loop_completion.py`, `scene_square.yaml`, `scene_continuous.yaml`。
- `experiment_logging/`：`__init__.py`。
- `app/`：`__init__.py`, `operator_input.py`。
- `robot/`：`__init__.py`。
- `safety/`：`__init__.py`。
- `./`：`run_simulation.py`, `calibrate_workspace.py`, `visualize_workspace.py`, `config.yaml`, `scan_calibration.yaml`, `requirements.txt`。
- `workspace/config/`：`workspace_calibration.yaml`。
- `tools/`：`check_px6d.py`, `check_workspace_calibration.py`, `check_rtde.py`, `run_mock_visualized.py`。

### 复制后按 clean 职责改造的文件

- `policy/`：`continuous_tracking.py`，真实几何搜索预算、执行层停稳确认与真实模式软件诊断。
- `safety/`：`force_guard.py`，零偏前 raw 检查和真实模式 processed 诊断。
- `config/`：`loader.py`。
- `workspace/`：`workspace_logger.py`。
- `simulation/`：`continuous_preview.py`, `simulator.py`, `simulation_config.yaml`。
- `experiment_logging/`：`continuous_writer.py`, `termination.py`, `data_logger.py`, `probe_log.py`。
- `robot/`：`rtde_controller.py`, `tcp_identity.py`。
- `./`：`run_project.py`, `run_calibration.py`, `save_reset_pose.py`。
- `tools/`：`visualize_run.py`, `check_scan_calibration.py`, `visualize_continuous_run.py`, `read_active_tcp.py`。

连续 runtime 保留原控制周期和安全检查，标定准备拆到 configuration，启动/返回共用 scan_startup 和 SafeReturnExecutor；离散 runtime 重新组织为单次资源生命周期。旧 app/main.py 和连续入口只用于参考，未作为平行旧实现保留。

未复制：旧 `data/`、`simulation_outputs/`、PNG、backup YAML、旧虚拟环境、Git 元数据、历史文档、app/real_validation.py、run_real_validation.py、simulation/continuous_validation.py、非菜单必需的历史 scene YAML，以及大部分历史测试。只选择连续数值策略的关键测试并补写 clean 生命周期、返回、日志和入口测试。
