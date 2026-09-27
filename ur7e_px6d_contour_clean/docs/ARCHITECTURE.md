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
| `safety/safe_return.py` | 唯一三段返回轨迹/执行器；可选 `PX6DForceMonitor` |
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

`request_stop()` 按最后发送的运动命令选择 `speedStop` 或异步 `stopL`，一次停止过程只发送一个 primitive，不逐个尝试备用停止接口。返回的报告记录 `method`、`return_value`、`exception`、`api_anomaly`、`physical_stop`。

`wait_for_standstill()` 与持续读取共享同一停稳判据：设备 timestamp 必须向前更新，实际线速度低于阈值且角速度足够低，持续配置的 hold 时间。重复读取同一缓存包不能累计“静止”时间；缺包、倒退 timestamp、非有限数据、持续运动均不能证明停稳。默认线速度阈值 0.1 mm/s、hold 80 ms，角速度上限 0.005 rad/s。

`speedStop == False` 或 SDK 异常只产生 API anomaly。若新鲜速度证明已停稳，记录 `physical_stop=confirmed`；若持续运动超时则报 `STOP_MOTION_ERROR`，若观测失效则记录观测错误。API 返回值不替代物理结论。下一次运动不会改写前一次 stop 报告。

连续模式在健康的制动观测期间仍监控 PX6D、日志和时间预算，并喂已有 watchdog；确认持续停稳后结束自己上传的脚本，再断开。异常力、传感器/日志故障或 watchdog 已失效时不继续喂狗，但仍尝试用 Receive 确认实际停稳。Control close 幂等，断开失败不会跳过其他资源释放。

## return

唯一 `SafeReturnExecutor` 负责轨迹、异步 moveL、每段停稳和最终误差检查。

- manual：`force_monitor=None`，不导入 PX6D 驱动，不采零偏，不创建扫描 policy。
- continuous/discrete startup：注入 `PX6DForceMonitor`，静止临时零偏只用于返回；到 P0 后重新采扫描零偏。
- 离散正常结束后返回：同一执行器使用当次扫描预处理器和传感器。

路径按旧几何规则取 +Z 安全高度，再到目标上方，最后下降；已在足够高的位置时不叠加抬升。预先验证全部端点和已启用的工作空间边界，段间必须停稳。完整姿态也显示给操作者，水平段可能包含目标姿态变化。它没有障碍物碰撞规划。

## 日志

`create_run(mode, strategy, config_source)` 在采样前创建带身份的 run。metadata 包含模式、策略、UTC 时间、Git branch/commit/dirty、配置绝对路径；混合模式“真实 PX6D + 模拟机器人”另记 devices。

CSV 按有记录才建立，保留 samples/full_log、有效 policy waypoints、边界/探测记录、配置快照、停止快照、summary 和 termination。手动返回只记录其实际 TCP、目标、停止请求和返回结果。连续写盘线程不调用设备；磁盘异常/积压能阻止下一次健康周期和运动。

菜单 19/20 及回放器读取 `metadata.json` 判断模式和策略；缺少元数据直接报错，不选“最新目录”或推测历史 schema。旧实验仍由旧工程查看；本轮没有旧日志迁移。

## 迁移文件清单

依赖审查后按文件选择，未整体复制。下列清单不包含新写的 runtime、paths、return executor、测试和文档。

### 原样复制的文件

- `core/`：`__init__.py`, `models.py`。
- `config/`：`__init__.py`。
- `sensor/`：`__init__.py`, `force_preprocess.py`, `force_features.py`, `px6d_reader.py`。
- `policy/`：`probe_episode.py`, `__init__.py`, `boundary_estimation.py`, `local_recovery.py`, `local_tracking.py`, `continuous_tracking.py`, `rule_policy.py`。
- `calibration/`：`__init__.py`, `scan_calibration.py`, `reset_pose.py`。
- `workspace/`：`__init__.py`, `workspace_visualizer.py`, `workspace_calibrator.py`, `workspace_transform.py`。
- `simulation/`：`continuous_session.py`, `__init__.py`, `simulated_force_sensor.py`, `continuous_view.py`, `simulated_robot.py`, `top_view.py`, `physical_validation.py`, `geometry.py`, `visualization.py`, `loop_completion.py`, `scene_square.yaml`, `scene_continuous.yaml`。
- `experiment_logging/`：`__init__.py`。
- `app/`：`__init__.py`, `operator_input.py`。
- `robot/`：`__init__.py`。
- `safety/`：`__init__.py`, `force_guard.py`。
- `./`：`run_simulation.py`, `calibrate_workspace.py`, `visualize_workspace.py`, `config.yaml`, `scan_calibration.yaml`, `requirements.txt`。
- `workspace/config/`：`workspace_calibration.yaml`。
- `tools/`：`check_px6d.py`, `check_workspace_calibration.py`, `check_rtde.py`, `run_mock_visualized.py`。

### 复制后按 clean 职责改造的文件

- `config/`：`loader.py`。
- `workspace/`：`workspace_logger.py`。
- `simulation/`：`continuous_preview.py`, `simulator.py`, `simulation_config.yaml`。
- `experiment_logging/`：`continuous_writer.py`, `termination.py`, `data_logger.py`, `probe_log.py`。
- `robot/`：`rtde_controller.py`, `tcp_identity.py`。
- `./`：`run_project.py`, `run_calibration.py`, `save_reset_pose.py`。
- `tools/`：`visualize_run.py`, `check_scan_calibration.py`, `visualize_continuous_run.py`, `read_active_tcp.py`。

连续 runtime 保留原控制周期和安全检查，标定准备拆到 configuration，启动/返回共用 scan_startup 和 SafeReturnExecutor；离散 runtime 重新组织为单次资源生命周期。旧 app/main.py 和连续入口只用于参考，未作为平行旧实现保留。

未复制：旧 `data/`、`simulation_outputs/`、PNG、backup YAML、旧虚拟环境、Git 元数据、历史文档、app/real_validation.py、run_real_validation.py、simulation/continuous_validation.py、非菜单必需的历史 scene YAML，以及大部分历史测试。只选择连续数值策略的关键测试并补写 clean 生命周期、返回、日志和入口测试。
