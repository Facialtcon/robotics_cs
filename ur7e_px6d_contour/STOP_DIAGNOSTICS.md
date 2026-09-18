# 扫描停止诊断

这次修改统一记录“为什么停止、停在哪里、当时输入和命令是什么”。原有扫描算法、停止条件和硬件操作保持不变。

正式扫描仍从主工程运行：

```bash
python3 run_project.py
```

选择 **12：执行真正的 UR7e + PX6D 轮廓扫描**。也可使用原有 `python3 main.py --sensor real --execute`。菜单确认和 `START` 流程保持原样；**不需要先执行空气验证**。

## 修改前的退出路径

下表列出已有行为；本次增加诊断，不新增这些停止条件。

| 文件 / 函数 | 原有退出原因或分支 | 原来不容易定位的地方 |
|---|---|---|
| `main.py`、`app/main.py` 的入口 | 参数、配置、标定、对象构造失败 | 通常只有异常类型和短消息；尚无实验目录 |
| `app/main.py::run` | mock运行结束；启动返回P0失败；未输入START | 原因分散于返回码、终端和return状态 |
| `app/main.py` 扫描循环 | Q、ESC、Ctrl+C；policy进入STOP/STOP_SCAN/LOOP_COMPLETE | 通用“STOP”不能说明触发阶段 |
| `app/main.py` 返回和清理 | Q后安全返回失败；停止、关闭设备、保存结果失败 | 后续异常可能掩盖最初停止原因 |
| `policy/rule_policy.py::_safety_reason` | 非有限输入；力、力矩、力变化率超限；初始化或运行超时 | 需要关联触发帧与活动probe |
| `policy/rule_policy.py::_acquired` | 返回搜索anchor后仍未取得有效接触 | 与后续tracking丢失目标容易混淆 |
| `policy/rule_policy.py::_retry_initialization` | 局部初始化重试耗尽 | 需要保留接触数量、间距和拟合残差原文 |
| `policy/rule_policy.py::_tracked`及纠偏函数 | NO_CONTACT、NO_FORWARD_PROGRESS、REPEATED_CONTACT、FIT_FAILURE；纠偏耗尽 | 多数首先进入恢复或重新初始化，并非立即停止 |
| `policy/rule_policy.py::_next_recovery_probe` | 扩展恢复射线耗尽 | 需要保留恢复编号、次数及anchor |
| `policy/rule_policy.py::_execute_transfer` | anchor转移期间意外接触 | 需要区分正常probe接触与转移碰撞 |
| `safety/safe_return.py::execute/_run_segment` | 返回力超限；阶段超时；位置/姿态误差；设备异常 | 异常原先转换为ReturnResult字符串，原traceback无法在外层重建 |
| `robot/rtde_controller.py` | RTDE断开、控制器拒绝、工作空间/Z/姿态/速度/TCP检查失败 | 需要区分配置问题、运动问题和位置限制 |
| `sensor/px6d_reader.py` | PX6D连接/串口/超时/CRC或报文错误 | 需要保留原异常及底层异常链 |
| `simulation/simulator.py` | policy终止；几何审计失败；完成一圈；时间/步数上限；人工停止 | 后续finalize或绘图错误可能遮住已有扫描结果 |
| `app/real_validation.py`、`run_real_validation.py` | 未授权、操作员停止、空气测试完成、独立真力保护、边界/停稳/覆盖检查、异常 | 需要分开记录真实传感器和测试输入 |

## 统一原因

定义位于 [experiment_logging/termination.py](experiment_logging/termination.py)。分类依据现有明确消息、异常类型和已观测上下文；无法确定时保留原文并使用UNKNOWN，不猜测原因。

| 枚举 | 含义 |
|---|---|
| `SUCCESS` | 既有完成判据通过；空气测试会注明其测试范围 |
| `STOP_USER_REQUEST` | Q、ESC、Ctrl+C、未确认开始或人工停止 |
| `STOP_FORCE_LIMIT` | 力、力矩或力变化率达到既有保护条件 |
| `STOP_WORKSPACE_LIMIT` | 既有工作空间或空气测试范围限制 |
| `STOP_UNEXPECTED_CONTACT` | 转移等不应接触的阶段出现接触 |
| `STOP_NO_CONTACT` | 未获得有效接触或接触不稳定 |
| `STOP_RECOVERY_EXHAUSTED` | 恢复尝试耗尽 |
| `STOP_NO_FORWARD_PROGRESS` | 接触点没有足够正向前进 |
| `STOP_REPEATED_CONTACT` | 重复接触已有位置 |
| `STOP_FIT_FAILURE` | 局部拟合/初始化不能建立有效边界 |
| `STOP_ANCHOR_ERROR` | anchor或返回目标的到位错误 |
| `STOP_MOTION_ERROR` | 运动执行、RTDE或机器人状态错误 |
| `STOP_SENSOR_ERROR` | PX6D连接、读取、超时或协议错误 |
| `STOP_INVALID_WRENCH` | 力/力矩样本包含NaN或无穷值 |
| `STOP_SEARCH_LIMIT` | 既有搜索距离限制 |
| `STOP_TIME_LIMIT` | 既有时间或仿真步数限制 |
| `STOP_CONFIG_ERROR` | 配置、标定或命令行错误 |
| `STOP_POINT_LIMIT` | 既有边界点数量限制 |
| `STOP_UNKNOWN_REASON` | 缺少足够证据分类；原始消息和可用traceback仍保留 |

**原因名称不等于停止命令。** 例如一次NO_CONTACT可以记为`terminal=false`事件，随后policy按原逻辑执行recovery；诊断层不会因此停止机器人。

首次`terminal=true`事件冻结主原因及当时现场。随后STOP、人工结束、返回失败、关闭设备失败或绘图失败作为后续事件记录，不覆盖真正的首次原因。Q后返回失败时，可以同时看到主原因`STOP_USER_REQUEST`和返回失败的详细事件。

空气模式有一个明确例外：fixture完成后仍需通过原有停稳和覆盖验收；若这些验收失败，撤销尚未确认的`SUCCESS`，将失败提升为主记录，原SUCCESS事件仍保留。该诊断处理不改变原来的status或动作。

## 接口及现场字段

`RuleBasedPolicy`持有`policy.termination`；正式应用、simulation和空气验证在各自执行链路中共享这个记录器，不维护另一套停止状态机。

| 接口 | 作用 |
|---|---|
| `TerminationRecorder(output_root=None, print_fn=print)` | 创建记录器 |
| `observe(policy=..., robot=..., raw=..., processed=..., command=..., ...)` | 更新可用上下文；仅内存操作 |
| `set_stop_reason(reason=None, detail="", exception=None, source="", terminal=True, replace=False, **context)` | 保存事件和必要时冻结主原因；仅内存操作 |
| `record` / `events` | 主记录 / 全部事件 |
| `bind(run_dir)` | 关联本次结果目录 |
| `flush(emit=False)` | 尽力写文件；可在终端输出报告 |
| `summary_fields()` | 为现有summary补充统一诊断字段 |

现场采集和原因记录放在原有停止动作附近；文件写入、报告输出安排在已有停止操作之后。`safe_return.py`只增加观测和异常记录hook，保留原来的移动、停止、检查和返回值。

| 字段 | 内容 |
|---|---|
| `reason`、`detail`、`source` | 统一分类、原始原因、触发函数/位置 |
| `state`、`policy_state`、`policy_sub_state`、`phase` | 终止时的应用/policy阶段 |
| `timestamp_utc`、`monotonic_sec` | 报告事件时间及可用的单调时钟/仿真时间 |
| `tcp_pose`、`tcp_speed`、`robot_sample_timestamp` | 当时最后可用的实际TCP观测及采样时间；不是“已停稳”的承诺 |
| `anchor_pose`、`last_contact` | 当前或最近probe的anchor、最后已观测接触点 |
| `probe_id`、`probe_phase`、`recovery_id` | 对应的探测及恢复上下文 |
| `recovery_probe_summary`、`recovery_outcome_counts`、`recovery_rejection_counts` | 当前恢复轮次的结果和拒绝统计，区分无接触与候选点失败 |
| `raw_wrench`、`processed_wrench`、`actual_force` | 原始六维传感器样本、处理后样本及力向量 |
| `command` | 当前/最近命令的目标、方向、速度、阶段及移动标志 |
| `exception_type`、`exception_chain`、`traceback` | Python异常类型、底层异常链、文件/行号/函数调用栈 |

没有取得的数据写为`null`，不伪造位置、接触点或传感器值。NaN/无穷值在诊断JSON中使用明确字符串表示，避免生成非标准JSON。正常policy停止并非Python异常，因此其traceback可以为空。

空气验证另外保留`synthetic_policy_wrench`、`policy_processed_wrench`和`actual_*`字段；测试输入不会冒充真实PX6D测量，也不会被描述为完成了物理接触验证。

## 输出文件和兼容性

正式扫描的常规目录仍为`data/run_<时间戳>/`；simulation仍使用原来的`simulation_outputs/<场景>_<时间戳>/`；空气模式仍使用自己的`air_validation_<时间戳>/`。

| 文件 | 用途 |
|---|---|
| `termination.json` | 主原因、冻结现场、`events`及`secondary_errors` |
| `termination.txt` | 可直接阅读的停止报告与异常栈 |
| `summary.json` | 保留已有字段，并增加`termination_reason`、`termination_detail`、`termination`、`termination_context`；更新已有summary时还保留终止事件及后续异常 |

旧的`reason`、`stop_reason`、`return_status`、`return_abort_reason`、仿真成功标记等继续保留。因此旧绘图和结果读取流程不需要迁移；查根因时优先看统一主记录，再看后续事件。

已有CSV、probe记录和轮廓图仍按原流程输出。异常发生时，诊断文件尽量先保存；不能把“诊断已保存”理解为“原轮廓图一定生成成功”。

若正式扫描在实验logger创建前失败，诊断默认写入`data/termination_<时间戳>/`；已经读取配置时遵循其日志根目录。没有绑定目录且未指定日志根目录的记录器使用系统临时目录`ur7e_termination_*`。simulation创建正式结果目录后，将诊断关联到该目录。

原目录不可写等持久化失败会被记为后续日志错误，并尝试系统临时目录`ur7e_termination_fallback_*`。如果磁盘和备用位置都不可用，只能尽力向stderr报告，不能保证文件一定落盘。空气模式拒绝重复使用已有结果目录时，诊断走独立备用位置，不覆写旧实验记录。

## 报告示例

下面只是格式示例，数值为示意，**不是本次真机运行结果**：

```text
SCAN TERMINATED
Reason: STOP_RECOVERY_EXHAUSTED
State: BOUNDARY_RECOVERY (policy=BOUNDARY_RECOVERY, phase=DONE)
Detail: boundary recovery exhausted expanded local sectors
TCP: [0.4, -0.2, 0.3, 0.0, 3.14, 0.0]
Force: [0.02, -0.01, 0.0]
Last contact: [0.403, -0.197, 0.3, 0.0, 3.14, 0.0]
Current anchor: [0.4, -0.2, 0.3, 0.0, 3.14, 0.0]
Timestamp: 2026-09-14T00:00:00+00:00
Source: policy.request_stop
Diagnostics: data/run_<时间戳>/termination.json
```

如果同时发生设备或程序异常，报告后面附上相应traceback；如果随后关闭设备也失败，另列SECONDARY，避免把关闭失败误认成扫描最初失败。

## 验证范围

本次不修改RTDE控制器、PX6D协议、标定数据、仿真物理模型、扫描距离/速度/阈值、tracking/recovery决策或安全停止条件。policy和safe_return中的修改仅用于记录已有事件；正式菜单12不新增空气验证前置要求。

离线验证覆盖原因分类、首次原因保持、故障现场、异常链、写盘降级，以及真实入口/仿真/空气适配器的行为兼容。没有据此宣称已经进行了真机验证。

最终完整回归结果（2026-09-14）：**227 passed，72.92秒**。其中包括原有扫描回归，以及未知异常、非法wrench、workspace/motion异常、返回中超力、清理异常和SystemExit等诊断测试。全程没有连接硬件。

从项目目录执行定向测试：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH= PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg \
  ../.venv312/bin/python -B -m pytest -q -p no:cacheprovider \
  tests/test_termination.py tests/test_termination_adapters.py
```

执行完整测试：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH= PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg \
  ../.venv312/bin/python -B -m pytest -q -p no:cacheprovider
```

另外已运行不连接设备的 `main.py --sensor mock` 入口检查，退出码为0，原正方形整圈验证通过。终端、诊断文件和summary均记录为`SUCCESS`，示例实测日志位于：

- [离线停止报告](simulation_outputs/full_loop_square_20260914_095728_579224/termination.txt)
- [离线结构化记录](simulation_outputs/full_loop_square_20260914_095728_579224/termination.json)

这是既有simulation的离线结果，没有连接UR或PX6D。

诊断覆盖已接管入口和运行器中的正常退出、policy终态、操作员中断及Python可捕获异常。`SIGKILL`、断电、解释器崩溃或其他未进入Python处理器的强制退出，不能由已被终止的进程保证事后写盘；这些情况只能依靠此前已写入的数据。

记录的是程序可见的停止原因。底层驱动未向外抛出或报告的失败，不能由这一日志层凭空确认；本次没有改变原RTDE停止实现，也没有增加“实际停稳已验证”的结论。
