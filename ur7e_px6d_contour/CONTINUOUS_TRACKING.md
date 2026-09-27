# 连续贴边跟踪：审查修复版

日常唯一主指南：[从这里开始.md](从这里开始.md)，入口 `python run_project.py`。

## 当前修订：力变化率保护按接触阶段启用

实际基线为 `experiment/continuous-tracking`，HEAD `edf84944e96fcf6bbeeaa3ceb98a69024dfb6847`（`edf8494`，`9_27`），修改前工作区干净。本次运行逻辑只改 `policy/continuous_tracking.py` 中 force-rate 终止保护的适用阶段和一项 telemetry；原 18 mm/s 搜索、阶段速度保护、制动余量、控制律和配置全部保持。

原问题是 `update()` 在接触判断之前无条件检查 `force_rate > 30 N/s`，导致 10 ms 内从 0.4 N 到 1.2 N 的正常接触（80 N/s）先永久停止，无法进入 FIRST_CONTACT；LOCAL_REACQUIRE 也受同一检查影响。现在仍逐帧计算/记录 force_rate，但只在 CONTINUOUS_TRACKING 的持续贴边阶段启用原 30 N/s 正向上升率终止保护。READY、TARGET_SEARCH、FIRST_CONTACT、DIRECTION_RECONFIRM、CONTACT_LOST、LOCAL_REACQUIRE 仅记录；CONTINUOUS_TRACKING 内已暂停运动的低力重确认 `_low_force_pending` 也仅记录，确认完下一周期恢复保护。STOP 继续保持终止和零命令，不消费新样本，日志保留最后有效 force_rate，不将其冒充新的测量。

18 mm/s SEARCH 就是寻找首次接触：`Fxy >= 1 N` 当帧立即 FIRST_CONTACT、零命令并请求停止，不因超过贴边的 rate guard 抢先终止，也不等待 50 ms 才开始刹车。原 contact hold 50 ms、方向确认、实测 XYZ ≤0.1 mm/s 连续 80 ms 和确认期限不改；确认成功当帧仍零命令，下一周期才能贴边。LOCAL_REACQUIRE 再次接触同样立即停止并冻结搜索参考，确认成功后才恢复贴边，不改恢复几何或引入目标真值。

所有非终止状态仍先执行原 `force_safety_reason`：processed force **12 N**、processed torque **1 Nm**、absolute raw force **60 N**、absolute raw torque **5 Nm**，达到硬阈值立即 STOP_FORCE_LIMIT，不能以接触触发放行。急停/保护停、RTDE/PX6D 故障、数据新鲜度、watchdog、30 ms 周期、边界、固定 Z/姿态、速度和人工停止均未改。原离散 force-rate 行为、菜单、返回、标定及日志线程均未改。

CSV 只追加 `force_rate_guard_active`：0 表示本样本只记录变化率，1 表示本样本适用终止保护；它保存本次判定使用的阶段，不按更新后的状态倒推。因此确认成功进入 TRACKING 的零命令帧是 0，TRACKING rate 触发 STOP 的判定帧是 1；此后 STOP 更新为 0。跨阶段不清空变化率历史，下一贴边周期立即检查。旧字段不改，新日志和删去新增列的旧格式均测试回放。

默认场景保持 18 mm/s 名义搜索，实际执行 `--dry-run --duration 15`：**2.02 s** 时 Fxy=**1.052911 N**、rate=**15.522407 N/s**、实际速度 **9.95 mm/s**，进入 FIRST_CONTACT 且 command=0、guard_active=0；**2.75 s** 时 processed force=**12.102659 N**，以 `STOP_FORCE_LIMIT / processed force safety threshold exceeded` 终止，实际速度仍 **6.5 mm/s**，尚未确认停稳或进入贴边。该场景的 rate 未超过 30 N/s；高变化率缺陷由独立 80 N/s 合成观测测试验证，不能声称默认仿真此前就是 rate 停止。没有修改模型、摩擦、减速度、预处理或阈值来取得通过。

本次仿真日志：`/tmp/continuous-force-rate-simulation/run_20260927_193535_418198`。下一次真机应重点对齐 raw/processed F/T、force_rate/guard_active、FIRST_THRESHOLD_STOP_REQUEST、零命令与实测 XYZ 速度、stop_requested/confirmed、接触/停稳计时、cycle_dt、speed_guard 和最终 termination detail，检查首次接触峰值、制动距离、停止调用耗时及恢复贴边时机。没有连接 UR/PX6D，也没有证明 18 mm/s 真机成功；菜单 12/18、Q/Esc/Ctrl+C 连续原地停止和原标定复用保持。

本次专项回归 **360 passed in 28.95s**；新增 42 项用例与加强后的默认仿真断言再次运行 **43 passed in 0.81s**。覆盖全部非终止阶段的硬 F/T、80 N/s 首次/再次接触、实测刹停和确认、贴边 rate 终止、低力暂停、跨阶段不重置历史、新旧 CSV 回放，并回归原离散、菜单、速度、执行与启动返回。旧测试将“所有阶段 rate 都必须终止”的断言替换为新的阶段断言，原硬保护断言保留。全部设备使用替身，测试夹具禁止构造真实 RTDE/serial 接口。

全量 **1032 passed, 3 failed in 199.70s**。失败仍是 `test_continuous_direction.py::test_first_turn_closed_loop_geometry_and_failures` 的 circle-0、square-0、square-30 三个历史 μ=0.20 几何案例，最大穿透 **0.032198522884 / 0.032650524729 / 0.032761858063 mm**，与此前记录一致；未改断言、未 xfail、未调模型。不能宣称全绿或 18 mm/s 贴边成功。`git diff --check` 通过；没有 commit/push/reset/stash，没有改标定或删除日志。

工程目录下复现命令（使用实际默认配置，未连接设备）：

```bash
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg MPLCONFIGDIR=/tmp/continuous-force-rate-mpl ../.venv312/bin/python -m pytest -q tests/test_continuous_force_rate.py tests/test_continuous_phase_speed.py tests/test_continuous_tracking.py tests/test_continuous_revision.py tests/test_continuous_execution.py tests/test_continuous_run.py tests/test_rule_policy.py tests/test_run_project.py tests/test_real_motion_speed_config.py tests/test_speed_limit_diagnostics.py tests/test_continuous_saved_calibration.py tests/test_continuous_startup_return.py
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg MPLCONFIGDIR=/tmp/continuous-force-rate-mpl ../.venv312/bin/python -m pytest -q
env -u PYTHONPATH MPLBACKEND=Agg MPLCONFIGDIR=/tmp/continuous-force-rate-mpl ../.venv312/bin/python run_continuous_tracking.py --dry-run --duration 15 --output /tmp/continuous-force-rate-simulation
```

控制台记录在 `/tmp/continuous-force-rate-targeted.txt`、`/tmp/continuous-force-rate-full.txt`。本次仅修改 policy、3 个测试文件和指定的 4 篇文档；下方章节的基线、改动清单和测试数字均为历史记录。

## 历史修订：TARGET_SEARCH / FIRST_CONTACT / TRACKING 执行边界

基线实际分支 `experiment/continuous-tracking`，短 SHA `f2b0ace`，修改前工作区干净。仅在原工程修改；没有连接 UR/PX6D、运动、覆盖标定、清理旧日志、commit/push/reset/stash。菜单 1～12 及其语义不变：12 原离散，15 预演，16 有限模拟，17 离线标定检查，18 连续真机（仍映射 `run_continuous_tracking.py --execute`），19/20 查看结果。直接 CLI 保留；连续 Q/Esc/Ctrl+C 原地停、不自动返回。

### 阶段与限值

- TARGET_SEARCH：连续 `search_speed` 从历史 0.001 改为 **0.018 m/s**，恢复原离散配置已有的搜索速度，方向仍来自保存的 P0→P1。保留 `command_acceleration=0.01 m/s²`，所以名义速度渐进达到；原 0.10 m / 110 s 搜索预算和 120 s 真机总预算不扩大。
- FIRST_CONTACT：原接触阈值立即触发零命令和停止请求；刹停读取保留前一阶段速度范围，执行层拒绝所有新运动。原 `contact_hold_time=0.05 s`、方向确认、实测 XYZ ≤0.0001 m/s 连续 0.08 s、1 s 确认期限全部保留。切换贴边同时要求 policy 持续停稳确认、controller stop 已接受及实际停稳；成功确认当帧仍零运动。
- CONTINUOUS_TRACKING：独立 `hypot(tangential_speed, normal_speed_limit)`，当前约 **0.001118034 m/s**，不含 search_speed。原力反馈、方向/重确认、过载减速、丢边与恢复算法不改。LOCAL_REACQUIRE 使用自己的 0.0005 m/s 上限；停止/重确认阶段沿用前一运动范围但不允许新增运动。

每个运动阶段保留 1.2 × nominal 作为轻微超限门限；增加 1.5 × nominal 的立即停止门限，并同时保留原 `1.2 × robot.max_tcp_speed` 通用实际速度硬保护（当前 0.036 m/s，命令最大仍 0.030 m/s）。1.5 是独立严重超限的工程比例，不是放大原 1.2 门限，更不是 RTDE 噪声已标定的结论。

轻微超限最多 **20 ms 或 3 个不同 RTDE 时间戳包**，先到者 fail-closed；20 ms 对应当前 100 Hz 两个名义周期。只在截止前取得新的低于门限包才清除；重复读取同一包既不计数也不清除；截止时即使刚恢复也终止，不能靠晚到数据重启窗口。阶段变化/stop 请求不会重置待决窗口。严重超速不等待确认。已有 RTDE 停滞/倒退/缺时间戳、host age、cycle timeout、watchdog 全部继续执行。不同 getter 仍不是原子包快照，不宣称严格同步。

搜索 nominal / trip / hard 为 **18 / 21.6 / 27 mm/s**；贴边分别约 **1.118 / 1.342 / 1.677 mm/s**。异常一旦确认锁定 motion_fault，runner 在故障记录/磁盘写入前请求停止。

### 边界与诊断

18 mm/s 无法继续使用原 0.5 mm 搜索余量。只在搜索与首次接触刹停期间收紧执行范围：`search_margin = max(boundary_margin, v_hard*(20 ms + 1/watchdog_hz) + v_hard²/(2*stop_deceleration))`，当前 **3.7125 mm**。同一余量从搜索距离预算扣除；P0 在这个收紧范围外时仍拒绝执行。贴边/恢复保留原 0.5 mm，返回路径和参数不改。公式是待现场验证的预算，不能保证实际停车距离。

新 CSV 仅追加 `speed_guard_phase / nominal_mps / trip_mps / hard_mps / state / elapsed_sec / count / device_timestamp`（各项都有 `speed_guard_` 前缀）。原 `current_state`、`command_speed / command_vx / command_vy` 是 policy 本周期输出；`tcp_vx/vy/vz` 与 `tcp_speed_mps` 是读到的 actual XYZ 速度，guard 字段对应同一次主循环状态读取，故过渡帧的 guard_phase 可以是旧阶段、current_state 是更新后的阶段。命令前二次读取若触发保护，完整拒绝观测另存 `termination.json.speed_limit_observation`，不能与上一有效状态或停止快照混用。发命令前也记录 policy command，使拒绝路径可诊断。

只读核对本机 ur-rtde **1.6.5** 的类 docstring：存在 Receive `getTargetTCPSpeed()`、`getSpeedScaling()`。本次未加入它们：未测量本机调用开销及包一致性，不增加设备调用。UR target speed / speed scaling 未采集，不以 policy command 冒充；日志与后台写入架构不变，旧字段/回放继续支持。

始终保留：UR 急停/保护停、RTDE/PX6D 故障、原协议/超时处理、数据新鲜度、20 Hz watchdog、30 ms cycle timeout、workspace/四角沙箱实际及预测边界、固定 Z/姿态、absolute raw F/T、processed F/T、通用实际速度上限和人工停止。force-rate 的当前适用阶段见本文开头。TCP、P0/P1、箱体四角、reset_pose 与原 data/run_* 复用，不重标定。

### 本次离线验证与限制

新增阶段速度、瞬态/持续/严重超速、新鲜包、重复包截止、接触触发、停稳门禁和边界预算测试；测试环境禁止真实 RTDE/serial 构造，设备仅替身。原几何、回放、预演长期测试显式保留其历史 **1 mm/s** 搜索配置，完整断言保留；新增默认 **18 mm/s** 仿真测试如实断言力保护停止。现有仿真减速度 0.005 m/s²，未改模型/摩擦/预处理/安全阈值，不把历史低速通过当作新速度接触成功。

专项回归 **241 passed in 27.65s**（设备替身、菜单、离散速度、启动返回及本次阶段速度/日志测试），工程目录下实际命令：

```bash
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg MPLCONFIGDIR=/tmp/continuous-phase-mpl ../.venv312/bin/python -m pytest -q tests/test_continuous_phase_speed.py tests/test_speed_limit_diagnostics.py tests/test_continuous_execution.py tests/test_continuous_saved_calibration.py tests/test_continuous_startup_return.py tests/test_continuous_run.py tests/test_continuous_tracking.py tests/test_run_project.py tests/test_real_motion_speed_config.py
```

最终全量回归：**995 passed, 3 failed in 184.05s**。3 项失败均为下面已用 HEAD 原始代码对照复现的 μ=0.20 历史几何穿透，不能宣称全绿。本次阶段速度/停止、菜单、离散、返回、旧日志兼容测试均通过。

```bash
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg MPLCONFIGDIR=/tmp/continuous-phase-mpl ../.venv312/bin/python -m pytest -q
```

另实际执行 `env -u PYTHONPATH ../.venv312/bin/python run_continuous_tracking.py --check-calibration`：退出码 0，读取当前已保存的 P0/P1、四角和本机 SDK 合同，未构造设备；输出搜索 / 贴边上限 0.018 / 0.001118033989 m/s、搜索余量 0.0037125 m。`git diff --check` 通过。以下各历史章节的测试数不属于本次结果。

历史缺陷对照：从 `git show f2b0ace:...` 读取原始 config、policy 和方向测试到 `/tmp`，替换仅用于测试的导入，再执行原三个几何断言（`/tmp/check_continuous_head.py`，无设备构造）。**3 failed, 14 deselected in 20.20s**：圆形、方形、旋转方形最大穿透分别为 **0.032198522884 / 0.032650524729 / 0.032761858063 mm**，与本次显式历史 1 mm/s 测试完全相同；保留原断言，不删断言、不用 xfail 掩盖，未改 μ=0.20 或模型。

改动文件（均在原 `ur7e_px6d_contour/`）：
- 运行逻辑/参数：`config.yaml`、`run_continuous_tracking.py`、`policy/continuous_tracking.py`、`robot/rtde_controller.py`。
- 文档：`从这里开始.md`、`操作文档.md`、`BRANCH_continuous-tracking_傻瓜式操作指南.md`、`CONTINUOUS_TRACKING.md`。
- 测试：`tests/conftest.py`、新增 `tests/test_continuous_phase_speed.py`、`tests/test_speed_limit_diagnostics.py`、`tests/test_continuous_run.py`、`tests/test_continuous_saved_calibration.py`、`tests/test_continuous_tracking.py`、`tests/test_continuous_geometry.py`、`tests/test_continuous_direction.py`、`tests/test_continuous_preview.py`、`tests/test_continuous_preview_duration.py`、`tests/test_continuous_debug_display.py`、`tests/test_continuous_friction_settings.py`。其中显示/几何测试只显式标记历史低速配置；旧离线标定测试改为比较实际已保存 P0，不再硬编码过期现场坐标。


下一次真机仍须验证：18 mm/s 的加速与实测波动、触发后的制动尾段/峰值力、实测停稳后低速恢复、guard 误报/持续超速响应、边界实际停车余量，以及原 `speedStop` 调用在较高搜索速度下是否能满足原 30 ms 周期。停止 API、返回路径和此前 45.480 ms 下降问题本轮不重新设计；若停止调用超时仍终止，不放宽周期或 watchdog。

## 历史修订：下降停稳周期与扫描启动衔接

核对 `run_20260921_132757_660403`：返回已进入 `DESCEND_TO_START_SETTLE`，失败周期 45.480 ms；从周期开始到 TCP 时间戳约 3.129 ms，额外约 42.35 ms 出现在此后的处理/写日志/调度期间。现有记录不能将这部分耗时全部归因于磁盘，但同步双 CSV 写入位于该路径，已用阻塞写入替身复现其占用控制预算的问题。此前 `13:27` 的连接快照确实记录 protective_stopped=true；这不等于 `13:28` 的下降超时也是同一种连接故障。

新增 `ContinuousLogWriter` 仅包装连续真机入口的原 `ExperimentLogger`，深复制样本、事件和返回结果，单独写入原文件格式。64 条队列包含正在写入的记录，积压上限使用原 confirmation_timeout_sec；失败、满队列或最老记录过期即拒绝后续健康周期。控制线程仍负责实时终止观测，磁盘线程不读取设备、不喂狗，旧排队样本不会覆盖最新诊断。停止设备之后分别尝试最终事件、停止快照与摘要，关闭文件前排空已接受记录；不可解除的系统写入阻塞有界返回错误并由线程延后清理，不宣称全部日志已经保存。原离散日志及仿真入口不改用线程。

最终下降原先用返回 0.5 mm 到位条件触发刹停，随后扫描却要求 0.2 mm；最新日志的约 0.433 mm 残差因此还会触发下一处拒绝。现仅连续最终下降使用两者中更严格的容差，停稳后再次检查；其他返回段、离散返回和校准点保持原值。返回预检、返回后的零偏和跟踪在 kickWatchdog 返回后补查原周期/样本时限，避免调用本身延迟后继续依据旧观测发命令。

回归先复现同步日志阻塞、45 ms kick 后错误放行下一段，以及下降在 0.433 mm 提前停止。组合入口测试使用完整 runner 和设备替身，模拟首条磁盘写入延迟 45 ms、下降 0.433 → 0.19 mm、重新采扫描零偏、进入 TARGET_SEARCH、有限时限停止，并核对两份 CSV 全部记录一致。保护阈值、控制律、实际标定、传感器超时及摩擦参数均未修改。本轮未连接设备，未验证真机调度、路径或现场保护恢复。

独立审查补齐了事件部分入队失败时仅重试未接受后缀，以及首个文件关闭失败时仍由磁盘线程逐个清理其余文件；首次错误保留。最终实际回归 **248 passed in 18.14s**，包含隔离单元/设备替身、伪终端及既有有限时长闭环仿真和三个入口的数据一致性测试；没有运行全量套件或新的长期几何验收。命令如下，测试全部显式有限终止，未连接机器人/传感器：

```bash
MPLCONFIGDIR=/tmp/continuous-mpl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv312/bin/python -m pytest -q tests/test_continuous_writer.py tests/test_data_logger.py tests/test_continuous_startup_return.py tests/test_continuous_run.py tests/test_continuous_execution.py tests/test_continuous_connection_cleanup.py tests/test_continuous_saved_calibration.py tests/test_safe_return.py tests/test_safe_return_termination.py tests/test_controller.py tests/test_real_policy_parity.py tests/test_app_termination.py tests/test_continuous_revision.py tests/test_px6d_reader.py tests/test_operator_input.py tests/test_termination.py
```

`git diff --check` 通过；保存标定文件、连续策略源码、原离散入口及共享日志类无差异。保留此前本地修改及实验数据，未 commit/push。

## 历史修订：现场总线报警相关的退出看门狗生命周期

最新两次 `13:13` 日志在 RTDEControlInterface 构造期间报告控制脚本 5 秒启动超时，尚未进入返回逻辑；用户随后报告示教器提示“检查现场总线连接”，尚未提供精确报警号。UR 官方 [C207 说明](https://www.universal-robots.com/manuals/EN/HTML/SW10_7/Content/prod-err-codes/topics/CODE_207.html) 明确列出 RTDE 看门狗；不能仅凭终端超时判定具体保护原因，也不能断言任何现场总线报警都来自本程序。

已确认的代码缺陷：终止收尾不再踢 20 Hz 看门狗，却继续等待至少 80 ms 的停稳确认，超过其 50 ms 周期。现改为停止运动、取得新鲜的低速观测后，先用本机 SDK `stopScript()` 结束本程序启用看门狗的控制脚本，再完成连续停稳确认和断开接口。正常终止、异常停止及返回中止均在终止打印/写日志之前进行此处理，避免终止 I/O 再占用看门狗期限；成功返回和非终止暂停不结束脚本。UR 的 [watchdog 官方说明](https://www.universal-robots.com/manuals/EN/HTML/SW10_14/Content/prod-scriptmanual/all_scripts/rtde_set_watchdog_variable_name.htm) 指出程序停止时移除其看门狗。仍在移动或无法取得新鲜状态时不移除保护；不增加后台续期，不修改频率、力限、速度或保护设置。`close()` 也采用同样的幂等收尾；原离散入口没有启用该看门狗，其关闭行为不变，共享返回仅增加默认空操作的终止扩展点。

连接失败时，先从已存在的 receive 接口保存机器人模式、安全/程序状态、状态位、TCP 等故障快照，再关闭接口；每项读取失败独立保留未知及错误，不覆盖原异常，也不把快照作为实时状态或停稳证明。连续入口打印并记录该快照；确认连接初始化失败且接口已关闭时，不再二次读取已关闭接口制造 `receive interface is not connected` 错误。没有新增设备连接或自动恢复动作。

验证使用 SDK/时钟/状态替身与原有有限仿真，覆盖正常收尾先结束脚本再等待、移动/不可读状态保留保护、脚本结束失败仍关闭接口、幂等、原离散关闭行为，以及连接失败快照和原始原因保留。最终十文件回归 **163 passed in 18.18s**：`tests/test_continuous_connection_cleanup.py tests/test_continuous_run.py tests/test_continuous_startup_return.py tests/test_continuous_execution.py tests/test_controller.py tests/test_app_termination.py tests/test_safe_return_termination.py tests/test_safe_return.py tests/test_real_policy_parity.py tests/test_continuous_revision.py`，沿用前述 pytest 环境前缀。另补充/强化终止 I/O 前结束脚本的两项专项测试，**2 passed / 65 deselected in 0.50s**（与上一批重叠一项）。包含角速度非零、设备包停滞及诊断 getter 不可用的拒绝/保留原异常测试。没有连接硬件，没有复位当前报警；现场根因及恢复效果仍需与示教器具体报警核对。未 commit/push。

## 历史修订：分段返回的非阻塞刹停与连续停稳确认

`run_20260921_130905_900053` 最后一个 `VERTICAL_RETREAT` 周期到下一段 `MOVE_ABOVE_START_PRECHECK` 相隔约 40.87 ms；原分段执行器调用阻塞 `stopL`，连续适配器因此触发原 30 ms 周期间隔限制。先用 40 ms 阻塞停机替身复现同一错误，并复现未停稳时直接尝试下一段的问题。

共享 `SafeReturnExecutor` 只增加 `_stop_at_segment_end` 扩展点，默认仍按原顺序调用同步停止和状态读取，原离散返回行为不变。连续子类改用本机 ur-rtde 1.6.5 已核对签名的 `stopL(a, asynchronous=True)`，随后在原线程持续执行力/TCP 采样、日志与时序检查。新增 `*_SETTLE` 阶段要求实测速度连续满足原 `settle_speed_mps` 和 `settle_hold_sec`，中断则重新计时，等待受原 `confirmation_timeout_sec` 限制；原执行器继续校验最终目标位置/姿态。停止请求成功不等于停稳，也不允许发送下一段 moveL。

看门狗健康检查与新运动授权检查分开：健康采样可在刹停期间续期，但新运动仍受“停稳未确认”限制；已经超期的看门狗不能通过重置时间恢复。没有清空周期历史、增大时序阈值、后台无条件喂狗或改变返回路径、减速度及跟踪公式。停止请求拒绝、刹停中力/数据/日志异常、持续未停稳和键盘停止都阻止后续分段。

实际测试：十文件回归 **163 passed in 15.55s**：`tests/test_continuous_startup_return.py tests/test_continuous_execution.py tests/test_continuous_run.py tests/test_safe_return.py tests/test_safe_return_termination.py tests/test_controller.py tests/test_real_policy_parity.py tests/test_app_termination.py tests/test_continuous_saved_calibration.py tests/test_continuous_revision.py`；随后新增停稳确认中断重置测试单独 **1 passed / 39 deselected in 0.53s**。均使用离线替身或原有有限仿真，未调用设备构造连接；`git diff --check` 通过。未做真机验证或全量/长期几何测试，未 commit/push。

## 历史修订：启动返回零偏采集的有限超时恢复

针对 `run_20260921_130152_573202` 在 `STARTUP_RETURN_BIAS` 的 PX6D 50 ms 超时，新增 `PX6DTimeout`（继承原 `PX6DError`），仅在连续入口返回前已确认停稳的临时零偏窗口捕获该类型。再次请求停止并检查新鲜 TCP、位置/姿态和速度后，清除残留缓存并用现有版本查询响应重新同步；不重连串口、不触发硬件清零。丢弃恢复后的首帧，清空此前零偏样本及滤波状态，重新取得全部有效样本后才允许返回。

最多两次重试，第三次超时终止；总窗口预算为 `startup_bias_sample_count / poll_rate_hz + confirmation_timeout_sec`，重同步超时计入同一预算。记录写入本轮 `startup_bias_retries.json`，写入失败仍终止。断开、非有限数据、超载、实测移动、过期状态及人工取消不被吞掉。返回及跟踪仍调用原不重试的 `read_wrench()`，原 50 ms 超时和运动时序阈值不变。原离散策略、共享返回路径及标定文件未修改。

先补五项失败测试后实现；最终相关九文件回归 **144 passed in 13.25s**：`tests/test_px6d_reader.py tests/test_continuous_startup_return.py tests/test_continuous_run.py tests/test_safe_return.py tests/test_safe_return_termination.py tests/test_app_termination.py tests/test_termination.py tests/test_continuous_execution.py tests/test_operator_input.py`，使用 `MPLCONFIGDIR=/tmp/continuous-mpl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv312/bin/python -m pytest -q`。覆盖短暂超时后重新采集完整窗口、持续超时/重同步超时次数限制、总时限、运动时禁止重试、移动和断开拒绝、人工取消及原异常类型兼容；包含原有有限时长仿真和终端回归。`git diff --check` 通过。

本次为软件恢复行为修复，所有新增串口/机器人交互均使用替身验证；未连接硬件，未验证现场 USB 链路稳定性，未运行全量测试或长期几何验收。未 commit/push，旧实验日志保留。

## 历史修订：连续入口复用原工程启动安全返回

后续输入修复：启动返回确认原先位于 `OperatorKeyboard` 的 cbreak 上下文内，`tty.setcbreak()` 关闭 ECHO，导致输入没有可见回显。新增 `read_line()` 在确认期间临时启用行输入/回显，支持退格和回车；正常返回、Ctrl+C、EOF 后均恢复逐键监听，外层退出恢复原终端。连续入口使用该方法，原离散调用不变。真实伪终端子进程测试验证输入在回车前可见、退格、取消、EOF、后续无需回车的 Q 以及最终终端恢复；不是硬件测试。相关四文件回归 **61 passed in 11.56s**：`tests/test_operator_input.py tests/test_continuous_startup_return.py tests/test_continuous_run.py tests/test_app_termination.py`，使用相同 pytest 环境前缀；`git diff --check` 通过。未连接硬件。

针对连接后 `TCP z drift +0.029567 m exceeds tolerance`，连续入口现在使用原控制器的 `allow_start_away_from_fixed_pose=True` 连接方式；实际 active TCP 和沙箱 XY 检查仍执行。离开 P0 时复用 `SafeReturnExecutor` 的安全高度计算和分段 moveL 路径，原 `safe_return` 参数不变，未修改 `app/main.py`、离散策略或共享返回算法。返回目标仍来自保存的扫描标定；跟踪仍使用原连续策略和 speedL。

需要返回时，将已有的一次 START 确认提前到返回之前，然后确认实测停稳、采集独立临时偏置、执行受力监控的返回；完成后重新检查 P0 和固定姿态，再采集扫描偏置、开始跟踪。已在 P0 时仍跳过返回并保留原确认流程。返回期间启用连续看门狗，仅通过新鲜、未超时且力检查及日志写入成功的周期续期；扫描零偏阶段继续监控并续期，不增加后台无条件喂狗线程，不在看门狗已启动时阻塞等待第二次 START。

连续入口的返回适配器只补充时序、非有限数据和键盘停止检查；返回轨迹、分段超时、力限及保存状态仍由原执行器负责。控制器在返回模式按原机器人最高速度校验；退出返回模式立即恢复较低的连续跟踪速度限制。moveL 目标及当前 TCP 均检查已有沙箱多边形，并要求返回模式、RTDE 包进展和有效看门狗。启动前校验返回配置，配置摘要新增返回参数。返回期间的 Q/Esc/Ctrl+C 或异常均终止本轮，不转入扫描；停止后不自动返回。

新增离线测试先得到 9 项失败，随后实现并覆盖：29.567 mm 偏高时复用原安全高度跳过逻辑、低于安全高度的三段返回、已在 P0、参数摘要/有限值、沙箱越界拒绝、返回/跟踪速度隔离、力异常/NaN、过期数据、停稳超时、串口/日志超时、日志失败、看门狗失败，以及完整入口返回后重新采集扫描偏置、只确认一次并运行共享策略。所有设备均为内存替身，有限步数或明确短时限。

实际回归 **146 passed in 13.14s**，命令：`MPLCONFIGDIR=/tmp/continuous-mpl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv312/bin/python -m pytest -q tests/test_continuous_startup_return.py tests/test_continuous_execution.py tests/test_continuous_saved_calibration.py tests/test_safe_return.py tests/test_safe_return_termination.py tests/test_controller.py tests/test_continuous_run.py tests/test_real_policy_parity.py tests/test_app_termination.py tests/test_continuous_revision.py`。包含隔离单元/设备替身测试及原有有限时长离线仿真；未运行全量测试或长期闭环几何验收。`git diff --check` 通过。

未连接机器人或传感器，未验证真机路径、碰撞余量或机械停稳效果；未改变旧标定、离散策略、连续控制公式和配置数值。本节替代下文历史版本中“连续入口不自动返回 P0”的说明。

## 历史修订：连续入口复用已有扫描点和沙箱标定

按本轮用户明确要求，连续真机入口直接沿用旧工程的 `scan_calibration.yaml` 和 `workspace/config/workspace_calibration.yaml`，不再要求默认为空的 `site_verification` 作为第二份启动记录。以下旧章节关于“缺省记录为空即拒绝连接”的描述为历史行为，以本节及操作指南为准。没有生成虚构核对人/时间/通过标记；显式提供的额外记录仍校验完整性和配置摘要，实验性恢复仍需独立记录、默认关闭。

`prepare_real()` 在任何设备构造前调用已有扫描/沙箱校验器，核对有效数据、已有机器人 IP/TCP 元数据、扫描 P0/P1 的距离和 Z 约束。两路径相对 `--config` 所在目录，沙箱路径由 `continuous_tracking.workspace_calibration_file` 指定。配置摘要现在包含两份文件内容；配置日志保留标准化扫描记录、完整实测沙箱四角及原始采集元数据、来源路径和有效执行范围。旧沙箱 `active_tcp_verified: false` 如实保留，运行时仍由原控制器检查实际 active TCP。

当前扫描 P0：`[0.6211867726148192, 0.25265294809033256, 0.09310900761687047, 1.513281627715119, -2.7528827770096442, 0.00068325703538375]`；搜索方向 `[-0.04853710860316569, -0.9988213799716366]`。沙箱角点 P0..P3 与扫描 P0/P1 是不同记录。固定 Z/姿态只取扫描 P0，不使用沙箱平均 Z 或坐标轴替换。

沙箱原始四角 Base XY 用于执行范围，原 `boundary_margin` 不变；显式额外矩形限制同时生效。RTDE 连接、实际状态读取及下一步预测通过同一多边形检查，拒绝“虽在包围盒内但越过斜边”的位置。沙箱几何只作边界保护，不输入策略找目标边。无连续边界配置的原离散控制器路径保持原行为。

正式命令仍为 `../.venv312/bin/python run_continuous_tracking.py --execute`。**本次接入标定数据，没有加入自动返回 P0；当前 TCP 仍须已在保存的扫描 P0 并静止。** 原连续零偏、START、watchdog、停稳及原地停止流程保留。首次接触后仍直接连续跟踪，不恢复三点初始化，不修改力控、方向估计、速度或任何阈值。

新增 `--check-calibration`：读取文件并执行离线配置/SDK 检查、打印加载结果，然后返回，不构建设备、不触发运动；实际已执行通过。保留的结果：[calibration_check.json](simulation_outputs/saved_calibration/verification_20260921_121034_570607/calibration_check.json)。两份旧标定文件与 `app/main.py`、`policy/continuous_tracking.py` 均无差异。

测试先复现 **3 failed / 7 passed**，实现后覆盖旧点位复用、扫描 Z 与沙箱 Z 不混用、相对路径、文件不被改写、无效标定/IP/TCP/范围拒绝、显式记录拒绝、恢复默认限制、四角摘要绑定、斜边实际/预测越界拒绝及只读 CLI 不构建设备。最终新增及连续执行测试 **23 passed in 0.40s**。此前相关六文件 **120 passed in 20.36s**；连续/旧返回测试 **39 passed in 0.49s**；控制器、原策略适配、旧应用收尾及连续修复回归 **60 passed in 3.05s**（各批有重叠）。命令均用 `MPLCONFIGDIR=/tmp/continuous-mpl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv312/bin/python -m pytest -q`。`git diff --check` 通过。

这些是文件校验、隔离及设备替身测试；本轮未运行真机或全量测试，也未重做长期闭环几何验收。此前 μ=0.20 的几何穿透问题未在本次修改中处理。未连接硬件、未 commit/push/reset、未改写旧标定或删除实验数据。

## 历史修订：fix5 后统一 Debug 样式、图例和真实切向对照

起点为 `experiment/continuous-tracking`、HEAD `c6f8fc7`（fix5），工作区干净，与该基准无后续差异。生产修改仅 `simulation/continuous_view.py`、`simulation/continuous_preview.py`、`tools/visualize_continuous_run.py`；控制策略、方向估计、传感器/摩擦模型、仿真及真机配置未变。

共享 `DISPLAY_STYLES` 定义颜色、线型、名称、单位，箭头、轨迹、右侧曲线、代理图例及分量说明复用。法向蓝实线，机器人估计红虚线，处理后合成力橙实线，估计切向绿色，朝内方向青色，命令黑色，实际速度紫色，轨迹深灰细线。Components 使用土黄点线、棕虚线、灰蓝点划线、粉点线。简洁模式不增加图例/辅助箭头；Debug 也保留单色轨迹，不叠加十字、重复标记和状态色路径。

左侧 Debug 图例用明确的箭头/线段代理，在 XY 上方预留区域显示。只有启用项目集合或仿真/真机标签上下文改变时重建，普通帧只更新文本与“零 / 不可用”状态；关闭 Debug 隐藏整个图例。新增真实切向按钮默认关闭：预演在 Debug + Settings 中，回放在 Debug 底部，也支持 `--true-tangent`。只把已有可信物理法向日志转成短灰虚线，估计 tangent 不作几何修正。夹角采用 `acos(abs(dot))`，为无方向直线 0～90°，不用于判断倒退。无接触/法向缺失不画线；真机隐藏按钮并注明无真值。

先补的两项显示测试在修改前失败。随后新增有限闭环比对，圆形及 30° 斜直边各 3000 步，开关 Debug / Components / 真值对照前后的时钟和 RNG 状态不变，raw、processed、控制方向、速度、move、状态逐值一致；运行中与停止帧的预演/回放箭头数值、颜色、线型、名称及真实切线一致。新增测试同时覆盖代理样式、缓存身份、零/缺失标注、按钮切换、真机无真值隐藏、16×9 / 12×8 英寸下图例及文字边界。

实际测试：六个相关文件（`test_continuous_debug_display.py`、`test_continuous_force_display.py`、`test_continuous_friction_settings.py`、`test_continuous_preview_duration.py`、`test_continuous_direction.py`、`test_continuous_preview.py`）在 `-k 'not first_turn_closed_loop_geometry_and_failures'` 下 **105 passed / 5 deselected in 32.27s**；最后补充按钮和真机日志测试后，`test_continuous_debug_display.py tests/test_continuous_run.py` **29 passed in 16.80s**（与前轮有重叠）。命令前缀为 `MPLCONFIGDIR=/tmp/continuous-mpl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv312/bin/python -m pytest -q`。回放 CLI 帮助和 `git diff --check` 通过。

首次含 180 秒几何用例的回归仍复现前轮三项穿透：圆形 0.03220 mm、方形 0.03265 mm、旋转方形 0.03276 mm，数值不变，未修改安全断言；同轮另两项失败为旧测试假定简洁模式也创建隐藏图例，现已改为检查不存在可见图例，并复测通过。后续没有重复这五项长几何用例；本轮未跑全量测试，也没有解决已知几何失败。

最终截图和完整日志在 [verification_20260921_094639_943208](simulation_outputs/debug_display/verification_20260921_094639_943208/)，汇总见 `verification.json`。圆形与 30° 斜直边各运行 30 秒（40 秒明确预算），暂停在 29.99 秒生成预演/同帧回放，随后显式停止。每种均输出 `preview/replay_16x9/12x8_simple/debug.png`，已实际打开检查两种窗口尺寸下的简洁及 Debug 图；另查看 `controls/run_20260921_094911_043996/controls_12x8.png` 的 Debug + Settings + 摩擦设置组合。图例与状态、力数值、按钮不重叠，修复小窗口 Y 标签裁切及回放右图图例过宽。实测估计切向与真实切线偏差分别约 12.091°、12.150°，显示保留该差异，不代表几何验收通过。

没有连接硬件或实机绘图，未手动点验真实桌面；检查使用离屏 GUI 回调、截图、有限仿真和设备替身。未 commit/push/reset，没有删除或覆盖旧实验数据。

## 历史修订：蓝色只显示法向反力

本轮从 `experiment/continuous-tracking`、HEAD `51f855f`、干净工作区开始。生产修改仅涉及 `simulation/continuous_view.py`、`simulation/continuous_preview.py` 和 `tools/visualize_continuous_run.py`；模型、策略、传感器、配置和摩擦参数均未修改。

蓝色标签为“法向反力 / Normal reaction”，直接读取已有的 `sim_normal_physical_fx/fy`，即模型物理外法向乘实际法向载荷。圆形取径向，直边取垂线，尖角沿模型有效法向；不重新投影合力，不用合力模长替代法向载荷。零接触不画蓝箭头，缺失法向字段显示 unavailable。红色仍读取 `sim_robot_estimate_fx/fy = -(normal_physical + friction + background)`，保留准静态及瞬态限制。默认添加“法向反力示意；摩擦分量未绘制”，两箭头仍共用 8 mm/N；Debug 显式打开分力时不显示“摩擦分量未绘制”。真机合力的标签、物理符号可信检查及不可用处理不变，不猜法向。

修改前新增的五项用例均按预期失败；修改后增加三角尖角、两种 μ 的控制独立性及缺失数据检查。隔离测试覆盖朝外、与切线正交、实际法向载荷、相同标尺、完整红色外力估计和零接触。μ=0.03 / 0.20 各做 1150 步有显示/无显示共享策略比对，raw、processed、方向、速度、move 和状态逐值一致，模型摩擦参数保持不变，预演与回放同采样箭头一致。

最终相关回归 **89 passed in 26.38s**：`MPLCONFIGDIR=/tmp/continuous-mpl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv312/bin/python -m pytest -q tests/test_continuous_force_display.py tests/test_continuous_friction_settings.py tests/test_continuous_preview.py tests/test_continuous_preview_duration.py`。全部使用有限步数或明确终止条件。`git diff --check` 通过；源码差异核对确认策略、传感器、会话控制及配置文件未变。

实际有限闭环演示：圆形、方形各运行 30 秒（配置总预算 40 秒），显式暂停截图后手动停止保存日志。截图为 29.99 秒的同一采样，蓝色法向载荷分别 1.85600 N、1.85679 N；红色仍包含表面摩擦及背景阻力。已实际查看 [圆形预演](simulation_outputs/normal_reaction/verification_20260921_091241_164771/circle/run_20260921_091241_432122/circle_preview_small.png)、[直边预演](simulation_outputs/normal_reaction/verification_20260921_091241_164771/square/run_20260921_091243_842455/square_preview_small.png)、[圆形回放](simulation_outputs/normal_reaction/verification_20260921_091241_164771/circle/run_20260921_091241_432122/circle_replay.png)、[直边回放](simulation_outputs/normal_reaction/verification_20260921_091241_164771/square/run_20260921_091243_842455/square_replay.png)。力标签、底部说明及按钮无重叠；同目录保留完整日志，汇总见 [verification.json](simulation_outputs/normal_reaction/verification_20260921_091241_164771/verification.json)。

本轮未重跑全量及 180 秒几何验收；前轮 μ=0.20 的三项几何失败仍未修复，截图不能作为几何验收通过的证据。未点验真实桌面或连接真机，未 commit/push；离屏 GUI 回调与闭环演示不等价于实机验证。


## 历史修订：表面摩擦图形设置及默认 μ=0.20

在当前 experiment/continuous-tracking 工作区保留前轮未提交修改。本轮生产改动仅 `simulation/continuous_preview.py` 与 `simulation/scene_continuous.yaml`：连续场景独立覆盖表面 friction_coefficient=0.20，公共 `simulation/simulation_config.yaml`、真实配置和控制策略没有修改。新建默认预演及无窗口入口读取同一场景；显式用户场景尊重其 μ，原三角形复现场景继续为 0.03。

底部右侧“摩擦设置”旁显示当前生效 μ，点击展开右侧紧凑输入区，输入或 ±0.01 只编辑草稿。应用后调用既有新场景流程：先校验/创建候选会话，旧轮次按 FRICTION_CHANGED 记录前后数值并关闭日志，再回到 READY；下一次 Start 才开始。目标/起点/其他环境参数保留，接触记忆、滤波和策略实例均重建。μ 实际进入既有传感器摩擦公式，不改 granular_drag_force，不改增益、方向重确认、速度或保护。输入区暂时替代 Debug 的速度面板，不挤压绘图区。

新增 `tests/test_continuous_friction_settings.py`：修改前 **12 failed / 1 passed**，实现后初批 **13 passed in 4.30s**。覆盖连续专用默认、0.03 精确输入、保存/加载、非法值、READY/RUNNING/PAUSED 应用、旧日志保留、新会话重置、无窗口参数/反馈一致性及相同载荷下 μ 改变真实模型摩擦分量。补充实际按钮回调、输入数字不触发形状快捷键、展开区域与原文字不重叠的检查。全部运行有有限时长或有限步数。

实际生成并查看：[待应用 0.03、当前仍为 0.20](simulation_outputs/friction_editor/run_20260921_082052_057337/friction_draft.png)、[小窗口 Debug/Settings](simulation_outputs/friction_editor/run_20260921_082052_057337/friction_small_debug_settings.png)、[应用后回到 READY](simulation_outputs/friction_editor/run_20260921_082052_057337/friction_applied_ready.png)。原 μ=0.20 轮次日志保留在同目录，`scene_mu_003.yaml` 是实际保存的 0.03 场景；重新 Start 的 0.03 日志在 `simulation_outputs/friction_editor/run_20260921_082053_665003/`。

最终全量命令：`MPLCONFIGDIR=/tmp/continuous-mpl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv312/bin/python -m pytest -q`，结果 **776 passed / 3 failed in 186.73s**；包含上述新增 13 项全部通过。全量验证仍复现下列三项几何失败，其余测试通过。

**新默认参数下有几何回归失败，未修改断言或掩盖。** 相关测试曾得到 **3 failed / 120 passed in 72.54s**。180 秒闭环实际结果：

| μ=0.20 场景 | 最大中心穿透 | 最大压缩 | 跟踪反馈范围 | 真实终止 |
|---|---:|---:|---:|---|
| 圆 | 0.03220 mm | 1.03220 mm | 1.3024–1.9052 N | STOP_TIME_LIMIT |
| 方形 | 0.03265 mm | 1.03265 mm | 1.3027–1.9063 N | STOP_TIME_LIMIT |
| 平移、旋转方形 | 0.03276 mm | 1.03276 mm | 1.3029–1.9061 N | STOP_TIME_LIMIT |

压缩超过 1 mm 模型包络，故不能把这些轨迹视为通过几何验收。失败记录及评分已原样复制保留在 [failed_geometry_reports.json](simulation_outputs/friction_editor/geometry_mu020_20260921_082255/failed_geometry_reports.json) 所列目录。本轮不调整控制律/保护/速度、不将 μ 偷改回 0.03，也不将测试改成接受穿透；默认参数变化带来的这项模型行为仍需后续独立处理。

未连接真机、未手动点验真实桌面；图像和 GUI 回调是离屏检查，μ 不是实机标定值。未 commit、push、reset 或删除实验数据。

## 历史修订：预演默认手动停止及底部排版

保留上一轮双力演示的未提交修改。`--preview` 在深复制的本次仿真配置中将 `continuous_tracking.max_runtime_sec` 设为 **None**，配置快照为 YAML **null**。共享策略允许 None 且只跳过总运行时限比较；原文件 `config.yaml` 的 120 秒和所有控制/保护参数不变。没有增加绕圈完成判定；正常跟踪持续到用户停止或保护触发，不以“过圈”作为退出条件。

`--preview --duration 180` 是明确有限的 180 秒仿真，可以大于原 120 秒；参数必须有限且为正。无窗口保留原默认预算，真机在设备/现场准备之前明确拒绝 null、非有限或非正总预算，原配置摘要和现场验证要求保留。预演模型直接用于测试/批量时仍尊重显式配置预算；仅 CLI 预演默认改为 None，避免既有有限批量用例变成无限循环。

Q / Esc（包括路径框有焦点时）、Stop、关闭窗口和 Ctrl+C 都按用户停止收尾；重复关闭/停止不覆盖首次原因。磁盘写入异常仍走原异常停止与 finally 日志关闭流程。日志每 20 个采样刷新，不把全历史留在内存；显示缓存仍最多 3001 个最近采样、4096 个路径点、200 个事件（默认 100 Hz）。完整 CSV 持续增长，未自动编码视频。

底部多个重叠的文字图元改为一个随窗口宽度换行的说明区；按行数给坐标轴和 Settings 控件预留间隔，Debug 的时间轴仅在下方显示标签。回放说明区也避开 Debug 按钮。顶部显示“仿真时间：xxx秒｜手动停止模式”，具体状态和停止原因集中在其下一行。有限模式明确显示时限。

新增 `tests/test_continuous_preview_duration.py`。修改前复现 **13 failed / 5 passed**；初始 18 项修复后全部通过，追加不限时保护与写盘失败测试后，相关回归 **92 passed in 26.02s**。覆盖默认 None/有限时长/配置不被改写、130 秒真实几何闭环仍在跟踪、有界缓存与持续落盘、真机前置拒绝 null、力/力矩/无效数据/采样过期/方向超时/搜索耗尽保护，以及手动停止、Ctrl+C 和故障清理。所有自动运行都用有限步数、显式时限或注入停止事件，没有等待不限时预演自然结束。

全量回归 **765 passed in 184.23s**。最后补充启动零偏采样中 Ctrl+C 应记为用户停止（而非传感器失败），并完成时间轴排版小修后，四个相关文件复测 **93 passed in 26.25s**；此次小修后没有重复全量测试。新增本轮测试共 26 项。实际命令使用 `MPLCONFIGDIR=/tmp/continuous-mpl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv312/bin/python -m pytest -q`；定向复测指定 `tests/test_continuous_preview_duration.py tests/test_continuous_force_display.py tests/test_continuous_preview.py tests/test_continuous_run.py`。CLI `--help` 及 `git diff --check` 通过。

离屏闭环记录与最终布局截图：[运行目录](simulation_outputs/manual_preview/verified_layout/run_20260921_073614_811746/)。[130 秒仍在运行](simulation_outputs/manual_preview/verified_layout/run_20260921_073614_811746/manual_running_130s.png)，[小窗口展开 Debug / Settings](simulation_outputs/manual_preview/verified_layout/run_20260921_073614_811746/manual_debug_settings_small.png)，均已实际查看。目录保留配置 null、完整采样、停止报告和缓存统计；停止前状态 CONTINUOUS_TRACKING，测试显式手动停止后为 STOP_USER_REQUEST。早一轮布局检查截图也保留，未覆盖实验数据。

本次没有真机连接或真实桌面手动点验；离屏截图/回调及合成闭环不等价于设备物理验证。没有 commit、push、reset 或删除实验数据。

## 历史修订：18ef729 fix3 之后的简洁双力演示

本轮从 `experiment/continuous-tracking`、HEAD `18ef729`、干净工作区开始。控制策略、配置阈值、方向重确认、低力暂停、恢复开关、speedL 及原离散策略未改；共同传感器只追加诊断字段，旧 raw/processed 和随机样本保持不变。以下旧版方向修复的可视化说明是历史记录，当前界面以本节及分支傻瓜式指南为准。

- 蓝箭头为 `F_boundary_physical = -object_force + friction_force`，物理法向朝外，表面摩擦仍反向于滑动。不是旧 inward 合成合力取负，也不固定到 F_ref。
- 红色虚线为 `F_robot_estimate = -(F_boundary_physical + background_force)`；本模型无其他已知平面外力。measurement noise 不进入物理平衡。它是忽略平面惯性的夹具作用力估计，不是速度型导纳输出的力命令或真实电机驱动力。
- 新 `force_display` 配置元数据记录 schema_version=1、Base 坐标、明确的旧 inward 到物理 outward 转换、来源、估计方法和可用性。CSV 保留旧分量，同时追加 normal/boundary/environment/robot_estimate 的 XY 及 physical_force_available。诊断真值不进入 policy。
- 真机入口只追加数值可用性与来源，不绘图。现有控制方向核对不等于物理作用对象/符号确认，故明确记录 unconfirmed。回放蓝标签自动为“测得环境合力”；只有另有可靠物理坐标/符号与标定记录时才显示合力及其反号估计，继承测量噪声不确定性。旧日志缺少这些字段仍能打开，物理箭头不可用，不推测分力。
- 默认约 75/25，灰色目标、小实心 TCP 点、单色执行轨迹、两根同标尺力箭头和一个 1 N 标记；右侧仅控制反馈 Fxy/F_ref。Debug 默认关；Settings 折叠形状、倍速、场景文件等次要控件。回放默认 PNG，视频需显式 `--format`。

修改文件：`simulation/simulated_force_sensor.py`、`simulation/continuous_session.py`、`simulation/continuous_view.py`、`simulation/continuous_preview.py`、`run_continuous_tracking.py`、`tools/visualize_continuous_run.py`；新增 `tests/test_continuous_force_display.py`，更新两处旧显示测试及本指南/操作指南。

新增前先运行 7 项测试，全部按预期失败；实现后定向回归 **79 passed, 5 deselected in 17.89s**。测试覆盖法向/摩擦符号、物理/旧合成求和、排除噪声、零接触且有阻力、加载/卸载、同标尺翻倍、未知值、默认与调试切换、逐周期控制独立及同时间戳预演/回放一致性。显示切换使用既有记录，测试核对 RNG 不前进。

最终全量 **740 passed in 179.44s**，前述五个几何用例也全部执行通过。命令：`MPLCONFIGDIR=/tmp/continuous-mpl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv312/bin/python -m pytest -q`。新测试共 10 项；旧测试仅更新布局/图元断言及显式视频导出请求，保留安全断言。回放 CLI `--help` 和 `git diff --check` 通过；GIF 显式导出及编码失败回退有自动测试，未验证真实桌面或成功 MP4 编码。

独立保存修改前原三角形 **12,001 周期**的 raw、processed、TCP、状态和命令；修改后逐值精确一致，无数值容差放宽。不是新动力学验证。全量仍保留圆/方/三角形/平移旋转几何、方向重确认、低力及安全回归。

实际截图与结果位于 `simulation_outputs/dual_force_demo/run_20260921_070749_157213/`：

- [简洁预演截图](simulation_outputs/dual_force_demo/run_20260921_070749_157213/simple_preview_stable.png)及[简洁回放截图](simulation_outputs/dual_force_demo/run_20260921_070749_157213/simple_replay_stable.png)均已实际打开检查。两者分别取 79.99 s 和 80.00 s；同一采样时刻的箭头数值一致性另有自动测试。
- 同目录 `continuous_preview.png` / `continuous_summary_target.png` 是停止后的图，`display_validation.json` 记录基准比对及两种窗口尺寸的 XY 等比例检查，`steady_display_report.json` 记录下列载荷数据。原实验数据未覆盖。
- 70–80 s 直边段：控制反馈 **1.7280–1.7366 N**，平均高于 1.5 N 目标 **0.2318 N**；物理边界力 **1.7293–1.7307 N**。箭头相对稳定但未抹平误差，也未宣称精确跟随 F_ref。存在细微速度变化时保守显示瞬态估计提示。该 120 s 轮次中心/运动线段穿透均为 0，最终是 STOP_TIME_LIMIT，不是一圈完成。

未连接真实 UR7e/PX6D，没有执行真机 CLI、commit、push、reset 或删除数据。实际桌面交互、真实动态驱动力和真机物理行为未经验证；自动 GUI 回调、离屏截图与设备替身不等价于现场验证。

## 历史修订：403e9e9 fix2 之后的方向重确认

本轮开始于 `experiment/continuous-tracking`、HEAD `403e9e9`，工作区干净。原三角形 run 为 `simulation_outputs/continuous_preview/run_20260920_235436_498310/`；读取 config_snapshot、samples 和事件后，用同一场景/seed 精确复现了 102.02 s 的停止位置。

停止帧相邻有效测量差 **2.8357°**，相对估计滞后 **46.3442°**；原先 `max(jump,residual)` 将二者合并。原记录中心穿透为 0，最大接触压缩 **0.961335 mm**（模型包络半径 1 mm），没有证据把这次失败归为进入内部后的法向切换。独立复现诊断保存在 `simulation_outputs/direction_revision/validation_20260921_001149_904331/baseline_report.json`，原数据未改动。

当前共享流程：SEARCH → 首次接触停稳确认 → CONTINUOUS_TRACKING；方向变化率/估计滞后增加时平滑减切向 → DIRECTION_RECONFIRM 零运动等待 → 新连续方向窗口、实测停稳均合格 → 确认当帧仍停 → 低速验证前进与载荷 → 正常跟踪。原低力确认、CONTACT_LOST 和默认关闭的弧恢复保留。任何已终止 STOP 不自动恢复。

新参数均在 `continuous_tracking` 下，仿真和真机共用定义，现场仍须验证：

| 参数 | 默认值 | 作用 |
|---|---|---|
| direction_slow_rate_deg_s / direction_slow_residual_deg | 45 °/s / 10° | 提前平滑降低切向，不改变法向力误差公式 |
| direction_reconfirm_residual_deg / direction_reconfirm_rate_deg_s | 30° / 360 °/s | 大滞后或较快测量变化暂停重确认 |
| direction_jump_deg / direction_reversal_deg | 45° / 150° | 前者暂停，后者是近反向不可解释疑点的终止保护；不是把 45° 放宽成 180° |
| direction_confirm_hold_sec / direction_confirm_spread_deg | 0.15 s / 8° | 独立收集新方向，不每帧被旧方向差挡住；不覆盖旧可靠方向直至确认 |
| confirmation_timeout_sec / direction_reconfirm_max_attempts | 原 1 s / 3 次 | 固定总确认期限；无 2 mm 净进展时最多三次尝试，不能每帧重置 |
| direction_min_progress | 2 mm | 重复确认计数的局部进展要求 |
| direction_resume_scale / direction_resume_sec / direction_resume_distance | 0.25 / 2 s / 0.25 mm | 新方向下低速验证、按新切向投影检查进展；反向或不足前进退出 |

力/力矩、速度、workspace、采样新鲜度、watchdog 硬保护未放宽，力符号及手性不自动翻转，不采用新旧切向点积必须为正的规则。120° 新可信方向的隔离回归测试可通过；近反向尖峰终止。停止代码新增 STOP_DIRECTION_UNCONFIRMED / STOP_DIRECTION_REVERSAL / STOP_DIRECTION_NO_PROGRESS / STOP_STALE_DATA，具体文字和首次终止原因保留。

日志记录 measurement_jump_deg、estimate_residual_deg、实际 cycle_dt、测量变化率及平滑率、滤波幅值/coherence、方向阶段、减速比例、重确认次数/时长和恢复运动的进展。仿真另记 sim_signed_distance_m、sim_penetration_m、sim_compression_m、sim_segment_penetration，以及 RAW 模型目标作用/摩擦/背景/噪声 XY 分量；四分量之和与 raw wrench 一致。processed 的零偏与滤波另算。真实合力没有这些可分离分量，明确 unavailable，几何诊断不进入策略。

三条入口复用同一 ContinuousTrackingPolicy。真机入口在 DIRECTION_RECONFIRM 仍收集新鲜反馈、发停止并按原规则维护 watchdog，不把它当实验结束；主循环只记录数值。设备替身回归用一段真正由合成闭环生成的原始力/TCP 流逐条比较预演、无窗口与真机适配的状态和命令，替身在调用入口前完全替换设备类，不连接硬件。

首次转折闭环验证仅把离线测试时长延至 180 s，其余控制/物理参数不因几何种类修改：

| 场景 | 转折后净前进 | processed 跟踪力范围 | 暂停 / 确认 / 验证前进次数 | 中心/线段穿透 | 终止 |
|---|---:|---:|---:|---|---|
| 圆（法向累计 90° 后） | 49.48 mm | 1.302–1.734 N | 0 / 0 / 0 | 0 / 0 | STOP_TIME_LIMIT |
| 方形 | 90.26 mm | 1.303–1.737 N | 1 / 1 / 1 | 0 / 0 | STOP_TIME_LIMIT |
| 原三角形 | 65.83 mm | 1.293–1.737 N | 1 / 1 / 1 | 0 / 0 | STOP_TIME_LIMIT |
| 方形旋转 30°、平移 [10,15] mm | 90.35 mm | 1.303–1.737 N | 1 / 1 / 1 | 0 / 0 | STOP_TIME_LIMIT |
| 目标不在搜索线上 | 不适用 | 无接触 | 0 / 0 / 0 | 0 / 0 | STOP_SEARCH_LIMIT |

这组最大压缩约 0.962 mm，未超过 1 mm 包络；评分器将中心穿透 >1e-9 m、连续运动段进入内部或压缩超过包络判为失败，未吸附 TCP 或裁剪已记录轨迹。达到时间预算不等于过角；通过列的依据是转折后至少 5 mm 的净前进与独立几何审计，**不是整圈完成**。完整报告与各轮日志：[direction_reports.json](simulation_outputs/direction_revision/directions_20260921_002243_013837/direction_reports.json)。中间不足前进的失败调试轮也保留在同一 direction_revision 目录中。

预演左/右约 70/30，默认目标局部取景，支持 Global/Target/Probe、Follow、滚轮/工具栏缩放平移和 Components。共享显示辅助 `simulation/continuous_view.py` 只处理等比例取景与标尺。测得力 8 mm/N，命令和实测速度 12 mm/(mm/s)，N 与 mm/s 分面板；有效方向辅助才是单位箭头。真机回放未知目标按轨迹/TCP 取景，不造目标。GUI 仍默认 10 fps、固定控制 dt、有界点数，不自动生成视频。

本轮问题—修改—测试对应：

| 问题 | 修改 | 实际验证范围 |
|---|---|---|
| 估计滞后被当作测量突跳终止 | 分别记录跳变/残差/变化率，提前减速并暂停重确认 | 修改前新增 8 项复现全部失败；修复后覆盖 120° 稳定新方向、180° 尖峰、停稳不足、方向波动和超时边界 |
| 反复停走或确认后无进展 | 固定总期限、有限重试、新切向低速进展与载荷检查 | 隔离测试两次确认、无进展退出、超载退出、低力/丢边衔接及首次停止原因保留；两次确认测试含注入位姿，不作为几何闭环证据 |
| 只修预演可能与执行入口不一致 | 共用策略、状态和数值日志 | 同一闭环观测流逐条比较预演、无窗口和完全替换硬件的执行入口，状态/命令/诊断一致；不证明设备时延或物理行为 |
| 目标太小、力/速度及分量含义不清 | 局部取景、独立单位和固定标尺、raw 分量及不可用标记 | 自动按钮/坐标回调、同步游标、分量求和、未知真机目标测试；离屏图及 16×9、12×8、20×8 英寸窗口缩放均检查 XY 等比例 |
| 仅停止得安全不能说明已过转折 | 独立几何评分，不向策略提供真值 | 上述五场景从 SEARCH 开始闭环运行；报告转折后距离、载荷、穿透、压缩和实际终止代码 |

最终实现另以默认 **120 s** 预算重跑原三角形并生成[预演图](simulation_outputs/direction_revision/final_preview/run_20260921_004100_164546/continuous_preview.png)及[静态回放](simulation_outputs/direction_revision/final_preview/run_20260921_004100_164546/continuous_summary_target.png)，已离屏查看；同目录保留完整采样、配置和 `offline_resize_checks.json`。只生成静态 PNG，没有编码新视频。

最终全量回归：**730 passed in 171.40s**，包含原离散测试。执行命令（项目目录）：

```bash
MPLCONFIGDIR=/tmp/continuous-mpl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv312/bin/python -m pytest -q
```

确认超时边界、原始力坐标标注收尾后的定向回归先得到 **138 passed, 5 deselected in 12.09s**；其中暂未选择的五个几何用例随后已包含在上述全量通过结果内。

以下各节中的早期验证表/数字是前轮历史证据；当前方向状态、视图及验收以上述修订和分支操作指南为准。本轮没有真实 UR7e/PX6D 连接，没有执行真机命令，也未手动点验桌面 GUI。

前一轮控制修复基准是 `98742befff46bc7c9c9e887e904c505d40f58bb3`，该轮开始时位于 `experiment/continuous-tracking` 且工作区干净。本次预演追加在同分支保留已有未提交修复后进行。本次没有连接 UR7e/PX6D，没有 commit、push、reset、切分支或删除实验数据。以下是软件与合成仿真证据，不是实机可用声明。

仍是：初始 SEARCH → 接触后停稳确认 → 直接连续贴边 → 丢边停止/有限恢复。没有三点初始化、PCA、目标分类、凹凸角分类、机器学习、颗粒补偿或完整 M-B-K 模型。Fxy 始终称为**平面交互力幅值**。

## 问题—修改—测试对应

在修改实现前，先补并运行复现测试：第一批 12 项失败（其中一项验证缺少延迟执行适配），SDK/停止第一批 5 项失败，时序/实机前置条件 2 项失败，回放 2 项失败。原 continuous 测试中跨时间空档的辅助函数改为逐周期提供真实连续样本；未放松原力/力矩/变化率保护。

| 已确认问题 | 修复 | 测试文件/证据 |
|---|---|---|
| 两个读数相隔 1 秒即被当成稳定接触；未检查真实停稳 | 首次越阈立即请求停；分别计力保持、实测 XYZ 低速保持；掉力/速度超限分别重置；采样空档终止；总确认超时 | `test_continuous_revision.py`：空档、速度、掉力/速度脉冲、确认超时、重捕获空档 |
| 恒定幅值的力方向反向、EMA 向量抵消仍可满速推进 | 幅值门槛、平均合向量比、突变检查；不可信则停止；可信估计按实际 dt 限转速，跨 ±π 正确计算 | `test_continuous_revision.py`：反向、抵消、噪声、平滑旋转、手性、速度变化率 |
| 硬死区造成跳变；低力和超载仍恒定切向速度 | 连续死区；第一帧低力即请求停，持续时间只确认 LOST；高力减切向，饱和且不改善限时停止 | `test_continuous_tracking.py` / `test_continuous_revision.py`：连续死区、低力、超载、饱和 |
| 0.9 N 移动后恢复仍以旧强接触点为圆心 | 保存弱而可信接触；独立保存历史记忆、当前位置、丢边位置、实际停稳 origin；origin 每次冻结 | `test_continuous_revision.py`：0.9 N 移动超过 4 mm、过旧记忆、独立日志字段 |
| 三角波仅限制 heading，实际空间覆盖不明确 | 单次弧形位置参考，实际 TCP 闭环；误差大时冻结/停止；同时检查位移/路径/角度/时间/预测点及余量 | `test_continuous_geometry.py`：积分真实执行轨迹、几何接触、空范围失败、旋转/镜像 |
| 停止 False/异常仍标记 stopped；void/bool 混用 | 按本机 1.6.5 合同检查返回值；请求与实测速度分离；不确定时锁定故障；清理仍执行 | `test_continuous_execution.py`：False、异常、void、待停禁止运动、失败也断开 |
| TCP 读数早于阻塞串口；日志阻塞后可继续发速度 | 串口后再读 TCP，记录各读区间；SDK 包时间只比较自身进展；超时禁止下一指令/喂狗 | `test_continuous_execution.py` / `test_continuous_run.py`：旧包、串口/日志阻塞、看门狗失败 |
| 无明确执行边界、sign/frame 核对记录 | 实机前置条件要求有限 XY 范围、绑定配置/标定内容的核对记录、已核实 SDK/看门狗；缺项在设备构造前拒绝 | `test_continuous_execution.py`：缺失记录、绑定变化、全过程位置边界 |
| 停止快照沿用运动前状态 | 停止后读新鲜状态并观察低速保持；失败明确标注 last_valid_sample、年龄和错误，旧 F/T 单独标记 | `test_continuous_run.py`：新停后 pose / 读取失败降级 |
| 回放未画目标、轨迹易误当轮廓、MP4 无总帧数限制 | 仿真真值轮廓、可信/其他运动分色、恢复参考与范围、命令/估计方向、扩大标记说明、统一帧预算 | `test_continuous_run.py`：图层、等比例、游标、帧上限、编码降级 |

## 状态与确认含义

```text
READY → TARGET_SEARCH
          ↓ 首次越接触阈值，立即 STOP 请求
       FIRST_CONTACT（确认阶段）
          ↓ 新鲜力保持 + 实测 XYZ 低速保持 + 方向统计合格
       CONTINUOUS_TRACKING
          ↓ 第一帧低力即暂停；低力保持够长
       CONTACT_LOST（继续请求停，等待实测停稳）
          ├─ 默认 recovery disabled → STOP
          └─ 离线显式启用、记忆新鲜且有进展 → LOCAL_REACQUIRE
                       ↓ 越阈即停，冻结参考，再次相同确认
                  CONTINUOUS_TRACKING

安全 / 时序 / 不可信方向 / 不确定停止 / 预算耗尽 → STOP
```

`FIRST_THRESHOLD_STOP_REQUEST` 保存第一次越阈的位置；`FIRST_CONTACT` 事件和 `initial_contact` 保存实际低速保持确认后的位置，二者不混用。确认阶段不恢复 SEARCH 推进：掉力仅重置力窗口，速度超过阈值仅重置速度窗口；等待超过总确认预算即停。采样间隔过大直接终止，不跨缺帧积累时间。这里的“停稳”是 XYZ 速度连续低于配置门槛，不声称绝对零速。

初始方向取稳定接触窗口内 processed XY 向量均值，其模长除以窗口平均幅值须达到 coherence 门槛。重捕获也用这一门槛，不凭单帧接触恢复。只初始化 continuous 自己的方向状态，绝不在接触中重置传感器零偏或 granular baseline。

## 方向与控制律

现有 wrench EMA 保留；原有 continuous 方向 EMA 保留为唯一的第二级滤波，不叠加其他方向向量滤波器。方向变化率另用同一 alpha 做标量平滑，只用于减速判断。其旧值权重按实际间隔变为 `alpha ** (dt / nominal_dt)`。低力期间冻结局部方向并标记无效；重捕获确认后仅重建局部方向状态，不能消除上游滤波延迟或摩擦影响。

记录三个不同量：processed 测量方向 `measurement_direction/force_direction`、经过 sign 和可信度/限速处理的 `contact_direction`、最终 `command_vx/vy`。未分离纯物体法向力。

```text
f = [Fx, Fy]                     # processed Base
Fxy = norm(f)
n = 估计朝向目标的单位方向         # sign 只能由现场核对确定
CLOCKWISE:        t = [-n_y, n_x]
COUNTERCLOCKWISE: t = [ n_y,-n_x]

e = F_ref - Fxy
e_dead = sign(e) * max(abs(e) - force_deadband, 0)
v_n = clip(Kf * e_dead, -normal_speed_limit, +normal_speed_limit)
v_xy = v_t * t + v_n * n
```

`n/t` 始终单位化、正交。方向突变或近抵消时先零速重确认；近 180° 符号疑点、确认超时等进入终止 STOP，不偷偷翻转新方向以维持行进。可信方向用最短有符号角更新，按实际 dt 限转速。`v_t` 随低载荷、高载荷、可信度/方向限速降低；正常条件恢复时由最终速度变化率限制渐进增加。低于 lost threshold 的第一帧即请求零速，消抖窗口内不会满切向推进并最大向内修正。高于目标但未触发硬保护时优先减轻载荷；最大向外修正持续不改善则停止。最终速度与变化率限制不能延迟硬停止请求。

## 单次局部弧形恢复

恢复默认关闭，`--enable-reacquire` 仅限离线。参考：

```text
p_ref(theta) = O + r * [sin(theta) * n_mem + (cos(theta)-1) * t_mem]
theta: 0 单调增加至 theta_max
omega = min(configured_angular_speed, reacquire_speed / r)
velocity = position_gain * (p_ref - actual_TCP_xy)，再做速度/变化率限制
```

O 来自实测停稳位置；历史可信接触记录包括位置、时间、normal/tangent 和可信度；冻结 n_mem/t_mem 后才开始搜索。参考误差超过冻结门槛不再推进 theta，超过最大误差即停。越接触阈即停止并冻结参考，确认失败后停止，不跳向新的参考位置。单次搜索达到角度/时间/实际累计路径/位移/预测范围预算即停；相邻恢复 origin 无足够空间进展也停止。不自动回旧接触点/P0，不叠加第二搜索策略。

位移和路径预算扣除 `boundary_margin`，规划参考、预测一步、实测位置都检查。这个余量是必须现场验证的工程预算，不是软件能保证的制动距离。真机前置检查至少要求覆盖配置速度下的看门狗时间与理想减速距离。

## 新参数

以下新增实验数值全部 **MUST CONFIRM ON SITE**。原 F_ref=1.5 N、Kf=0.0005 (m/s)/N、切向 0.001 m/s、法向上限 0.0005 m/s、搜索历史值 0.001 m/s（当前初始搜索为 0.018 m/s）、恢复 0.0005 m/s 及原硬安全阈值见当前修订；本段其余为历史记录。

| 参数 | 默认值/单位 | 作用与约束 |
|---|---|---|
| max_sample_gap_sec | 0.03 s | 正时间间隔，至少覆盖名义周期；超过即停 |
| max_observation_age_sec | 0.02 s | 主机读区间年龄、RTDE 包停滞门槛；不代表硬同步采样年龄 |
| cycle_timeout_sec | 0.03 s | 包括设备、指令、日志的循环预算；须短于 watchdog 超时 |
| settle_speed_mps / settle_hold_sec | 0.0001 m/s / 0.08 s | 实测 XYZ 速度连续保持；均正值 |
| confirmation_timeout_sec | 1 s | 确认总预算，必须大于停稳保持 |
| direction_min_force | 0.5 N | 测量方向幅值门槛，≤ lost threshold |
| direction_min_filtered_force | 0.3 N | 滤波向量模长门槛，≤测量门槛 |
| direction_min_coherence | 0.8 | 平均合向量/平均幅值，范围 (0,1] |
| direction_jump_deg | 45° | 相邻有效测量突跳触发暂停重确认；与估计滞后分开记录 |
| direction_rate_deg_s | 90°/s | 可信估计最大转速，正值 |
| command_acceleration | 0.01 m/s² | 正值且不超过原机器人加速度；硬停绕过 |
| overload_tangent_zero_force | 2.25 N | 切向降为零，须高于参考+死区、低于硬保护 |
| overload_stall_sec / overload_improvement_force | 0.5 s / 0.1 N | 饱和向外修正的限时改善要求 |
| memory_max_age_sec | 0.5 s | 开始恢复时记忆允许年龄 |
| reacquire_enabled | false | 默认丢边停止；离线才允许 CLI 开关 |
| reacquire_radius | 0.002 m | 独立弧半径，与目标真值无关 |
| reacquire_max_path | 0.004 m | 实际累计路径及规划弧长预算 |
| reacquire_position_gain | 5 s⁻¹ | TCP 对参考点的比例速度控制 |
| reacquire_reference_freeze_error | 0.0001 m | 超过则冻结参考进度 |
| reacquire_max_tracking_error | 0.0005 m | 超过则停，须大于冻结门槛、小于有效范围 |
| boundary_margin | 0.0005 m | 从位移/路径边界扣除；必须小于两者预算 |
| reacquire_min_progress | 0.002 m | 相邻恢复 origin 最小空间进展 |
| watchdog_frequency_hz | 20 Hz | 1.6.5 机器人侧看门狗；名义超时 50 ms |
| real_test_xy_limits | null | Base 米，有限 x_min/x_max/y_min/y_max；无原 workspace 时必填 |
| site_verification | null | 本次工具/标定/力方向/变换/看门狗的显式现场记录 |

原 `reacquire_max_angle_deg=60` 现在表示 theta 从 0 到 60°，不再是 ±heading 扫描；`reacquire_max_distance=4 mm` 现在相对实际停稳 O。原 `reacquire_max_time_sec=8 s`、角速度上限 30°/s 保留；本配置路径速度上限会将弧实际角速度限制为约 14.32°/s。方向 EMA alpha 仍为 0.8（0≤alpha<1）；接触阈值 1 N、接触力保持 0.05 s、lost threshold 0.5 N / hold 0.15 s 保留。

## 时序、SDK、执行前置条件

本机只读检查 `ur-rtde 1.6.5` 的 Python docstring，并与 [SDU Robotics 接口声明](https://gitlab.com/sdurobotics/ur_rtde/-/blob/master/include/ur_rtde/rtde_control_interface.h) 核对：`speedL(time)` 是函数返回等待时间，**不保证到时停机**；`speedStop` 返回 bool；`stopL` 正常返回 void/None；`setWatchdog/kickWatchdog` 返回 bool，SDK 文档描述超时关闭控制。没有通过真机验证实际停车行为。代码只认可本次已核对的 1.6.5 合同，其他版本/缺接口拒绝 continuous 真机执行。

STOP 请求失败会锁定 `motion_fault`；即使备用停止成功，也不自动解除故障。实测速度才能给出 stopped，连续低速窗口由 policy/退出确认检查。待停状态禁止发新运动。控制对象只由主线程使用，不在后台线程无条件喂狗。

零偏采样和 START 等待均无运动且不启用看门狗。START 后重新核对 P0，然后启用看门狗；只有完成当前健康检查、尚未 STOP 的周期才 kick。串口/RTDE/指令/日志阻塞会使后续 kick 中断；主机解除阻塞后也不再发送新运动。主机不能在同线程阻塞期间立即运行 stop，所以机器人侧看门狗和现场验证必需，不能代替急停与机器人安全设置。

串口完成后读取 TCP，记录主机各读区间；`RobotState.timestamp` 是主机观测时间。RTDE `getTimestamp()` 是机器人启动后的设备时间，只比较其自身单调进展，再用主机时钟测停滞，不与主机绝对时刻相减。首次运动要求已观察到包进展。PX6D 接口没有设备采样时间，日志明确标记主机近似区间；不能证明硬同步或绝对设备数据年龄。

真机默认配置现在会在设备构造前拒绝：缺少有限执行范围和现场核对记录。现有 `workspace.enabled: false` 不被改写；本模式需要单独 `real_test_xy_limits`，不能把箱体图形当运动边界。范围覆盖连接、零偏、START 后、每次状态读取及每个预测指令；还限制本实验实际速度。TCP 身份、固定 Z/姿态、raw/processed F/T、force-rate、保护停/急停继续复用。

现场记录示意（不能直接把示意当作已确认）：

```yaml
site_verification:
  operator: 实际核对人
  checked_at: 实际核对时间
  configuration_sha256: 离线命令输出的摘要
  force_sign_checked: true
  base_transform_checked: true
  watchdog_stop_verified: true
  recovery_verified: false
```

摘要绑定 TCP、preprocessing、sign/continuous 参数、机器人/安全 policy、workspace 和标定文件内容。改变工具、变换、阈值或标定必须重新核对；`--duration` 只允许缩短已审查时限。实机开启恢复还要求独立 recovery 验证记录；本次不提供实机恢复启用证据，不自动启用。

## 离线运行与回放

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour
# 默认离线、默认丢边停止
../.venv312/bin/python run_continuous_tracking.py --dry-run --duration 30
# 显式启用离线实验性弧形恢复
../.venv312/bin/python run_continuous_tracking.py --dry-run --enable-reacquire --duration 30
# 独立闭环评分：直边、圆弧、凸出端点、空范围和延迟压力场景
../.venv312/bin/python -m simulation.continuous_validation
# 全局等比例双视图；默认 10 fps
../.venv312/bin/python tools/visualize_continuous_run.py <run目录>
# 恢复范围附近的局部等比例裁剪，不拉伸 XY
../.venv312/bin/python tools/visualize_continuous_run.py <run目录> --local-xy --format gif
# 只读取本地配置与标定文件并生成核对摘要，不连接设备
../.venv312/bin/python run_continuous_tracking.py --site-digest
# 原有 + 新增全部测试
MPLCONFIGDIR=/tmp/continuous-mpl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv312/bin/python -m pytest -q
```

本次不执行任何实机命令。未来实机入口仍是 `--execute`，但须先完成上述现场条件；Q/ESC/Ctrl+C、异常、退出都先请求停止，再采集停止后状态和写最终日志，不自动回位。

回放严格 XY 等比例；仿真画真实目标轮廓，真机不造目标。可信接触 TCP 片段和 SEARCH/LOST/REACQUIRE 分开，均不是重建物体边界。恢复参考与位移圆单独显示；测得力按 8 mm/N、命令及实测速度按 12 mm/(mm/s) 显示；估计方向仅作 8 mm 单位方向辅助；TCP 点注明扩大标记。局部视窗只裁剪，不改变比例。MP4 最多 1200 帧，GIF 最多 240 帧；达到预算降低 fps 保持时长，无中间 PNG 序列，编码失败保留静态总结。本环境缺 ffmpeg，实际验证的是 GIF 和 PNG。

## 日志与证据分层

复用 `ExperimentLogger` 的原 CSV/事件/termination 文件。新增方向有效性/可信度/限速、观测 dt、loop-start 间隔、serial/TCP 主机读区间、RTDE 设备时间与停滞时间、命令调用时间、停止请求/确认、力/速度保持窗口、历史可信接触位置/时间/方向、loss detection、冻结 O/n_mem/t_mem、恢复参考/误差/累计路径、限制原因。

配置快照记录 branch/commit/dirty/status 与有效参数。停止快照包含停止后状态来源、实际确认状态；失败标明最后有效样本及年龄。快照中 F/T 仍是最后一次有效读取，明确标注并保留年龄，不冒充停稳时的新测量。

证据区分：

1. **隔离单元/设备替身**：手动 F/T 输入验证状态、安全与时序。包含按时间输入重新接触的测试，但只证明状态转移，不证明几何重捕获。
2. **从 SEARCH 开始的闭环合成仿真**：`SimulatedRobot` 积分实际运动，现有 `SimulatedForceSensor` 由独立几何产生力；加入两周期（20 ms）延迟与 0.005 m/s² 加减速度。无 TCP 裁剪/吸附，policy 不读几何。
3. **闭环恢复子场景**：显式种入已丢边状态和历史局部方向，之后全部力由实测积分位置产生；用于检验弧策略，不能等同完整 SEARCH→自然丢边→恢复证据。
4. **真机**：本次零连接、零运动；增益、外参、sign、停稳门槛、实际数据年龄、看门狗/停止时延、边界余量、颗粒污染和恢复效果均无真机验证。

八个默认评分场景（合成模型有 1 mm 弹性 tip sensing envelope；实际 TCP 穿入和 envelope 压缩分别报告；最大值含停止后的制动尾段）：

| 场景 | 结果 | 最大力 N | 实际 TCP 最大穿入 | 最大 envelope 压缩 | 恢复参考最大误差 |
|---|---|---:|---:|---:|---:|
| SEARCH→直边，20 ms 延迟 | 维持接触，接触保持率 100% | 1.350 | 0 mm | 0.750 mm | 不适用 |
| SEARCH→圆弧，20 ms 延迟 | 维持接触，接触保持率 100% | 1.340 | 0 mm | 0.744 mm | 不适用 |
| SEARCH→凸出端点，20 ms 延迟 | 维持接触，没有自然触发 LOST | 1.350 | 0 mm | 0.750 mm | 不适用 |
| 直边恢复子场景 | 稳定接触+停稳后 REACQUIRED | 1.083 | 0 mm | 0.601 mm | 0.095 mm |
| 圆弧恢复子场景 | 稳定接触+停稳后 REACQUIRED | 1.085 | 0 mm | 0.603 mm | 0.095 mm |
| 端点丢边恢复子场景 | 稳定接触+停稳后 REACQUIRED | 1.066 | 0 mm | 0.592 mm | 0.095 mm |
| 搜索范围内无目标 | theta 达上限，明确失败 STOP | 0 | 0 mm | 0 mm | 0.095 mm |
| SEARCH→凸出端点，300 ms 执行延迟压力 | 方向不可信，失败 STOP，无备用策略 | 1.800 | 约 7e-15 mm 数值量级 | 1.000 mm | 不适用 |

前三个跟踪场景力误差 RMS 分别约 0.158、0.165、0.202 N。三个恢复子场景实际路径约 1.167、1.172、0.677 mm；空范围失败实际路径（含制动尾段）约 2.030 mm。真值只用于力模型和评分器；参数未从真值计算，也未提高阈值、速度、增益或参考力来通过测试。300 ms 是故意超大的执行滞后压力条件，不是声称真机正常具有该延迟。

## 文件变化与复用边界

- `policy/continuous_tracking.py`：仅连续 policy；仍复用 `core.models`、`handed_tangent`、`force_guard`。
- `run_continuous_tracking.py`：时序、实机前置检查、单线程看门狗、停止后快照与溯源。
- `robot/rtde_controller.py`：在同一驱动里修复停止合同、故障锁定；增加 continuous 可选时间戳/边界/看门狗接口，不复制驱动。
- `config.yaml`：新增参数和默认关闭恢复；原安全阈值未提高。
- `experiment_logging/data_logger.py`：停止快照增加可选元数据，旧调用默认不变。
- `simulation/simulated_robot.py`：可选延迟/加减速适配，默认保留原理想积分行为。
- `simulation/continuous_validation.py`：小型独立评分与场景串接，复用原几何和力模型。
- `tools/visualize_continuous_run.py`：只读离线回放增强。
- `tests/test_continuous_tracking.py` / `test_continuous_run.py`：原断言更新为连续新鲜样本并补入口测试；新增 `test_continuous_revision.py`、`test_continuous_execution.py`、`test_continuous_geometry.py`。

`rule_policy.py`、原离散入口、PX6D 驱动、标定文件未修改。完成本轮要求后不扩展其他控制策略。

本轮闭环原始结果与有效配置见 [reports.json](simulation_outputs/continuous_revision/validation_20260920_212838_032429/reports.json)。每个 run 独立保留 CSV、配置快照、几何评分和局部等比例静态图；端点恢复另有 GIF。报告中的局部恢复是显式历史记忆假设下的闭环子测试；默认从 SEARCH 开始的端点场景并未自然丢边，不能据此声称完整丢边恢复链已经经过现场验证。

前一轮控制修复完成时全量测试：**668 passed in 119.12s**。其中 543 项为原有其他模块测试，125 项为 continuous 相关测试；较审查基准新增 56 项，既有 continuous 辅助函数按新鲜连续采样要求修正。`git diff --check` 通过。最终 HEAD 仍为 `98742be`，全部修改留在当前分支工作区，未自动提交。

端点恢复子场景产物：[局部等比例静态图](simulation_outputs/continuous_revision/validation_20260920_212838_032429/run_20260920_212840_132093/continuous_summary_local.png)、[同步 GIF](simulation_outputs/continuous_revision/validation_20260920_212838_032429/run_20260920_212840_132093/continuous_replay_local.gif)。这些只展示合成几何恢复子测试，不展示真机行为。


## 追加：离线交互预演

本次追加保留前轮控制修复及所有阈值。`run_continuous_tracking.py --preview` 只在显式请求时加载 Matplotlib，与 `--execute` / `--dry-run` 互斥；默认入口仍为无窗口离线仿真。

- `simulation/continuous_session.py` 抽取原离线装配、零偏捕获及单步：SimulatedRobot → SimulatedForceSensor → WrenchPreprocessor → ContinuousTrackingPolicy → SimulatedRobot。无窗口与预演调用同一个 step；目标几何不传入策略。模拟延迟 2 步、加速度 0.005 m/s² 保持不变。
- `simulation/continuous_preview.py` 提供待开始场景、三种形状、拖动、15° 旋转、暂停/继续、1/5/10 倍播放、停止/重置、独立 YAML 保存/加载及显式 PNG。固定控制 dt，默认 10 fps，回调最多 100 步并设计算时间预算；性能不足只降低实际播放速度。
- 场景校验整目标在容器内、起点在接触包络外、P0/P1 与尺寸有效。有效编辑先结束旧轮次并记录 SCENE_CHANGED，重新装配零状态；暂停继续不重置。独立场景只允许创建新 YAML，不覆盖现存配置/标定/数据。
- 双视图只使用已观测数据；力显示最近 30 秒，整轮轨迹最多 4096 点并保守合并跨状态段，XY 等比例、力极值保留，明确区分搜索/可信接触/不确定/恢复运动。力与速度使用分开的固定幅值标尺，仅方向辅助箭头是单位方向。CSV 保存所有步，预演停止后的制动尾段也采样记录；原无窗口入口仍保留原有独立最终停止快照。
- `tests/test_continuous_preview.py` 覆盖真实无窗口日志与预演日志的逐步一致性、显示 fps/倍速独立、暂停确认时钟、编辑记忆清空、布局拒绝、保存保护、实际按钮/拖动回调、极值降采样、GUI 懒加载、无桌面回放和硬件构造前互斥拒绝。

操作、输出路径及核验范围见上方分支指南。关闭或停止窗口只表示本轮结束，不表示通过轮廓验收；本次不提供真机物理验证或运行许可。

第一轮预演追加完成时全量回归：**692 passed in 128.03s**，其中本次新增预演测试 24 项；原有 668 项（含离散回归与前轮控制修复测试）保持通过。自动测试调用按钮、拖动与计时回调，未阻塞于 `plt.show()`；这不等于手动点验真实桌面窗口。

| 问题 / 核验层级 | 修改与实际证据 |
|---|---|
| 预演是否换了控制器 / 闭环仿真 | 相同场景/seed、12 秒预算的真实无窗口 CSV 与预演 CSV 逐条比对状态、TCP、原始/处理力和命令；10/20/5 fps 与 1/5/10 倍组合一致 |
| 编辑沿用记忆、暂停计入墙钟 / 回调及集成测试 | 有效编辑记录 SCENE_CHANGED、保留旧目录并重建会话；暂停保留记忆和仿真时钟；首次接触确认中暂停不满足保持时间 |
| 场景、设备、显示保护 / 隔离与接口测试 | 三种形状、旋转、P0/P1 独立、非法布局拒绝、YAML 不覆盖、真实设备构造前互斥拒绝、无 GUI 下 headless/回放、XY 等比例/同时间/力极值和有界路径测试通过 |
| 长运行数据量 / 完整离线闭环 | 默认 120 秒圆形预演记录 12,018 条样本（含制动尾段），显示轨迹 3,826 点、力窗口 3,001 条；按 STOP_TIME_LIMIT 停止，未宣称完成轮廓 |
| 真机物理行为 / 未验证 | 没有真实 UR7e/PX6D 连接、运动或桌面手动点验；合成接触包络、力方向约定、刚度、噪声、阻力及执行延迟不构成现场物理验证 |

本次完整预演产物：[同步静态 PNG](simulation_outputs/continuous_preview_verification/run_20260920_215819_746145/continuous_preview.png)、[停止摘要](simulation_outputs/continuous_preview_verification/run_20260920_215819_746145/summary.json)、[独立场景](simulation_outputs/continuous_preview_verification/run_20260920_215819_746145/independent_scene.yaml)。12 秒 CLI 和显式静态回放产物位于 `simulation_outputs/continuous_preview_cli_verification/run_20260920_215552_796097/`。未自动编码长视频，未自动提交或推送，未删除旧数据。
