# 连续贴边跟踪：审查修复版

新用户先看：[【分支 experiment/continuous-tracking】连续贴边：傻瓜式操作指南](BRANCH_continuous-tracking_傻瓜式操作指南.md)。


## 当前修订：403e9e9 fix2 之后的方向重确认

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

以下新增实验数值全部 **MUST CONFIRM ON SITE**。原 F_ref=1.5 N、Kf=0.0005 (m/s)/N、切向 0.001 m/s、法向上限 0.0005 m/s、搜索 0.001 m/s、恢复 0.0005 m/s 及硬安全阈值均未提高。

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
