# UR7e + PaXini PX6D 规则式力反馈轮廓探索

这是一个独立的 Linux/Python 工程。它不依赖 ROS 2、MoveIt 2 或
`ur_robot_driver`，通过 `ur-rtde` 直接读取和控制 UR7e，并通过 USB 串口直接读取
PaXini PX6D。

当前版本实现固定姿态、固定 Z 下的二维 XY 探索：

```text
PX6D raw F/T
  -> bias -> EMA low-pass -> fixed-pose gravity correction
  -> wrench frame transform -> no-target granular baseline
  -> deterministic state machine
  -> guarded planar speedL command
```

默认配置是 `mock + dry-run`，不会连接或移动机器人。真实运动入口不再要求布尔确认门锁或风险
口令。当前软件 workspace 限位已关闭；TCP、标定和安全返回数值仍会校验。

如果不熟悉项目结构，请不要从下面的模块说明开始读。直接打开
[从这里开始.md](从这里开始.md)，然后运行：

```bash
python3 run_project.py
```

## 工程目录

```text
ur7e_px6d_contour/
├── app/                        # 主循环与操作者输入
├── core/                       # 模块间共享数据结构
├── sensor/                     # PX6D、预处理、force features
├── policy/                     # 沿边、边界恢复、确认和几何
├── robot/                      # RTDE 状态、运动和 stop
├── calibration/               # 双点扫描标定与 TCP 绑定复位点
├── safety/                     # force guard 与 safe return
├── experiment_logging/         # CSV/JSON 写入，不参与运动决策
├── config/                     # YAML loader
├── simulation/                 # 共用同一 policy 的二维闭环模拟
├── tests/                      # 单元测试和三场景闭环测试
├── tools/                      # 只读检查与离线可视化
├── old/                        # 旧兼容工程/策略的两个 tar.gz
├── main.py                     # 主程序入口，转发到 app.main
├── run_project.py              # 中文安全菜单
├── run_simulation.py           # 模拟入口
├── run_calibration.py          # 只读标定入口
├── save_reset_pose.py          # 任意静止位置保存为 TCP 绑定复位点
├── return_to_reset.py          # 从任意当前位置返回复位点
├── return_to_start.py          # 兼容的人工返回扫描 P0 入口
├── config.yaml
└── scan_calibration.yaml
```

完整到文件级的说明、数据流和“应该去哪个文件修改”见 [ARCHITECTURE.md](ARCHITECTURE.md)。
当前算法教学见 [POLICY_EXPLANATION.md](POLICY_EXPLANATION.md)：切线来自多个 TCP 接触点，
不是单帧力方向；所有 probe 都先返回自己的 anchor，再做决策。新策略尚未完成真机验证。

运行后数据写入 `data/run_YYYYmmdd_HHMMSS/`：

- `samples.csv`：每个控制周期的 raw、processed、derived、TCP 和 policy 全字段；
- `full_log.csv`：正式扫描与安全返回的完整周期日志；
- `boundary_points.csv`：单独保存轮廓采样点（与标定 P0/P1 是两类数据）；
- `probe_episodes.csv/json`：每次探测的 anchor、方向、接触、返回、接受/拒绝与时间；
- `probe_forces.csv`：按 probe_id 关联的 PROBE/HOLD/RETURN 力时间序列；
- `scan_stop_snapshot.json`：停止瞬间的 TCP、速度和 raw/processed F/T；
- `return_status.json`：返回是否完成、终止原因、P_stop、P0 和最终 TCP；
- `config_snapshot.yaml`：本次配置快照；
- `summary.json`：结束状态、原因、边界点数和返回状态；
- `contour_result.png`：扫描结束后的轮廓结果图；
- `policy_waypoints.csv`：策略离散关键点（contact、retract、tangent、probe、anchor、确认）；
- `boundary_recovery_rays.csv`：每条扇形射线的固定 anchor、角度、方向、实际终点和结果；
- `scan_strategy_debug.png`：完整 TCP 轨迹、策略点、轮廓点和全部扇形射线；
- `boundary_recovery_<id>.png`：每个拐角的 Pc、P_clear、P_anchor、射线与确认结果；
- `visualization.png`：旧真实运行离线分析工具的多面板图；菜单 2 已改为正方形单图。

## 模块说明

`sensor/px6d_reader.py` 使用实验室已有实测帧交叉验证过的协议：帧头 `AA 55`、设备 ID
`0x7F`、CRC-8 多项式 `0x07`、初值 `0x00`，六个量按 little-endian `float32`
解析。超时、串口异常、CRC 或帧格式异常会抛错，主循环随即停止机器人。

`sensor/force_preprocess.py` 始终保留 raw 数据，另生成 processed wrench。处理顺序明确且可测：

1. 软件 bias correction；
2. 六维 EMA low-pass；
3. 固定工具姿态下的常量 gravity wrench subtraction；
4. sensor 到 base/control frame 的完整 wrench 变换（包括力矩臂 `r × F`）；
5. 固定插入深度下的 no-target granular baseline subtraction。

如果启动 bias 样本已经包含安装后工具重力，应将 `gravity_wrench_sensor` 保持为零，避免重复
扣除。若 bias 是安装前的电子零偏，才应独立标定重力项。

`sensor/force_features.py` 计算 `Fxy`、`atan2(dFy,dFx)` 和二维单位方向。代码只把 residual
XY force direction 称为 **estimated interaction/contact direction**；它不是目标真实表面法向，
也不是 ground truth。`estimate_effective_contact_location()` 和
`check_tip_contact_consistency()` 已预留，但 v1 不使用力矩决定运动。

`policy/rule_policy.py` 同时供真机主程序和二维模拟器使用，模拟器没有复制扫描策略。它实现：

```text
TARGET_SEARCH -> LOCAL_INITIALIZATION -> BOUNDARY_TRACKING
BOUNDARY_TRACKING: PROBE -> RETURN TO SAME ANCHOR -> UPDATE LOCAL FIT -> TANGENT STEP
No contact after return -> BOUNDARY_RECOVERY -> BOUNDARY_CONFIRMATION
Confirmation: three consistent contacts -> BOUNDARY REACQUIRED -> BOUNDARY_TRACKING
Rejected confirmation -> retrace anchor path -> next local recovery ray
Q normal stop -> STOP_SCAN -> RETURN_TO_START -> STOP
ESC / emergency / safety / communication failure -> STOP
```

正式 execute 不设置边界点数或总运行时自动完成条件；保持接触时上述轮廓状态循环运行，直到
用户按 Q。TARGET_SEARCH 仍保留 1.5 m 未接触距离上限，搜索失败、初始化失败、恢复耗尽与安全/通信异常会停止。

SEARCH 的初始直线方向来自 `scan_calibration.yaml` 中由示教点 P0→P1 计算的
`scan_direction_xy`。第一次稳定接触后，仅沿已走搜索射线局部回撤到初始化 anchor（当前 9 mm，
不超过实际已走距离），停稳后直接做三个局部平行探测；不再回远处 P0 后重新靠近目标。
通过局部 PCA 估计切线；力只用于接触与安全，不决定切线。配置文件
`policy.search_direction_xy` 只供未连接真机的 mock/dry-run 使用，正式 `--execute` 会要求完整双点
标定并用标定结果覆盖它。

首次过接触阈值立即停止推进，原地等待 `contact_hold_time`。成功 acquisition 返回上述局部目标；
其他探测仍沿自己的射线返回原 anchor。原始搜索起点与实际返回目标分别保存在
`anchor_pose` 和 `return_target_pose`，不通过修改原始起点伪造返回完成。
稳定接触才参与拟合；瞬态或无接触不添加边界点。probe 方向随局部拟合切线更新，正负方向由
历史成功探测侧和固定 follow_hand 决定。

转移到位后输出停止命令，只有位置、实测线速度和力连续满足原返回就绪条件才推进下一段。
若反馈跨过目标，先停止；漂出位置容差时先停稳，再以不高于局部 probe 速度纠偏，超过 2 s
仍未稳定则记录 `STOP_ANCHOR_ERROR`。转移阶段超过 1 N 接触阈值仍停止。
完整 probe 历史在扫描及安全返回结束后导出；控制循环保留逐采样日志，避免重复全量写盘。

当前真机名义速度：搜索 18、局部/恢复探测 6、切向/回撤 12 mm/s；
正常安全返回水平 30、竖直 18 mm/s。拐角 RECOVERY/CONFIRMATION 探测在力达到接触阈值一半
（当前 0.5 N）后，将该次探测限速到 2 mm/s，力回落也不重新加速；下一条射线重新判断。
接触阈值、力保护、加减速度保持原值。
实现与离线验证说明见 [SCAN_EXECUTION_UPDATE.md](SCAN_EXECUTION_UPDATE.md)。

恢复起点就是已返回的 probe anchor，不用物体真实几何计算“外侧安全点”。从预测探测方向附近
开始，逐级扩张可配置局部角域，每条射线都是短距离增量、触力停止、原路返回。候选必须经
附近 anchor 的额外探测，默认总计三个有间距、拟合稳定且向前的点，才确认 BOUNDARY REACQUIRED。
两个有效接触点建立局部方向后，确认探测使用拟合法向，并按最新接触点补偿下一 anchor 的
切向进度；回退射线的中间 clearance 点也纳入原路回滚。不稳定接触返回停稳后最多沿同一
anchor/方向重试两次，保留已有有效点；真正未接触或重试耗尽才放弃候选。
确认失败沿已走过的 anchor 折线路径回到恢复起点，绝不从接触点斜穿物体。所有阶段受 F/T 和
正向力变化率安全保护。

默认正方形模拟采用显式 1 mm 柔性感力包络，TCP 位于边界外就可触力；位置按命令积分，不裁剪
穿透。独立检查器检查完整运动线段、触力后前推、每次 anchor 返回、接触位置及四次恢复/闭环。
任何 TCP 穿透显示 PHYSICAL FAILURE，不能用状态完成覆盖。实际探头/颗粒物理尚未由该模型验证。

扫描策略只能使用平面 `speedL`：Z 速度及三个角速度永远为零。控制器每周期读取实际 TCP
pose/speed，检查工作区、锁定 Z、锁定姿态、命令速度和预测下一位置。独立安全返回器可以使用
受监控的异步 `moveL` 完成三段路径；每段到达容差后先停稳再进入下一段。扫描和返回均可立即
停止。这些软件检查不能替代 UR 自身的安全配置、保护停止、急停、风险评估或现场监护。

正式扫描的 Q 正常停止由主程序编排为 `STOP_SCAN → RETURN_TO_START → STOP`。返回路径固定为
沿 Base +Z 相对抬升、在安全高度平移、垂直下降三段，且全程继续监控 PX6D、RTDE 和机器人
安全状态。ESC、Ctrl+C、通信故障、保护停和力安全阈值属于紧急停止，绝不自动返回。详见
[SAFE_RETURN.md](SAFE_RETURN.md)。

正式 execute 命令启动时如果 TCP 不在 P0，也会先复用上述三段路径自动返回 P0；返回完成后
才采集 baseline 并等待 `START`。启动返回失败不会进入扫描。

## 快速开始（只做离线测试）

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour
source /home/user-linux/robotics_cs/.venv312/bin/activate
unset PYTHONPATH
~/.local/bin/uv pip install -r requirements.txt
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
python main.py
```

最后一条仍是 mock dry-run：不打开 UR 网络连接，也不发送任何运动命令。完整分阶段流程见
[EXECUTION.md](EXECUTION.md)。

## 正方形整圈验证

默认离线入口现在验证一个中心位于原点的 100 mm × 100 mm 正方形。菜单 2（无窗口）、菜单 6（实时动画）和 `python3 main.py --sensor mock` 使用同一个几何闭环。
力模型为确定性弹簧接触：自由空间为零，接触力沿目标面的局部法向关系生成；noise、drag、friction 均为零。

所有动作来自 `policy/rule_policy.py` 中的真实共享策略；目标几何只提供接触力和事后评估，不传给 policy。
闭环成功要求：四条边都有接触、四次边界恢复确认、足够边界路径、至少 0.95 圈同向环绕、回到首次接触附近、切向一致。
`simulation/loop_completion.py` 只在 simulation 中评分和停止，不用于真机默认停止逻辑。

```bash
python3 run_simulation.py             # 单张 XY 图实时动画
python3 run_simulation.py --no-gui    # 自动运行并保存结果
xdg-open simulation_outputs/full_loop_square.png
```

实时动画可点击图内标签或按 `1/2/3` 切换 square/circle/triangle。左键拖动只移目标；
Shift+左键拖动同时平移目标和 P0/P1。交互后自动重启实验，且 policy 不能读取形状或中心。
命令行也支持 `--shape square|circle|triangle` 和单位为米的 `--center X Y`。

SPACE 暂停，Q 停止，R 重启，ESC 关闭。成功显示 FULL LOOP COMPLETED；失败显示 LOOP FAILED 和原因。
每次运行另存时间戳目录，默认不生成多面板曲线或单独 corner 图。完整操作见 [操作文档.md](操作文档.md)。

## 标定扫描起点和初始方向

`run_calibration.py` 只建立 RTDE Receive 连接，不发送运动命令。完整流程是：

1. 用示教器将 TCP 放到扫描起点 P0，输入 `SAVE_P0`；
2. 沿期望的初始扫描方向在 XY 平面移动到 P1，输入 `SAVE_DIRECTION`；
3. 程序检查 XY 距离和 Z 差，并计算 P0→P1 的 XY 单位向量；
4. 运行 `python3 tools/check_scan_calibration.py`，生成
   `scan_calibration_preview.png` 复核箭头；
5. 正式 SEARCH 从 P0 沿该向量直线扫描。

P1 只是方向参考点，不是轮廓采样点，也不是正式扫描必须经过的第二个点。新 P0 保存后，旧 P1
会立即失效；如果第二阶段取消或校验失败，正式扫描门锁会拒绝 execute 模式。

模拟详情见上方正方形整圈验证；真实运行日志仍保存在 `data/`。

## 必须现场确认的参数

以下项目在 `config.yaml` 中即使已有示例数字，也不能视为真机参数：

| 类别 | 必须确认/测量 |
|---|---|
| 安装与坐标 | PX6D 轴向和符号、`rotation_sensor_to_base`、传感器原点力矩臂、探针尖端 active TCP |
| 方向规则 | P0→P1、固定 follow_hand、初始化偏移与局部 PCA；已知轻触验证坐标旋转，旧 probe_direction_sign 不再用于运动 |
| 基线 | 软件 bias 的采集物理状态、固定姿态重力项、相同深度且无目标的颗粒基线 |
| 阈值 | 空载/颗粒噪声分布、`contact_threshold`、hold time、processed/raw F/T safety threshold |
| 运动 | 四种低速、加减速度、retract、tangent step、probe/search 最大距离、位置容差 |
| 几何 | 当前 `workspace.enabled: false`；仍需确认固定 Z、姿态容差和整段工具/线缆扫掠空间 |
| 安全返回 | P0、向 Base +Z 抬升 0.030 m、三段扫掠空间、返回速度/加速度、F/T 阈值和最终误差容差 |
| 实验结束 | 正式轮廓扫描由 Q 正常结束；recovery 角域、确认点数和 anchor 返回容差 |
| UR 控制器 | IP、Remote Control 模式、active TCP、payload/CoG、安全平面、工具安装和急停可用性 |

execute 模式没有 `confirmed`/`allow_robot_motion` 布尔门锁；当前 workspace 软件限位关闭，
TCP、标定和安全返回的数值完整性以及运行期间的 F/T、通信、姿态和机器人安全状态检查仍有效。

## 当前边界

- 没有碰撞模型、IK 规划、自动避障或完整轮廓闭合判断；
- `speedL` 的零 Z/角速度不是笛卡尔位置伺服，软件只用 drift watchdog 监控固定 Z/姿态；
- constant gravity correction 只适用于姿态基本不变的本实验；
- 不对 force direction 的物理含义、符号或 target normal 作未经实验的保证；
- torque 完整记录，但尚无经过验证的 tip-contact 模型；
- follow_hand 固定；正方形 simulation 有独立的整圈评分和自动停止，真机仍默认人工 Q 停止。
