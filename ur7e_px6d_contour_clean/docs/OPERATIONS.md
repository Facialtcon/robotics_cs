# UR7e + PX6D 操作手册

## 日常启动与停止

在前台交互终端打开原菜单：

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean
/home/user-linux/robotics_cs/.venv312/bin/python run_project.py
```

1. 选择 **18** 启动连续扫描，按 Enter 打开任务。
2. 核对当前 TCP、P0、搜索方向、探针倾角和完整返回路径。确认探针脱离目标及颗粒、基座水平、抬升后有足够旋转空间，再按 Enter：先抬升、将探针轴调到 Base −Z、移动到 P0 上方、下降到 P0。已在 P0 且姿态正确时跳过返回。
3. 核对调正后的倾角。在最终姿态的 P0，确认探针脱离目标、静止空载，按 Enter 采集扫描零偏；采零完成后，再按 Enter 开始扫描。SEARCH 保持 **18 mm/s**。调正前不采零。
4. 运行中按 **Q / Esc / Ctrl+C** 原地停止，不自动返回。等待停稳确认和日志保存、回到菜单，再执行下一项；菜单 **0** 退出。

每个确认提示出现后单独按一次 Enter。程序清除积压按键，不接受预先粘贴或管道传入的回车；确认时按 Q/Esc/Ctrl+C 可取消。菜单选择和数值输入仍按原方式填写。探针接触目标时不能采零。

原菜单 **11** 用于手动返回 reset（没有 reset 文件则返回 P0）：按 Enter 打开任务，核对路径后再按 Enter 执行。手动返回不连接 PX6D；自动启动返回有 PX6D 力监控。返回中 Q/Esc/Ctrl+C 中止。

原菜单 **12** 是离散扫描，启动同样逐步按 Enter；Q 按原配置正常停止并可能返回，Esc/Ctrl+C 不自动返回。急迫危险使用现场物理急停。

## 故障排查

### 垂直姿态与接触方向

保存扫描姿态原先使 TCP +Z 偏离 Base −Z **5.0466°**。当前以操作者确认的探针轴
`calibration.probe_axis_tcp: [0, 0, 1]` 计算最小旋转，尽量保留绕轴朝向；不修改 TCP 偏置，
不覆盖原始示教文件。派生的返回目标、扫描起点与固定姿态使用同一姿态。
`startup_alignment.json` 保存实际调正前后倾角和目标。Base −Z 为重力向下的前提是基座水平。
旋转仅在原安全返回流程抬升并确认静止后进行；返回阶段使用原始力模长监控，不使用旧姿态零偏。
每段返回沿用原 60 秒预算，超时停止，不继续下一段。

最新 `run_20261002_204147_019798` 是低力丢接触、恢复角度用尽：1.078 N 在约 0.606 秒内
降至 0.498 N，实际只移动 0.226 mm，不代表沿边成功。旧配置把主要 +Y 的力方向当作补压方向，
而导致接触的搜索主要沿 −Y。此类方向矛盾须明确停止，不能自动翻符号或扩大恢复角度。
实际 `speedL` 指令、调用序号及返回值写入逐帧日志；序号未变表示该帧没有新的 `speedL` 调用。
SDK 接受指令不代表已完成运动，仍以实际 TCP 和接触力判断。

姿态变化后，力变换按实际 TCP 姿态更新：提供实测 `rotation_sensor_to_tool` 时使用
`R_BS = R_BT * R_TS`；已有实测 `rotation_sensor_to_base` 时必须同时给出测量时的
`reference_tool_orientation`（TCP 旋转向量，rad）。这两个输入都没有时，只显示
`sensor_uncalibrated`，不冒称 Base 力；程序在连接设备、启动返回及 SEARCH 之前报告
`STOP_CONFIG_ERROR / CONFIG_PREFLIGHT`，不会等到接触后才发现缺少安装方向。
当前两项均为 null，旧单位矩阵是占位值。缺少的是 PX6D +X/+Y/+Z 在当前 TCP 中的方向，
即 `rotation_sensor_to_tool` 的三列；“探针轴等于 TCP +Z”不能替代这项安装信息。
也不能把保存的 TCP 姿态与旧占位单位阵组合成所谓实测标定。
不生成强制验证文件，也不自动推导安装旋转或翻转 `force_direction_sign`。
`force_transform_at_start.json` 记录安装矩阵、参考姿态、最终实际 TCP 姿态和合成的 Base 矩阵。
直接安装关系使用 `R_BS = R_BT_actual * R_TS`；参考姿态关系使用
`R_BS = R_BT_actual * transpose(R_BT_reference) * R_BS_reference`。
`force_direction_sign` 仍只在控制方向计算时生效，不加入这两个旋转矩阵。

### 力处理顺序与 Kalman 参数

扫描链路统一为 PX6D raw Sensor wrench → Sensor 零偏/已有重力补偿 →
Sensor→Base 旋转/已有 Base 颗粒基线补偿 → Base 三轴独立 Kalman →
原接触阈值、死区、n/t 方向估计及力反馈 → policy → RTDE speedL。
`URRTDEController.read_state()` 从 `getActualTCPPose()` 读取
`[x,y,z,rx,ry,rz]`；后三项是 rad 单位的旋转向量。
`sensor/force_preprocess.py::_tool_rotation()` 用 Rodrigues 公式构造 `R_BT`，
扫描每帧在串口读取后更新实际姿态，`processed.force` 就是 `filtered_force_base`。
离散与连续入口均拒绝将安装关系未知的 Sensor 力传给扫描 policy；未知数据仍可用于只读诊断。

`preprocessing.kalman` 的默认值为 `enabled: true`、`process_noise: 0.01`、
`measurement_noise: 0.25`、`initial_covariance: 1.0`。
Q/R/P 为 N²，Q 按每个有效采样步计；可用标量或按 Fx/Fy/Fz 的三个数配置。
这些是保守起始值，尚未用 PX6D 静态实验数据辨识，采样频率改变后需重新评估 Q。
滤波状态在帧间保存，采零/重新设基线后重置，第一帧由真实测量初始化。
旧配置不含 kalman 时使用上述默认值；`filter_alpha` 仅保留兼容，已不参与 wrench 滤波。
`enabled: false` 直接透传补偿后的 Base 力，不恢复 EMA。
缺失、NaN、Inf 不更新滤波状态；串口超时沿用停止路径，不补零也不将旧输出当新测量。
连续 policy 既有的方向/一致性和方向变化率平滑仍保留，输入来自 Base Kalman，
它们属于原跟踪策略，不是 Sensor-frame wrench 滤波。
卸力配置校验中的滤波稳定时间改为 Kalman 增益的保守估计，卸力预算与策略不变。

当前 `sensor_origin_in_tool_m: null` 表示几何偏置未知，力矩只在传感器原点处旋转到 Base，
不是 TCP 原点的力矩。旧 `sensor_origin_in_base_m: [0,0,0]` 不证明原点重合。
如需完整参考点变换，实测从 TCP 原点指向 Sensor 原点的向量（TCP 表达，m）后填写
`sensor_origin_in_tool_m`；届时使用 `M_B@TCP = R_BS M_S + (R_BT r_TS) × (R_BS F_S)`。
已有非零 Base 偏置配合实测参考姿态的配置也继续支持。
`force_transform_at_start.json` 中 `torque_reference_point` 和
`wrench_reference_point_transform_complete` 明确记录当前范围。
三维力旋转不依赖这项平移参数；不新增 torque 控制或动态重力模型。

日志保留 `raw_fx/fy/fz/tx/ty/tz`（raw Sensor wrench）和 `dfx/dfy/dfz`（policy 输入）。
新增 `force_base_fx/fy/fz` 保存补偿后滤波前的 Base 力，
`filtered_force_base_fx/fy/fz` 是 Base 输出的显式别名。
图表继续读取历史 dfx/dfy/dfz 并按 `processed_force_frame` 标识；
启动返回使用原始 Sensor 力模长时，Base 别名留空，不混用参考系。

### 日志与现有菜单

停止后先查看终端打印的本次运行目录。数据在 `data/real/continuous/`、`data/real/discrete/`、`data/real/return/` 或 `data/simulation/`，目录时间为 UTC。

| 项目 | 查看或操作 |
|---|---|
| 停止原因 | `termination.json` / `termination.txt` 的 reason、detail、phase；`summary.json` 的停止确认结果 |
| 为什么不动 | `samples.csv` 的 `tangent_limit_reason`、处理后力、n/t、指令和实际 TCP 速度/位移；`software_warnings` 可能包含历史告警，不等于当前停止原因 |
| 接触刹停与卸力 | `policy_waypoints.csv` 的接触、首次低速、停稳、进入跟踪、卸力和恢复事件；停止请求历史区分制动与保持确认 |
| 传感器异常 | 请求编号、主机单调时间、实际总耗时、write/输出等待/read/解析耗时、收发字节数、CRC 失败数、缓冲区及循环最长间隔 |
| 最后有效力 | `last_valid_wrench_host_monotonic` / `last_valid_wrench_age_sec`；主机接收时间不是传感器采样时间，超时快照不是新测量 |
| 配置和标定来源 | `config_snapshot.yaml`、`metadata.json`；实际停稳信息另见 `scan_stop_snapshot.json` 或 `return_status.json` |
| 离线检查与预演 | 菜单 **1** 测试；**14/17** 已有标定检查；**15/16** 连续离线预演/模拟；模拟不能证明真实力方向 |
| 只读设备检查 | 菜单 **3** PX6D、**4** 机器人状态；会连接对应设备，不发运动命令 |
| 回放 | 菜单 **19** 选择明确运行目录；**20** 配合本次箱体标定查看工作空间投影 |

PX6D 使用配置中的串口路径，921600 baud、8N1、100 Hz。先结束其他占用串口的程序。
Linux 上本项目的读取器以排他文件锁打开串口，重复打开会失败；该协作锁不能阻止所有不遵守锁的外部程序。
50 ms 限制整个请求，含节拍、write、输出等待、read 和解析；没有延长超时。
读取失败后立即停止，由 RTDE 独立、有界确认静止；不会继续请求、复用旧力、盲目重试或重新采零。
`rx_bytes=0` 只代表主机未读到字节；结合 `serial_bytes_at_failure` 判断队列积压。
收到部分字节看缓冲长度，候选包无效看 CRC/类型计数；`valid_packet_buffered_at_failure` 只检查缓冲头部。
实际超预算及调度间隔另有记录，不会一律显示成恰好 50 ms。
`tx_bytes=6` 只证明主机写入接受了请求，不能证明 PX6D 已收到。只读工具在失败后被动监听
1 秒，结果见 `passive_after_failure`，期间不发请求、不把迟到包用作新力。
若请求期间和被动监听均为零接收，仍不能区分读取器、USB 驱动/链路和设备固件。
按下方独立串口对照取证，不直接归因线缆，也不修改接触阈值或增大超时。
若 PX6D 初始化就失败、机器人接口尚未连接，停止报告显示 `NOT_CONNECTED`；这不代表真机停稳失败，
也不冒称已通过 RTDE 确认停稳。

本机 2026-10-03 的同口换线实测：原线重新上电后四轮均断流；对照线四轮正常；换回原线
四轮再次断流；最终接回对照线四轮再次正常。对照线共八轮、23719 帧，合计约 240 秒，
零超时、零 CRC 错误。两种独立读取程序、参数、源码、设备及 USB 口均一致，
支持原线或其接头连接问题；不能进一步区分线体、接触、供电压降与信号完整性。
本次改用通过对照的数据线，原线停止用于扫描。最终换回对照线的复核结果和原始收发见
`data/analysis/px6d_compare_20261003/cable_comparison.json`。这些都是静止只读测试，
没有验证长期或运动工况；不能把以后所有超时都归因于线缆。

### 接触后切向为零与卸力

首次接触后停止推进，确认实际停稳和接触后才进入跟踪。高力时切向受抑制，但法向仍按现有反馈卸力：

| 条件 | 当前行为 |
|---|---|
| Fxy ≥ 2.25 N | 暂停切向，法向上限仍为 0.5 mm/s |
| 当前力及 0.25 s 均值均 ≤ 2.0 N、保持 0.1 s | 在 0.5 s 内平滑恢复切向，并保留原指令加速度限制 |
| 卸力 1.5 s 后实际投影位移不足 0.2 mm | `STOP_UNLOAD_NO_MOTION` |
| 已移动，但均值力改善不足 0.1 N | `STOP_UNLOAD_INEFFECTIVE` |
| 累计 3 s 或最大 XY 偏移 1 mm | `STOP_UNLOAD_BUDGET` |

这些是有限软件预算，不能据此断言方向反了；现场仍须考虑位移噪声、滤波、颗粒背景及局部空间。
参考力 1.5 N、死区 0.15 N、增益 0.0005 (m/s)/N 不变。Fxy 是处理后的平面合力，并非纯目标法向力。
不自动扣背景、翻转方向或接触采零。TCP、工作空间及原有速度/力保护不变；原真实模式原始力/矩
60 N / 5 Nm 为硬停止阈值，处理后 12 N / 1 Nm 保持原诊断语义。

### 接触丢失与重接触路径耗尽

低力确认接触丢失后，先停稳，再按原局部弧线重接触。路径预算采用实际平面 TCP 速度模长
按主机 RTDE 观测时间积分，并以本次最大起点位移作为下界；不再把每帧位置往返波动全部算成行程。
`samples.csv` 同时记录 `reacquire_speed_path_length`（速度积分）、`reacquire_raw_pose_path_length`
（原逐帧位置差累计）、`reacquire_max_displacement`（最大起点位移），单位均为米。
生效计数为 `reacquire_path_length`。这些是反馈估计，不能单靠两种计数的差值区分测量噪声与真实微动。

原 4 mm 路径/位移预算及 0.5 mm 预留、0.5 mm/s 速度上限、参考弧长和空间边界不变。
重接触累计达到原配置的 **8 秒**仍未完成时，报告 `STOP_RECOVERY_EXHAUSTED` 并停止；
真实模式也执行此截止，避免没有进展时长期挂起。力重新出现不会重置本轮截止时间。
若重接触时力继续下降，须核对实际安装方向；软件不自动翻转符号或扩大搜索范围。

真实模式的阶段超速告警在新鲜 RTDE 样本恢复到阈值内后清除，当前状态回到 `OK`；
重复时间戳不能清除告警，原超速记录仍保留在此前日志行。全局速度硬限不变。

### 可选方向观察与只读通信诊断

方向观察不是启动前置步骤，不生成验证文件，不绑定配置，不修改旋转矩阵或 `force_direction_sign`。
`rotation_sensor_to_base` 直接将传感器分量旋转到 Base；未知安装旋转不能仅从 TCP 姿态推导。
当前单位矩阵及符号尚未实测；缺少安装关系时显示传感器分量，不能给出 Base 卸力方向。
方向工具默认展示派生的最终扫描姿态，但不会读取或调整机器人姿态；实际姿态不同可用
`--tool-orientation RX RY RZ` 明确输入旋转向量（rad）。观察前必须确认机器人已在显示姿态且空载。

`n = sign * unit(Base XY)` 是估计压入方向；`v = vt*t + vn*n`，力偏大时 `vn < 0`，沿 `-n` 卸力。
从 Base +Z 看，CW 的 `t=R90*n`、目标在右侧，CCW 的 `t=-R90*n`、目标在左侧。
工具显示去偏置的传感器力、处理后 Base 力、n/t 和卸力方向，不把它们宣称为已标定的物理力。

以下命令只连接 PX6D、不连接机器人。机器人静止、探针脱离目标时按需单独运行；方向工具在
初次采零前提示按 Enter，运行中 Q/Esc/Ctrl+C 退出，不提供接触后再次采零入口。

```bash
/home/user-linux/robotics_cs/.venv312/bin/python tools/check_px6d.py --samples 3000 --diagnostics-output /tmp/px6d_readonly.json
/home/user-linux/robotics_cs/.venv312/bin/python tools/check_force_direction.py
```

通信诊断 JSON 在退出时一次写出；没有逐帧同步写盘或大量打印干扰控制循环。

### PX6D 断流：独立读取与 USB 对照

先在项目目录运行系统检查；它不打开串口、不连接机器人：

```bash
/home/user-linux/robotics_cs/.venv312/bin/python tools/check_px6d.py --comparison-dir data/analysis/px6d_compare_20261003 --campaign --system-only --samples 3000 --duration 30 --repeats 2
```

现场停稳且不会自动运动后，在前台终端运行以下命令。先按 Enter 确认停稳，随后每次只按屏幕提示做当前一次操作，完成后单独按 Enter；Q/Esc/Ctrl+C 中止。

```bash
/home/user-linux/robotics_cs/.venv312/bin/python tools/check_px6d.py --comparison-dir data/analysis/px6d_compare_20261003 --campaign --samples 3000 --duration 30 --repeats 2
```

程序先保持现有连接自动做软件对照，此阶段起始上电状态未知，不参与线缆归因。随后按提示进行
原线原口重新连接 → 同口已知正常数据线 → 换回原线。每组分别运行现有读取器和独立
`select + os.read` 程序，各重复两次并交换软件顺序，绝不同时打开串口。为保持起始条件，每轮提示
PX6D 断电至少 5 秒后重连；另有独立供电时也需断开。程序记录这是操作者报告，未测量实际电压。
不会自动复位、重连、采零或连接机器人。每轮最多 30 秒/3000 帧；初始化另有原 2 秒等待和
2 秒版本请求预算，失败后只被动监听 1 秒。50 ms 力请求预算不变。失败证据保存后才提示下一步。

中断后可用相同命令继续：已保存轮次（包括失败轮次）不重跑、不覆盖；不同条件请换新目录。
不加 `--campaign` 可先顺序测试两种软件而不换线，但这不能证明起始上电状态相同，也不作线缆归因。
只在诊断中比较频率可用 `--poll-rate-hz 50` 并换新目录；扫描配置不改。周期大于或等于整个请求
预算的低频输入会被拒绝，避免节拍等待本身制造通信超时。

所有证据在同一目录：`comparison.json`/`conclusion.txt` 汇总各轮；`round_*/wire.jsonl` 保存主机
原始请求、实际 write 接受的字节、各次 read 字节和时刻，退出采集后一次写出；`request.json` 保存条件
与源文件哈希；`result.json` 保存有效帧数、耗时和故障。保留原启动清缓存行为，其丢弃字节数未知，
因此串口 reopen 不等于设备重新上电。原始记录有内存上限，截断会显式标记且不能用于严格对照。

后台另存 USB 拓扑/驱动/该设备省电状态与占用快照、`kernel_events.jsonl`、目标 bus/device 的
`usbmon.log`。权限不足写入 `system_summary.json`，不运行 sudo、不加载模块、不改省电或系统服务。
usbmon 文本可能截短数据，完整 OUT 命令无法逐一匹配时不强行归属请求。成功 OUT 完成只说明
主机 USB 层完成报告，不能证明设备应用处理；IN 返回包含失败后的被动观察时段，不冒充及时新样本。
主机时间不是传感器采样时间；无内核报错也不能证明链路正常。

厂家程序及原协议未找到时，对照程序只是独立实现，不能宣称厂家认证；共享的 USB/pyserial/设备层
仍待核查。同条件原线失败、对照线成功、原线再失败才支持线缆/插接相关；重新上电恢复本身不归功于换线。

### TCP、标定与保存

若遇到不匹配，核对实际 active TCP、机器人 IP、扫描 P0/P1 和箱体四角，勿用随意改坐标或放宽限制消除错误。
原 TCP 和工作空间校验继续保留；不需要新增方向验证记录。

| 原菜单 | 用途 |
|---|---|
| **7** | 手动示教扫描 P0、方向参考 P1；各点停稳后单独按 Enter 读取并保存。只完成 P0 的文件仍是不完整标定 |
| **9** | 读取 active TCP，核对后按 Enter 保存到 `config.yaml` 并备份；不发送 setTcp 或运动命令 |
| **10** | 手动到达复位点并停稳，核对后按 Enter 保存 reset pose |
| **13** | 手动示教四个箱体角点，各点按 Enter 读取；预览后另按 Enter 保存 |

需要只读核对 active TCP 时运行 `tools/read_active_tcp.py`，不加 `--write-config`。
TCP、安装或箱体实际改变后，原相关标定必须与现场一致。机器人连接异常时检查配置 IP、
Remote Control、安全状态和其他控制程序占用。Q/Esc 不响应时检查前台终端焦点，Ctrl+C 中止并等待收尾。

### 离线测试与已有实验重放

以下命令不连接设备、不发送运动命令：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/user-linux/robotics_cs/.venv312/bin/python -m pytest -q
/home/user-linux/robotics_cs/.venv312/bin/python tools/replay_continuous_policy.py data/real/continuous/run_20261002_204147_019798 --output data/analysis/continuous_20261003_204147/replay_direction_fix.json
```

重放使用保存配置和记录输入，跳过独立执行的启动返回段；报告首次指令/状态变化及原停机帧的
新旧路径计数。不能把改变指令后的记录输入重放视为真实闭环轨迹，也不能推断日志结束后的接触结果。
