# UR7e + PX6D 操作手册

## 日常启动与停止

在前台交互终端打开原菜单：

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean
/home/user-linux/robotics_cs/.venv312/bin/python run_project.py
```

1. 选择 **18** 启动连续扫描，按 Enter 打开任务。
2. 核对当前 TCP、P0、搜索方向、探针倾角和完整返回路径。确认机器人静止、基座水平、抬升后有足够旋转空间，再按 Enter：先垂直抬升脱离颗粒、将探针轴调到 Base −Z、移动到 P0 正上方的抬升位置。复用 `safe_return.return_lift_distance`（当前 30 mm）；启动时已经在 P0 也须先抬升。
3. 在 P0 正上方、最终扫描姿态，确认探针脱离目标及颗粒、静止空载，按 Enter 采集 F/T 空气零偏；采零完成后选择贴边速度倍率（有限正数，回车为 1 倍），核对基准、倍率和本次名义速度（mm/s），再单独按 Enter：垂直向下插入到 P0 扫描深度并停稳。倍率仅作用于原配置的名义切向速度，实际速度仍受原受力减速、加速度和限幅影响；超出总速度上限时重新输入。SEARCH 保持 **18 mm/s**。调正前不采零。
4. P0 已确认无目标。在 P0 静止后，再按 Enter 确认“只有颗粒背景、未接触目标”，独立采集 Base-frame F/T 颗粒 baseline，再开始扫描。此处不重采空气零偏，不把目标接触力归入背景。默认 `preprocessing.granular_baseline.sample_count: 100`；结果仅存入本次 `granular_baseline_at_start.json` 和配置快照，不覆盖原配置。扫描扣除的是该深度、姿态下的静态背景；运动产生的额外颗粒阻力仍需实测，不能假定静态 baseline 已消除全部动态阻力。
5. 运行中按 **Q / Esc / Ctrl+C** 原地停止，不自动返回。等待停稳确认和日志保存、回到菜单，再执行下一项；菜单 **0** 退出。

每个确认提示出现后单独按一次 Enter。程序清除积压按键，不接受预先粘贴或管道传入的回车；确认时按 Q/Esc/Ctrl+C 可取消。菜单选择和数值输入仍按原方式填写。探针接触目标时不能采零。
颗粒 baseline 的 `diagnostics` 记录未滤波、空气补偿后的 Base 三轴力/力矩总体标准差及首尾趋势。
默认 100 帧时，趋势为最后 20 帧均值减去最初 20 帧均值；三维力趋势模长超过接触阈值的 25%（当前 0.25 N）仅输出 WARNING 并记录，不拒绝 baseline、不阻止扫描，原安全保护仍生效。
连续真机启动在抬升位置采零之前，原始 F/T 含未确定的零偏、重力及可能的接触载荷。
此时返回力 8 N / 力矩 0.7 Nm 按连续模式原有规则记录诊断，不能把未补偿读数当作补偿后的接触力停止条件；
原始力 60 N / 力矩 5 Nm 硬上限、非有限值及通信故障仍停止。上述数值均未提高。
这不证明原始读数全部属于零偏；到 P0 正上方后仍须确认脱离颗粒、静止空载，才采集扫描零偏。
空气采零后，下降阶段切换为 Sensor 零偏补偿、Sensor→Base 变换后的总载荷，严格使用原返回力 **8 N / 力矩 0.7 Nm** 保护。
该保护不扣颗粒背景，以未滤波的补偿 Base wrench 检查上限，避免背景扣除和滤波延迟掩盖过载；原始 60 N / 5 Nm 硬保护继续生效。
`return_status.json` 记录此切换及 `return_guard_wrench_base`。扫描本身沿用原 Kalman、阈值和力反馈。
倍率选择仅用于本次菜单 18 / 命令行 `--execute`，每次重新读取原配置基准，不修改 `config.yaml`。
`config_snapshot.yaml` 的 `continuous_speed_selection`、`tracking_speed_selection.json` 和逐帧日志保存
`tracking_base_speed_mps`、`tracking_speed_multiplier`、`tracking_nominal_speed_mps`（速度按 m/s 记录）。
搜索、启动/返回、法向补压/卸力、丢边探索速度及原阶段监测阈值均保持不变。
贴边阶段速度 envelope 同步本次名义切向速度，沿用切向速度与原法向速度上限的合成模长；配置快照记录相同值，总 TCP 速度上限继续生效。

连续策略中的 `CONTACT_LOST` 表示“接触减弱持续达到判定条件”，是低力判断，不是物体拐角识别，也不证明完全脱离。
低力停车后，接触恢复确认最多等待 `confirmation_timeout_sec`（当前 1 s）；力仅回到 0.5～低于 1.0 N 或再次短暂下降，不重启该次等待期限。
低力等待期间，低于接触阈值的力方向不进入方向估计或反向判断；达到阈值后复用停稳和稳定接触窗口确认方向，不进入会重置该次期限的 `DIRECTION_RECONFIRM`。可靠接触力下的反向停止保护保持生效。
超时进入 `CONTACT_LOST`，确认停稳后沿用 `LOCAL_REACQUIRE`；未能确认停稳则停止。持续低力仍按原 0.15 s 判定提前进入 `CONTACT_LOST`。
`memory_max_age_sec` 为 **1.5 s**，配置校验要求至少覆盖恢复确认超时、低力判定时间及两个最大采样间隔。
等待期间不刷新真实接触记忆的时间戳；找回被禁用、记忆缺失、确实过期或方向置信度不足时安全停止，不无限等待。
`LOCAL_REACQUIRE` / `REACQUIRED` 的第 N 次表示整次运行中的局部找回轮次，与拐角数量无关。
找回路径角度上限为 90°，沿用原圆弧与方向；半径 2 mm、找回速度 0.5 mm/s、总时间 8 s、
实际路程和位移上限 4 mm、停止余量 0.5 mm 均保持不变。理论弧长 3.142 mm、理想时间 6.283 s
不包含跟随误差和停稳确认，不能据此认定一定够用或保证通过拐角；接触确认期间参考点冻结，总时间预算继续计时。
终止报告的“最后接触记录”来自最后方向有效的记忆样本，不等同于最近一次达到阈值并确认成功的位置；缺失时为 null。

新运行目录如 `run_2026-10-05_17-33-05_896939` 使用北京时间 `Asia/Shanghai`，表示目录创建时间。
metadata 保留 UTC `timestamp`，另记 `timestamp_local` 和 `timezone`；逐帧日志及终止记录同样保留 UTC 并增加本地时间。
菜单按 metadata 创建时间排序并显示北京时间，旧目录无需改名。
停止后生成首次接触至最终停止的 Base XY TCP 全轨迹，以及每次找回放大图；图中不是已知物体外形。
离线回放可指定独立输出目录，保留历史日志及已有图：

```bash
/home/user-linux/robotics_cs/.venv312/bin/python tools/visualize_continuous_run.py \
  data/real/continuous/run_20261005_093305_896939/ --format none \
  --output data/replay_figures/run_20261005_093305_896939/
```

旧运行参考圆弧按旧 `config_snapshot.yaml` 的 60°重建，不读取当前 90°配置；缺少起点或保存方向时不补画参考圆弧。

从 `run_2026-10-06_11-10-51_804801` 起，连续实验结束、设备及日志关闭后，在该 run 内生成
`trajectory_overview.png`、`trajectory_local_zoom.png` 和可执行的 `play_visualization.sh`。
overview 从实际 P0 开始，包括搜索、接触确认、贴边、找回和停止；local zoom 从首次接触确认开始。
两图使用 Base XY 毫米、等比例、宽画布和外部右侧图例，所有找回合并为 `Lost-edge recovery`。
画的是 TCP 实测路径，不补画未知物体轮廓，也不将转向直接标为物体拐角。
cutoff 优先读取 metadata 的目录创建时间，兼容新旧目录名；自动流程只处理刚结束的单次 run，不扫描历史目录。
如果启动取消、尚无扫描样本，两张图会明确说明没有轨迹；不会将启动返回当成扫描数据。

进入 run 文件夹，直接双击 `实验回放.html`，用浏览器打开即可播放、拖动时间轴或点击曲线同步定位，
不需要网络、Tk 或可执行文件的启动授权。页面内嵌显示数据，原始 CSV 不变。
也可双击 `打开实验回放.desktop`；若文件管理器首次提示，选择“允许启动”。
也可运行或以“作为程序运行”打开 `play_visualization.sh`，它使用项目已有虚拟环境调用公共
`visualization/run_animation.py`；不会复制主程序或提前生成 MP4/GIF。也可在终端运行：

```bash
bash data/real/continuous/run_2026-10-06_11-10-51_804801/play_visualization.sh
```

动画默认 1×，提供 Play/Pause、Restart 和 0.5×/1×/2×/4×。固定完整 Base XY 范围，从 P0 播放至停止；
结束保留最终画面，手动关闭。左右两图使用同一 recorded monotonic 时间索引，只显示截至当前时刻的数据。
力优先读取 `filtered_force_base_fx/fy`（历史等价列为 `dfx/dfy`），Fxy 由这两个实时 policy 输入计算；
Sensor/未知 frame 的读数不会冒称 Base 力。`filtered_fxy` 是方向估计器状态，找回期间可能冻结，不能作为实时力曲线。
默认仅显示层按约 40 Hz 取点、界面约 30 FPS；保留状态切换两侧和事件点，不改原始 CSV、控制或分析数据。

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
当前采用现场确认的 `rotation_sensor_to_tool = Rz(+45°)`，Sensor +Z 与 Tool +Z 同向朝下，
矩阵三列分别表示 PX6D +X/+Y/+Z 在当前 TCP/tool 中的方向。旧 Base 单位矩阵仍是占位值，
直接安装矩阵优先生效；`reference_tool_orientation` 保持 null。
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

当前 `sensor_origin_in_tool_m: [0,0,0.024]` 表示从当前 TCP/tool 原点指向 Sensor 原点，
以 tool 坐标表达，暂按两者同轴及 PX6D 图纸的 24 mm 总高度填写。
这是“载荷参考平面中心为 Sensor 参考原点”的暂定假设，不是厂家已确认的原点定义。
该配置启用 `M_B@TCP = R_BS M_S + (R_BT r_TS) × (R_BS F_S)`，力矩参考点结果依赖此假设。
旧 `sensor_origin_in_base_m: [0,0,0]` 不证明原点重合；显式 tool 偏置优先生效。
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
当前控制采用“Base 力为环境作用在探针上的反作用力”的约定，`force_direction_sign=-1`，
即 `n=-unit(Base XY)` 补压、`-n` 沿反作用力方向卸力。首次接触接近 180° 的矛盾记录
支持原符号错误的怀疑，但不能证明当前 45° 安装矩阵或真实传感器方向正确。
符号只参与控制方向计算；首次接触一致性检查及 `STOP_DIRECTION_UNCONFIRMED` 保留。
方向工具默认展示派生的最终扫描姿态，但不会读取或调整机器人姿态；实际姿态不同可用
`--tool-orientation RX RY RZ` 明确输入旋转向量（rad）。观察前必须确认机器人已在显示姿态且空载。

`n = sign * unit(Base XY)` 是估计压入方向；`v = vt*t + vn*n`，力偏大时 `vn < 0`，沿 `-n` 卸力。
从 Base +Z 看，CW 的 `t=R90*n`、目标在右侧，CCW 的 `t=-R90*n`、目标在左侧。
工具显示去偏置的传感器力、处理后 Base 力、n/t 和卸力方向，不把它们宣称为已标定的物理力。

以下命令只连接 PX6D、不连接机器人。机器人静止、探针脱离目标时按需单独运行；方向工具在
初次采零前提示按 Enter，运行中 Q/Esc/Ctrl+C 退出，不提供接触后再次采零入口。
自动运动关闭、探针空载静止且姿态已核对时，先空载采零，再分别沿已知 Base +X/+Y
轻施外力，检查处理后 Base 分量及正负号；对应主要分量应为 +Fx/+Fy，补压 n 应反向。
方向工具不读取实际机器人姿态，须核对显示姿态或提供实际 `--tool-orientation`。

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

## 基础接触实验：完整速度记录与离线回放（2026-10-09）

这次新增功能以 clean 工程的连续入口为主，继续使用现有 RTDE、PX6D、CSV、后台写盘和 Matplotlib。
没有改写跟踪/恢复策略，没有修改 `config.yaml` 中的速度、阈值、安全停止参数，也不连接 ROS2。
仓库中先前未提交的轨迹图和回放改动已保留并扩展。

**先离线检查**，以下命令不连接设备：

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean
/home/user-linux/robotics_cs/.venv312/bin/python tools/visualize_contact_experiment.py --demo --output data/analysis/contact_demo
/home/user-linux/robotics_cs/.venv312/bin/python run_continuous_tracking.py --dry-run --basic-contact --duration 4 --output data/analysis/basic_contact_simulation_check
```

第一条生成明确标为 SYNTHETIC 的示意记录，包含旋转的传感器数据、已知示意目标、不同的实际/指令速度、碰撞减速、丢接触和恢复。
示意信号用于检查记录/回放，不证明机器人或目标的物理行为。第二条运行工程原有模拟器，并走完整保存、停止收尾和自动出图流程。

**现场重新开展基础实验时**，沿用原有标定和逐步确认，明确使用以下入口：

```bash
/home/user-linux/robotics_cs/.venv312/bin/python run_continuous_tracking.py --execute --basic-contact --experiment-metadata experiment.json
```

`--basic-contact` 仅在本次配置副本中关闭颗粒基线采集、把颗粒背景设为零，保留原空气采零、抬升返回路径、固定姿态、P0/P1、搜索和跟踪运动及全部安全保护。
完成原启动流程后，先在 P0 记录至少 1 秒静止 TCP/力，再进入原搜索。预记录不发送运动指令；复用原静止/力保护和已有 watchdog。
预记录使用滤波器副本，不推进实际控制滤波器。随后从第一条搜索指令开始，连续保留全部采集帧，包括碰撞前、接触确认、跟踪、恢复和停车过程。
模拟模式的静止预记录使用 t=0 之前的合成时间，不推进模拟器的运动/策略时钟。

三个实验阶段通过固定目标、原有 P0/P1 示教和实验标签区分。单点实验的不同接近方向仍由原示教方向设置，按原 Q/Esc/Ctrl+C 流程结束；没有新增自动多方向运动或新的单点控制器。
直边和旋转正方体实验同样沿用原运动程序。标签和目标轮廓只进入记录/离线绘图，不参与控制。

可选 `experiment.json` 示例（必须用现场测量值替换示例坐标；未知目标请删掉整个 `target`）：

```json
{
  "kind": "rotated_cube",
  "label": "固定正方体，旋转15度，第1次",
  "approach_direction_base_xy": [0, -1],
  "target": {
    "center_base_mm": [640, 300],
    "side_mm": 50,
    "rotation_deg": 15
  }
}
```

`kind` 可写 `single_point`、`straight_edge`、`rotated_cube`；它是实验标签，不改变策略。
`target` 须同时提供 Base XY 中心、毫米边长和相对 Base +X 的逆时针角度。几何缺失时不画目标轮廓。
这些内容保存于本次 `config_snapshot.yaml` 的 `experiment`，不覆盖源配置、已有标定或历史记录。

### 新数据列及时间语义

- `tcp_x/y/z/rx/ry/rz`：实际 TCP 位姿，m / 旋转向量 rad。
- `tcp_vx/vy/vz/vrx/vry/vrz`：真机继续由 RTDE `getActualTCPSpeed()` 读取完整六维实际速度，m/s / rad/s；后三列对应 wx/wy/wz，不是姿态差分。
- `actual_tcp_timestamp`：这次 RTDE 状态读取的主机单调时间；`actual_velocity_source` 区分 RTDE 与 simulation。
- `commanded_tcp_vx/vy/vz/wx/wy/wz` 和 `commanded_tcp_timestamp/sequence/kind/accepted`：最近一次实际调用的笛卡尔速度输入，包含制动的六维零指令。它可能在这一行 RTDE 读取之后发送，所以**不得把行号当成两者严格同时发生**。
- `tcp_commands.csv`：每一次实际 `speedL` 输入及其独立的发送/返回时间、序号、SDK 接受结果、Base frame 和单位。回放按此表的发送时间绘制阶梯曲线，查询实际采样时刻之前最后一次发送的指令；不使用策略的 `command_v*` 或 `commanded_speed_mps` 冒充下发值。SDK 返回未知/失败仍按原样保留。
- `raw_fx/fy/fz/tx/ty/tz`：原始 Sensor 六维数据；`raw_base_*` 是同一数据按现有安装关系/实际 TCP 姿态转换后的六维 Base 数据，未减空气偏置/颗粒基线。
- `force_base_fx/fy/fz/tx/ty/tz`：补偿后、滤波前的 Base 数据；`filtered_force_base_*` / 原 `df*/dt*`：实际传给策略的 Base 数据。力矩参考点沿用现有变换和配置假设，范围由原 `force_transform_at_start.json` 说明。
- 原 `monotonic_sec`、UTC/北京时间、串口读起止时间、RTDE 读起止时间和设备时间诊断保留。真机行的 UTC/北京时间由采集时刻映射，不使用后台落盘时刻替代。

PX6D 没有设备采样时间；串口与 RTDE 为顺序读取。日志保留各自获取区间，统一采用主机单调时间作为回放基准，不能声称硬件同步。
轨迹/力按记录行时间、实际速度按实际 RTDE 读时间、指令速度按真正发送时间绘制；不插值补造数据。
首次碰撞附近的“接触前实际速度”取触发接触的记录帧**之前**的最后一帧，即使接触帧 RTDE 读时间略早于策略事件，也不会把它误标成碰撞前样本。

原 `policy_waypoints.csv` 中 `FIRST_THRESHOLD_STOP_REQUEST` 是最初触发接触停车的事件，优先用于碰撞观察；`FIRST_CONTACT` 是接触确认完成，`TRACKING_ENTERED` 是进入跟踪。
丢接触、恢复、停车请求及停车确认继续使用真实记录事件。滤波/串口延迟可能让物理碰撞早于阈值事件。
历史数据只有 `FIRST_CONTACT` 时明确标为确认时刻；没有事件时不从转弯或 TRACKING 状态推断碰撞时刻。
历史速度缺失就显示 N/A；历史采集时间不够 1 秒就提示不足，不补造数据。

连续真机仍在现有后台线程保存原始 CSV；显示层减点不改动原始文件。新运行开启 `lossless`：超过原队列容量时保留积压记录于内存并给出诊断，不通过丢原始帧或阻塞控制线程处理积压。
停止后排空记录再生成报告，结束时保存 `logging_status.json` 的完整性/写盘诊断。持续磁盘故障无法保证完整保存，必须检查 `complete`；尚未关闭的写盘线程不会被当作完整数据自动出图。

### 自动生成文件与交互

每次连续实验结束、设备和日志关闭后，在原 `data/real/continuous/run_*` 或模拟运行目录生成：

| 文件 | 内容 |
|---|---|
| `trajectory_overview.png`、`trajectory_local_zoom.png` | 保留原轨迹图；Base XY 毫米、等比例、外置图例；已知目标才画轮廓 |
| `experiment_overview.png` | PPT 总览：阶段轨迹、关键位置、力/阈值、实际/下发速度，以及接触前后速度与力数值 |
| `first_contact_zoom.png` | 接触前约 1 秒至接触后约 1 秒的力和速度同步放大 |
| `contact_lost_NN_zoom.png` | 每次丢接触附近的力/速度放大，恢复事件用同一时间基准标记 |
| `contact_analysis.json` | 首次接触来源、接触前实际/下发速度及时间、接触后观测、接近方向与力方向的夹角、丢失/恢复附近实测值、速度误差、采样间隔及保存的终止/写盘诊断 |
| `play_visualization.sh` | 可执行的二维离线回放入口，不连接机器人 |
| `实验回放.html` | 推荐的双击入口：浏览器离线联动回放，文件内含显示数据，无需网络或 Tk |
| `打开实验回放.desktop` | 文件管理器双击入口，自动加载所在文件夹的实验 |

直接双击实验文件夹里的 `实验回放.html`，即可在浏览器查看。
`打开实验回放.desktop` 和不带参数运行的 `play_visualization.sh` 也会优先打开这个文件；
若启动失败，终端保留错误并等待回车。仍可运行原 Matplotlib 窗口：

```bash
/home/user-linux/robotics_cs/.venv312/bin/python visualization/run_animation.py --run-dir data/real/continuous/你的run目录
```

回放需要桌面与 Tk。窗口提供播放/暂停、重启、0.5/1/2/4倍速、拖动 Time 滑块、点击力/速度图定位。
速度图以实线/虚线分别显示实际/下发速度，All/vx/vy/XY 可选；Contact zoom / Lost zoom / Full time 同步改变力与速度的时间范围，反复点击 Lost zoom 查看各次丢边。
左侧轨迹显示当前 TCP、阶段、首次阈值点、丢失/恢复点与最终位置；紫色力箭头、蓝色实际速度箭头从 TCP 出发，固定独立缩放，旁边显示 mm/N 与 mm/(mm/s)。
小方向区域和实时 Fx/Fy/Fxy、实际/指令 vx/vy/XY 数值同步更新。箭头从 Base 实测力和实际速度计算，不加入控制方向符号。
默认界面 30 FPS、按约40 Hz选择显示点，并保留状态变化、事件和每个显示时间桶的力/速度极值；静态图和分析仍使用完整原始帧。

### 历史实验重放及本次离线检查

显式指定旧数据；`--output` 写入独立分析目录，不改历史 CSV、配置或已有图：

```bash
/home/user-linux/robotics_cs/.venv312/bin/python tools/visualize_contact_experiment.py \
  data/real/continuous/run_2026-10-06_11-10-51_804801 \
  data/real/continuous/run_2026-10-08_14-08-05_791694 \
  --output data/analysis/basic_contact_history_review
```

每个 run 有独立报告/放大图/启动脚本，多次记录另有 `experiment_comparison.json`。外置启动脚本指向原始数据目录，不复制或改写原始数据。
旧记录的精确指令只读取已保存的 `speedl_host_monotonic/speedl_vx/vy` 或新 `tcp_commands.csv`；旧记录没有保存的其余四维仍不可用。

本次已执行示意数据、原模拟器和以上两份历史记录的离线检查。原模拟器在约2.75秒达到原有处理后力保护（约12.067 N）并以 `STOP_FORCE_LIMIT` 停止；停止后的数据、报告和回放入口正常生成，没有放宽保护来延长模拟运动。历史观察：

- 10月6日 11:10 那次共有8次丢接触、7次记录到恢复。第8次丢接触前约1秒 Fxy 为0.619 N，丢失事件时为0.344 N，实际 XY速度约0.043 mm/s，最后一次已下发速度为0。
  最后恢复样本的路径计数约3.495 mm、最大位移2.655 mm、角度85.55°、Fxy约0.450 N；保存的停止原因是 `local reacquire path budget exhausted`。
  这次旧策略的路径计数不能当作新版速度积分计数；应结合原快照、原预算和实际速度分析，不直接用它认定真实路径长达3.495 mm。
- 10月8日 14:08 那次丢接触时 Fxy约0.474 N，实际XY速度约0.075 mm/s，已下发速度为0；恢复末帧Fxy约0.929 N、角度22.22°、路径计数0.816 mm，累计约7.998秒，实际和下发速度均为0。
  保存的停止原因是 `local reacquire time budget exhausted`。力再次出现不等于达到原接触确认条件，恢复中的等待也会消耗原总时间预算。
- 两次首次触发接触前实际XY速度分别约18.282和17.848 mm/s，已下发XY速度均为18.000 mm/s；约0.1秒后实际速度分别约1.598和1.857 mm/s。

这些数据支持继续检查低力停车、恢复等待与原预算之间的关系；未知物体轮廓不能由TCP转向补画，也不能仅凭轨迹确认真实接触了几条边。
本次没有改动恢复角度、速度、阈值或预算，没有用旧输入做反事实轨迹预测。

本次主要改动文件：`robot/rtde_controller.py`、`sensor/force_preprocess.py`、`experiment_logging/data_logger.py`、`continuous_writer.py`、新增 `contact_capture.py`、`app/continuous_runtime.py`、离散入口的指令日志接入、`simulation/simulated_robot.py`、`tools/visualize_continuous_run.py`、新增 `tools/visualize_contact_experiment.py`、扩展 `visualization/run_plots.py` / `run_animation.py`、新增 `visualization/contact_report.py`，以及对应离线测试。
