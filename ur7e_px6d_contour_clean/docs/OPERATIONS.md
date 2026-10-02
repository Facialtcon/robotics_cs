# UR7e + PX6D 操作手册

## 日常启动与停止

在前台交互终端打开原菜单：

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean
/home/user-linux/robotics_cs/.venv312/bin/python run_project.py
```

1. 选择 **18** 启动连续扫描，按 Enter 打开任务。
2. 核对程序显示的当前 TCP、P0、搜索方向及返回路径。若不在 P0，按提示分别确认空载采零、自动返回；已在 P0 时跳过返回。
3. 在 P0 确认探针脱离目标、静止空载，按 Enter 采集扫描零偏；采零完成后，再按 Enter 开始扫描。SEARCH 保持 **18 mm/s**。
4. 运行中按 **Q / Esc / Ctrl+C** 原地停止，不自动返回。等待停稳确认和日志保存、回到菜单，再执行下一项；菜单 **0** 退出。

每个确认提示出现后单独按一次 Enter。程序清除积压按键，不接受预先粘贴或管道传入的回车；确认时按 Q/Esc/Ctrl+C 可取消。菜单选择和数值输入仍按原方式填写。探针接触目标时不能采零。

原菜单 **11** 用于手动返回 reset（没有 reset 文件则返回 P0）：按 Enter 打开任务，核对路径后再按 Enter 执行。手动返回不连接 PX6D；自动启动返回有 PX6D 力监控。返回中 Q/Esc/Ctrl+C 中止。

原菜单 **12** 是离散扫描，启动同样逐步按 Enter；Q 按原配置正常停止并可能返回，Esc/Ctrl+C 不自动返回。急迫危险使用现场物理急停。

## 故障排查

### 日志与现有菜单

停止后先查看终端打印的本次运行目录。数据在 `data/real/continuous/`、`data/real/discrete/`、`data/real/return/` 或 `data/simulation/`，目录时间为 UTC。

| 项目 | 查看或操作 |
|---|---|
| 停止原因 | `termination.json` / `termination.txt` 的 reason、detail、phase；`summary.json` 的停止确认结果 |
| 为什么不动 | `samples.csv` 的 `tangent_limit_reason`、处理后力、n/t、指令和实际 TCP 速度/位移；`software_warnings` 是历史告警 |
| 接触刹停与卸力 | `policy_waypoints.csv` 的接触、首次低速、停稳、进入跟踪、卸力和恢复事件；停止请求历史区分制动与保持确认 |
| 传感器异常 | 请求编号、主机单调时间、实际总耗时、write/输出等待/read/解析耗时、收发字节数、CRC 失败数、缓冲区及循环最长间隔 |
| 最后有效力 | `last_valid_wrench_host_monotonic` / `last_valid_wrench_age_sec`；主机接收时间不是传感器采样时间，超时快照不是新测量 |
| 配置和标定来源 | `config_snapshot.yaml`、`metadata.json`；实际停稳信息另见 `scan_stop_snapshot.json` 或 `return_status.json` |
| 离线检查与预演 | 菜单 **1** 测试；**14/17** 已有标定检查；**15/16** 连续离线预演/模拟；模拟不能证明真实力方向 |
| 只读设备检查 | 菜单 **3** PX6D、**4** 机器人状态；会连接对应设备，不发运动命令 |
| 回放 | 菜单 **19** 选择明确运行目录；**20** 配合本次箱体标定查看工作空间投影 |

PX6D 使用配置中的串口路径，921600 baud、8N1、100 Hz。先结束其他占用串口的程序。
50 ms 限制整个请求，含节拍、write、输出等待、read 和解析；没有延长超时。
读取失败后立即停止，由 RTDE 独立、有界确认静止；不会继续请求、复用旧力、盲目重试或重新采零。
`rx_bytes=0` 只代表主机未读到字节；结合 `serial_bytes_at_failure` 判断队列积压。
收到部分字节看缓冲长度，候选包无效看 CRC/类型计数；`valid_packet_buffered_at_failure` 只检查缓冲头部。
实际超预算及调度间隔另有记录，不会一律显示成恰好 50 ms。

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

### 可选方向观察与只读通信诊断

方向观察不是启动前置步骤，不生成验证文件，不绑定配置，不修改旋转矩阵或 `force_direction_sign`。
`rotation_sensor_to_base` 直接将传感器分量旋转到 Base；未知安装旋转不能仅从 TCP 姿态推导。
当前单位矩阵及符号是否符合实物，仍须现场判断。

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
/home/user-linux/robotics_cs/.venv312/bin/python tools/replay_continuous_policy.py data/real/continuous/run_20261002_121516_883033 --output data/analysis/continuous_replay_20261002.json
```

重放使用保存配置和记录输入，不再区分“方向已验证/未验证”。记录仅包含约 0.63 s 跟踪，
不能外推 1.5 s 之后的卸力效果，也不能把改变指令后的记录输入重放视为真实闭环轨迹。
