# UR7e + PX6D 操作手册

## 1. 用途与目录

本项目使用 UR7e 机械臂和 PX6D 六维力传感器进行平面轮廓搜索与接触跟踪。正式连续实验使用 continuous tracking；项目同时提供离散扫描、离线仿真和结果回放。UR7e 通过 Python `ur-rtde` 库进行 RTDE 状态读取与控制，**不依赖 ROS2**。

统一入口为 `python run_project.py`。本文的相对路径均以工程根目录为基准：

```text
/home/user-linux/robotics_cs/ur7e_px6d_contour_clean
```

若 clone 或下载到其他位置，请在命令中使用实际目录。

| 路径 | 用途 |
|---|---|
| `run_project.py` | 统一操作菜单 |
| `config.yaml` | 设备连接、TCP、实验与记录配置 |
| `scan_calibration.yaml` | 扫描 P0/P1、固定 Z/姿态和 TCP 绑定 |
| `workspace/config/workspace_calibration.yaml` | 箱体四角、sandbox polygon 与工作空间坐标变换 |
| `reset_pose.yaml`（可选） | 独立复位目标及 TCP 绑定 |
| `app/`、`policy/`、`robot/`、`sensor/`、`safety/` | 实验运行、控制和设备代码 |
| `calibration/`、`config/`、`core/`、`experiment_logging/`、`workspace/` | 标定、配置、数据模型及记录工具 |
| `tools/`、`simulation/`、`tests/` | 检查工具、离线仿真和测试 |
| `data/` | 自动创建的实验数据目录，不纳入 Git |

## 2. 环境与安装

使用 Linux、Python 3.12 和可交互终端。真机实验需要机器人以太网连接及 PX6D USB 串口访问权限；交互预演需要图形桌面及可交互的 Matplotlib 后端。无桌面环境可使用菜单 16 的无窗口模拟。

首次使用时，在工程内创建虚拟环境并安装依赖：

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

依赖包括 NumPy、Matplotlib、Pillow、PyYAML、pyserial、ur-rtde 和 pytest。若系统没有 `python3.12` 或 `venv` 模块，先通过系统的软件管理方式安装相应组件。无需启动 ROS2 或加载 ROS2 环境。

以后每次打开终端：

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean
source .venv/bin/activate
python run_project.py
```

菜单 1 运行离线测试，不连接设备。也可直接执行：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
```

## 3. 真机连接前提

### UR7e

- `config.yaml` 的 `robot.robot_ip` 当前为 `192.168.1.10`。确认它对应现场机器人，计算机与机器人之间网络可达。
- 在示教器上确认机械臂已上电、制动已释放，且没有 emergency stop 或 protective stop。按照实验室操作规程处理已有安全停止，不绕过 UR 的安全系统。
- 运行外部运动程序前，启用并切换到 **Remote Control**。PolyScope 5 的入口为 Settings → System → Remote Control → Enable，再在模式菜单中选择 Remote Control；仅点 Enable 不等于已经切换模式。参见 [UR Remote Control 操作说明](https://www.universal-robots.com/manuals/EN/HTML/SW5_21/Content/prod-usr-man/software/PolyScope/content/hamburger_menu_g5/System_remote_en.htm)。
- 确认没有其他程序同时控制机器人。需要示教器移动或 Freedrive 时先结束本程序并切回 Local Control；示教完成后，真机扫描前再切回 Remote Control。
- 菜单 **4** 只读取机器人状态，不发送运动命令。检查终端显示的机器人地址、TCP pose/speed、robot mode 和 safety mode。

### PX6D

将 PX6D 接入计算机，确认设备存在、当前用户具备串口读写权限，且没有其他程序占用串口。配置使用稳定设备路径 `sensor.serial_port`，当前为：

```text
/dev/serial/by-id/usb-GigaDevice_GD32-CDC_ACM_1B556A774A92-if00
```

连接设置为 921600 baud、8N1，轮询频率为 100 Hz。更换传感器或计算机后，应核对实际设备路径；不要直接套用其他设备的地址。

菜单 **3** 读取 500 个 PX6D 样本，显示各轴统计值及采样率；不连接机器人，也不执行传感器硬件清零。检查读数有效、无通信异常，完成后再启动扫描，避免两个程序同时打开串口。

## 4. TCP 与标定检查

### TCP

先在示教器上确认当前 active TCP 对应实际探针尖端。以下命令连接机器人读取 active TCP，**不发送运动命令、不写配置**：

```bash
python tools/read_active_tcp.py
```

检查输出 `config_match=true`，并核对 active TCP 与 `config.yaml` 的 `tcp.offset`、扫描标定和复位点中的 TCP 绑定一致。位置单位为 m，旋转向量单位为 rad。工具通过机器人只读 30012 状态接口读取 TCP；RTDE 状态读取正常而该工具失败时，也需检查该端口是否可达。

**菜单 9 会将当前 active TCP 写入 `config.yaml`，发生改动时生成备份。** 它用于有意更新现场 TCP 配置，不是日常只读检查。TCP 发生实际变化时，由实验负责人重新建立对应标定，不能仅改数值以消除不匹配提示。

### Scan calibration

`scan_calibration.yaml` 定义扫描起点 P0、方向参考点 P1、初始搜索方向 P0 → P1、固定 Z/姿态，并记录机器人 IP 和 active TCP。P1 用于定义方向，不是搜索终点。真机连续搜索范围还受 sandbox polygon 约束。

菜单 **8** 或以下命令离线检查文件，并显示 P0、P1、方向和绑定 TCP：

```bash
python tools/check_scan_calibration.py
```

确认保存的 P0、Z、姿态和搜索方向符合现场布置。扫描标定缺失、不完整或已不适用时，才使用菜单 **7** 重新采集：由操作者用示教器将机器人移动到 P0，静止后输入 `SAVE_P0`；再沿初始搜索方向移动到 P1，静止后输入 `SAVE_DIRECTION`。采集程序不发送运动命令，但会写标定文件。保存 P0 后文件即处于未完成状态，必须完成 P1 才能用于扫描。

### Workspace calibration

`workspace/config/workspace_calibration.yaml` 保存箱体四角及坐标变换。连续真机使用其中的原始四角 XY 多边形作为 sandbox 范围；工作空间坐标变换还用于结果显示。这里的四角 P0/P1/P2/P3 与扫描标定的 P0/P1 含义不同。

菜单 **14** 或以下命令离线检查几何一致性：

```bash
python tools/check_workspace_calibration.py
```

核对四角顺序、箱体位置及尺寸与现场一致。离线检查通过仅表示文件和几何满足程序要求，不证明现场箱体没有移动或 active TCP 正确。需要重新采集时使用菜单 **13**：按提示由示教器依次定位四角并读取静止位置，核对输出后输入 `SAVE`。程序不发送运动命令，会保存标定并备份已有文件。

菜单 **17** 联合检查连续实验配置、扫描标定、sandbox 和搜索范围，也可运行：

```bash
python run_continuous_tracking.py --check-calibration
```

以上离线检查不连接设备、不重建标定。日常实验优先检查已有标定，不必重复采集。

### Reset pose

`reset_pose.yaml` 是可选的独立复位目标，包含完整 TCP pose、机器人 IP 和 active TCP 绑定，不替代扫描 P0。需要保存时，由操作者先用示教器将机器人移动到合适位置并停稳，再用菜单 **10** 核对输出并输入 `SAVE_RESET_POSE`。该菜单只读取机器人并写文件。

菜单 11 有 reset 文件时优先返回 reset；没有该文件时返回扫描 P0。始终检查返回程序显示的目标来源和姿态。

## 5. 菜单与推荐顺序

| 菜单 | 功能 | 设备与写入行为 |
|---|---|---|
| 1 | 运行测试 | 离线，不连接设备 |
| 3 | PX6D 读数检查 | 只连接传感器 |
| 4 | UR 状态检查 | 只读机器人，无运动 |
| 8、14、17 | 扫描、工作空间、连续实验联合检查 | 离线，只读配置与标定 |
| 15 | 连续交互预演 | 离线，需要图形桌面 |
| 16 | 连续有限时长模拟 | 离线，无窗口；默认输入时长 12 s |
| 18 | 连续真机扫描 | **会使机器人运动，包括必要的启动返回** |
| 11 | 返回 reset 或 P0 | **会使机器人运动，不连接 PX6D** |
| 19、20 | 选择运行记录并回放/工作空间投影 | 离线，在所选运行目录中生成结果 |
| 7、13 | 采集扫描/箱体标定 | 只读机器人，写标定；定位由示教器操作 |
| 9 | 保存当前 active TCP 到配置 | 只读机器人，写 `config.yaml` 及备份 |
| 10 | 保存 reset pose | 只读机器人，写复位文件 |
| 2、6 | 离散仿真与可视化 | 离线，不连接设备 |
| 5 | 真实 PX6D 驱动离散 dry-run | 连接真实传感器，机器人为模拟对象 |
| 12 | 离散真机扫描 | **会使机器人运动** |
| 0 | 退出菜单 | 不启动任务 |

连续实验推荐顺序：**1 → 8/14/17 → 16（或 15）→ 3/4 → 只读 TCP 检查 → 18 → 19/20**。7、9、10、13 是配置或标定写入操作，仅在确有需要时使用。菜单 11 用于需要单独返回目标的情况。

## 6. Continuous tracking 真机操作

### 运行前检查

1. 完成设备读取、TCP 和标定检查；探针、传感器、箱体与标定所对应的布置一致。
2. 机器人静止，探针在扫描姿态下无目标接触，零偏采集期间保持空载。启动程序会采集软件零偏；不要在接触目标时把接触力归零。
3. 连续真机连接前必须有与当前配置匹配的实测方向记录。执行下方 `--verify` 工具：先在保存的扫描姿态下静止空载，输入 `UNLOADED` 采集唯一一次软件零偏，然后按提示施加 Base ±X、±Y、±Z 方向的小力。这里的方向指**施加在探针上的力向量**，不是从哪一侧推来。工具只连接 PX6D，不连接 UR、不修改配置或符号；过程中不提供重新采零入口。验证失败时不启动跟踪。
4. 核对从当前位置到 P0 的完整路径，包括抬升、在安全高度旋转、平移和下降的空间。自动返回没有障碍物碰撞规划。
5. 使用前台交互终端运行，保证 Q/Esc 可被读取；操作者留在现场并能操作示教器急停。

```bash
/home/user-linux/robotics_cs/.venv312/bin/python tools/check_force_direction.py --verify
/home/user-linux/robotics_cs/.venv312/bin/python tools/check_force_direction.py --status
```

默认记录为 `calibration/force_direction_verification.json`。记录绑定传感器设备路径、预处理、TCP、
`force_direction_sign` 和扫描标定文件；这些配置改变后旧记录失效。安装被实际改变但文件未变时，
软件无法自行知道，操作者必须重新核对。`--status` 完全离线，未通过时退出码为 1。
首次运行前没有实测文件是正常的“未验证”，单位矩阵不会自动被认作已标定。

六个方向分别采 50 帧，检查去偏置力模长 0.5～3 N、方向误差不超过 15°、三轴合成标准差
不超过 0.15 N，避免用近零或不稳定信号判断符号。它们是核对工具的判据，不是控制力限；
现场若无法稳定施加这些已知方向，保留失败记录并使用实际安装测量校准，不能伪造通过记录。
静止/空载/已知方向依赖现场确认，单个恒定负载无法仅靠传感器自己识别为“错误采零”。

### 力方向、卸力和恢复的约定

`R_BS = rotation_sensor_to_base` 直接把传感器分量转换为 Base 分量，仅适用于保存的固定扫描姿态。
若从工具安装标定组成它，则 `R_BS = R_BT @ R_TS`；TCP 姿态只提供 `R_BT`，不能推导未知的
`R_TS`。力只旋转；力矩另外使用已配置的原点偏移项 `r × F`。工具不根据本次力上涨拟合或翻转安装方向。

预处理顺序保持为：空载偏置 → EMA → 配置重力项 → 坐标变换 → 配置颗粒背景基线。
`Fxy` 是处理后的平面合力，包含无法分离的摩擦、背景及偏差，不是纯目标法向力。
没有新增自动扣背景、目标接触采零或目标力调整。空载偏置已经包含当时的静态重力；
另设重力/背景项时必须确认其意义，不能把同一静态负载扣两次。

`n = force_direction_sign * unit(filtered Base XY)` 定义为估计压入/朝目标方向；卸力沿 `-n`。
若转换后的传感器报告的是环境作用于探针的力，所需 sign 为 -1；若报告相反作用方，所需 sign 为 +1。
这不是对当前实物符号的判断。工具用实际施力检查配置，失败时由现场标定修正后重新验证，不自动改值。
`v = vt*t + vn*n`；力过大时 `vn < 0`。从 Base +Z 看 XY，CW 的 `t=R90*n`、目标在右侧；
CCW 的 `t=-R90*n`、目标在左侧。代码在 n 更新时一起重算 t，不能单独把 n 再取反。
可视化中的 n 是压入方向估计；实测作用力箭头依据验证记录中的作用方转换，独立于 n/t，避免重复取反。

首次接触阈值到确认期间始终发送零速度请求，需实际 TCP 低速持续保持且执行器确认停止后才进入跟踪。
记录“首次低速观测”和“持续静止确认”，二者不是同一个时刻。SEARCH 仍是 18 mm/s，减速度未提高。

| 卸力参数 | 当前值和依据 |
|---|---|
| 切向暂停 | 2.25 N，保持原阈值；较低过载区仍按原 load_scale 降低切向 |
| 法向反馈 | 原参考 1.5 N、死区 0.15 N、增益 0.0005 (m/s)/N、上限 0.5 mm/s |
| 恢复条件 | 当前力与 0.25 s 均值都不超过 2.0 N，持续 0.1 s；与暂停阈值相差 0.25 N |
| 恢复速度 | 0.5 s 渐变，同时保留原 0.01 m/s² 指令变化上限；没有提高任何速度上限 |
| 无效评估 | 1.5 s 后，实际卸力投影位移至少 0.2 mm，力均值至少改善 0.1 N |
| 总预算 | 从该次卸力开始累计 3 s 或最大 XY 偏移 1 mm，间歇改善不会重置 |

0.2 mm 采用已有位置容差尺度；以 0.5 mm/s 走这段距离至少需要 0.4 s，此外还有指令加速、
EMA 的约 0.206 s（100 Hz、alpha=0.8、99%阶跃收敛）和 0.25 s 力趋势窗口。1.5 s 提供观测余量，
不再把 0.5 s 的几帧上涨判成方向错误。这些是保守软件预算，不代表测得了目标刚度或现场位移噪声；
现场仍需确认 0.2 mm 是否可可靠观测、1 mm 的局部卸力空间是否可用。原 sandbox 保护继续生效。

明确退出原因：`STOP_UNLOAD_NO_MOTION`（卸力方向实际位移不足）、`STOP_UNLOAD_INEFFECTIVE`
（已移动但平滑后的合力未改善）、`STOP_UNLOAD_BUDGET`（总时间/空间预算耗尽）。不动态反转方向。
没有方向记录则 `STOP_FORCE_DIRECTION_UNVERIFIED`，真实入口在构造设备前拒绝启动；策略层也保留停止检查。
原真实模式原始力/矩硬停止阈值仍为 60 N / 5 Nm；处理后 12 N / 1 Nm 在原真实模式中是诊断阈值，
本轮没有把它们抬高、冒称为原有硬停止或改动这层既有语义。卸力新预算是额外的有限退出条件。

### 串口时限与“为什么不动”的记录

上一轮的完整内部包解析、跨请求残留拒绝、失败后禁止继续请求、CRC/类型校验和有界内存诊断均保留。
50 ms 现在约束整次主机请求（节拍等待、非阻塞 write、输出队列等待、read、解析），没有延长。
不再调用可能无限等待的 `flush()/tcdrain`，而在同一截止时间内轮询 `out_waiting`；短写直接失败。
通用操作系统调度仍可能造成实际耗时超过预算，诊断同时保留实际总耗时、各阶段耗时及循环最长间隔，
不会把长调用仍只标成“耗时 50 ms”，也不接受截止时间后才完成的包。无序号协议不能证明任意迟到重复包的新鲜度。
错误诊断只检查有限缓冲前缀，避免错误后的证据扫描延迟刹停；`buffer_candidate_scope=head only`。

串口失败立即走既有刹停链路，后续仅用 RTDE 确认静止，不重读失败传感器、不恢复请求、不重新采零。
RTDE 确认也有原有超时，不能无限等待。`termination.json` 的 `failure_host_monotonic` 是异常时刻；
`last_valid_wrench_host_monotonic`、`last_valid_wrench_age_sec` 明确标记缓存样本。所有接收时间均为主机时间。

`policy_waypoints.csv` 新增低速、静止确认、进入跟踪、卸力开始/解除、切向恢复、卸力退出和传感器失败事件。
`samples.csv` 的 `tangent_limit_reason` 是当前限制；`software_warnings` 保留历史告警，不能当作当前仍生效的限制。
原力、n/t、指令与实际 TCP 数据保留，并增加实际 n/t 速度、接触后位移、卸力位移/力趋势/预算耗时。
停止请求历史包含请求、首次低速、静止保持起点和确认时间。磁盘仍使用既有异步记录队列，不逐帧打印。

以下命令前两条完全离线；后两条仅连接 PX6D，需在现场探针脱离目标且机器人静止后执行：

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/user-linux/robotics_cs/.venv312/bin/python -m pytest -q
/home/user-linux/robotics_cs/.venv312/bin/python tools/replay_continuous_policy.py data/real/continuous/run_20261002_121516_883033 --output data/analysis/continuous_replay_20261002.json
/home/user-linux/robotics_cs/.venv312/bin/python tools/check_px6d.py --samples 3000 --diagnostics-output /tmp/px6d_readonly.json
/home/user-linux/robotics_cs/.venv312/bin/python tools/check_force_direction.py --verify
/home/user-linux/robotics_cs/.venv312/bin/python tools/check_force_direction.py --status
```

重放同时输出“保留方向未验证事实”和“仅离线假设方向已验证”两种结果。后者仅用于隔离比较卸力逻辑，
不是绕过现场验证的开关。旧记录仅有约 0.63 s 跟踪，不能外推 1.5 s 后的真机卸力结果。
任何改变指令后的记录输入重放都不代表新控制策略的闭环真实轨迹。这里没有连接机器人或运行运动的命令。

### 启动

1. 执行 `python run_project.py`，选择 **18**，输入 `OPEN_CONTINUOUS_SCAN`。
2. 检查终端显示的标定来源、P0、搜索方向、到 sandbox 边界的几何距离、停车余量及有效搜索距离。
3. 程序连接 PX6D 和机器人，检查静止状态，显示当前 TCP、目标 P0、姿态误差和返回路径。此时核对现场位置与显示一致。
4. **输入 `START` 后允许实际运动。** 若当前位置或姿态不满足 P0 要求，程序先在当前静止位置采集返回用零偏，然后自动返回 P0；已在 P0 且姿态对正时跳过返回运动。
5. 自动返回按以下顺序执行：沿 Base +Z 抬升并保持当前姿态 → 停稳后在安全高度对正保存的扫描姿态 → 平移至 P0 上方 → 下降到 P0。姿态已经对正时跳过旋转，各段均需停稳后才继续。
6. 到 P0 后采集扫描零偏，沿保存的 P0 → P1 方向以 **18 mm/s** 搜索。`Fxy >= 1 N` 时进入首次接触，先停止搜索并确认实际停稳与接触，再进入连续跟踪。

首次接触停车后接触消失时，程序重新搜索；跟踪中持续失去接触时，程序停稳后执行 local reacquire，恢复接触并确认后继续跟踪。恢复空间预算耗尽可能结束实验。

连续真机默认没有 120 s 自动结束；`--duration` 只用于离线模拟或预演。运行过程中关注真实接触状态和终端输出，不以出现 WARNING 判断实验已经停止。

### 正常停止

在运行终端按 **Q**（大小写均可）、**Esc** 或 **Ctrl+C**。连续真机模式原地停止，**不会自动返回 P0 或 reset pose**。等待程序完成停稳确认、日志保存并返回菜单，再启动下一项任务。若存在即时危险，使用 UR 的物理急停；终端按键不是物理急停的替代品。

离散真机菜单 12 的停止行为不同：Q 按配置执行正常停止及可能的自动返回，Esc/Ctrl+C 不自动返回。不要用离散模式的返回行为推断连续模式。

## 7. 返回 P0 或 reset pose

以下操作均会使机器人实际运动，执行前先结束扫描并确认停稳。

- **返回 reset，缺省返回 P0**：菜单 **11** → 输入 `OPEN_REAL_RESET` → 核对目标来源、完整路径和姿态 → 输入 `RETURN`。
- **明确返回扫描 P0，即使 reset 文件存在**：在工程目录执行 `python return_to_start.py`，核对目标和路径后输入 `RETURN`。
- `python return_to_reset.py` 与菜单 11 使用相同的目标选择规则；直接命令行入口仍需输入 `RETURN`。

手动返回执行抬升、对正目标姿态、平移、下降四段动作；目标姿态来自所选 reset 或扫描 P0。**手动返回不连接 PX6D，没有外部力监控**；continuous 启动时的自动返回则包含 PX6D 监控。按 Q/Esc/Ctrl+C 可中止返回，等待程序停稳收尾后再操作。

## 8. 实验数据与 STOP 排查

数据自动保存在工程根目录的 `data/` 中，每次运行使用独立目录。`run_` 后的时间为 UTC，与本地时区显示可能不同。

```text
data/
├── real/
│   ├── continuous/run_*/    # 连续真机
│   ├── discrete/run_*/      # 离散真机
│   └── return/run_*/        # 手动返回
└── simulation/
    ├── continuous/run_*/    # 连续仿真/预演
    └── discrete/run_*/      # 离散仿真/dry-run
```

菜单 5 使用真实 PX6D，但机器人为模拟对象，记录位于 `simulation/discrete/`；可通过 `metadata.json` 中的设备信息区分。

出现 STOP 时，先查看终端给出的实际运行目录，再按下列顺序排查：

| 文件 | 查看内容 |
|---|---|
| `termination.txt`、`termination.json` | 扫描终止原因、原始错误、阶段及后续收尾异常 |
| `summary.json` | 最终状态、原因与汇总诊断 |
| `scan_stop_snapshot.json` | 扫描停止时的位姿及停稳信息 |
| `samples.csv`、`full_log.csv` | 停止前后的力、TCP、状态和诊断数据 |
| `return_status.json` | 启动返回或手动返回的结果、终止阶段及原因 |
| `metadata.json`、`config_snapshot.yaml` | 本次运行的模式、设备与配置来源/快照 |

文件按实际运行阶段生成；启动早期失败可能没有采样 CSV。手动返回优先查看 `return_status.json` 和 `summary.json`，不要求存在扫描的 termination 文件。若日志无法写入，保留终端错误输出。

`STOP_SEARCH_LIMIT` 表示搜索已到达允许范围而未确认接触；`STOP_MOTION_ERROR` 需结合原始错误和实际停稳信息排查。先处理原因再重新启动，不只看退出码，也不要通过修改标定坐标或放宽参数消除报错。

菜单 **19** 选择明确的运行目录查看摘要/回放；菜单 **20** 选择记录及与该实验匹配的 workspace calibration，生成工作空间投影。手动返回记录只展示结果，不作轮廓回放。

`data/` 中的记录、图像和动画不会随 Git 提交。需交付实验结果时单独复制相应完整运行目录；自定义输出也建议放入 `data/`、`outputs/` 或 `results/`。

## 9. 常见问题

| 现象 | 检查与处理 |
|---|---|
| RTDE 无法连接 | 核对机器人 IP、网卡地址与网络连接，确认机器人上电；先运行菜单 4。状态能读但无法控制时，检查 Remote Control、机器人安全状态及其他控制程序是否占用。 |
| PX6D 无法读取 | 检查 USB 连接、`sensor.serial_port`、串口访问权限和端口占用；先单独运行菜单 3。CRC/读取异常时检查线缆及设备状态，不在失败状态下继续扫描。 |
| TCP 不匹配 | 使用示教器和只读 TCP 工具核对真实探针 TCP，检查配置及标定绑定；仅当确认需要更新现场配置时使用菜单 9。 |
| 标定文件缺失或未完成 | 检查配置指向的文件及路径；优先恢复与现场对应的有效文件。无有效文件时按第 4 节由负责人重新采集，不使用随意填写的坐标。 |
| 机器人不在 P0 | 连续启动会显示必要返回路径，确认 `START` 后自动返回；也可先执行 `python return_to_start.py`。返回目标不符时取消确认，核对 P0 和 TCP。 |
| startup/bias 提示机器人移动 | 停止手动操作并等待静止，检查真实线速度、角速度和姿态。连续启动的位置/漂移容差为 1 mm，线速度为 1 mm/s，角速度为 0.01 rad/s；运动后的停车确认另有更严格要求。 |
| 停车超时或无法确认停稳 | 检查实际机器人状态和停止日志；确实无法停稳时按现场规程处置。不要在未确认停止时重新启动运动。 |
| 预演窗口无法打开 | 检查桌面会话和 Matplotlib 后端，或改用菜单 16。离线模拟的停止条件与真机不完全相同，应按对应运行日志判断结果。 |
| Q/Esc 无效 | 确认程序在前台交互终端运行且焦点正确；可使用 Ctrl+C。等待停止收尾，紧急情况使用物理急停。 |
| 找不到数据或无法回放 | 按终端输出的运行目录查找；检查 `metadata.json` 和实际产生的采样文件。不要仅依赖目录时间推断设备模式或选择记录。 |
