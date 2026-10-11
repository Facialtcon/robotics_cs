# 单点多方向接触实验

独立入口 `run_single_point_contact.py` 不调用连续跟踪、丢边恢复、圆弧或射线搜索。
仅复用现有 RTDE/PX6D、空气零偏、Sensor→Base 变换、卡尔曼滤波、日志、制动确认和 HTML 回放。
本轮只做离线验证，没有连接或运动真实机器人。连续跟踪的配置、控制公式、标定和 Reset 不变。

## 运行

在 `ur7e_px6d_contour_clean` 目录：

```bash
# 纯离线演示：生成临时 A/B/C 几何，只运行 A，不写真实标定文件
.venv/bin/python run_single_point_contact.py --simulate
# 离线查看菜单；默认不连接硬件
.venv/bin/python run_single_point_contact.py
# 已有标定的离线模拟，可选择 B、C 等；--speed 单位 m/s
.venv/bin/python run_single_point_contact.py --simulate --group B --speed 0.018
# 真机交互菜单，仅由现场操作人员主动运行
.venv/bin/python run_single_point_contact.py --execute
# 真机选择已有 P0：展示准备路径，经 Enter 确认后自动抬升、调正、采零并下降
.venv/bin/python run_single_point_contact.py --execute --group A --speed 0.018
```

`--config`、`--settings`、`--calibration`、`--output` 可指定独立文件及输出根目录。
每次运行新建 `data/real/single_point/run_*` 或 `data/simulation/single_point/run_*`。
模拟为简单刚性接触力和加速度有限的 TCP 模型，用于软件验证，不能证明真实碰撞安全。

## 沿用原工程的运行参数

`single_point_experiment.yaml` 默认启用 `inherit_project_parameters: true`。
程序加载 `--config` 指定的原工程配置，直接复制有对应项的运行参数；修改原配置后单点入口自动跟随。
机器人 IP、PX6D 串口/采集参数、TCP、坐标变换、卡尔曼滤波、加速度、制动减速度、力/力矩限制本来就直接读取原配置。
速度、停稳保持和返回参数沿用原值；单点工作空间、接近时间及停车确认预算独立配置，不会被继承覆盖：

| 参数 | 当前原值 |
|---|---|
| 默认接近速度 / TCP 上限 | 18 / 30 mm/s |
| 速度预设 | 6 / 12 / 18 mm/s |
| 接触阈值 | 1 N |
| 独立接近时间上限 | 110 s |
| 控制频率 / 周期预算 | 100 Hz / 0.03 s |
| 停稳保持 / 确认预算 | 0.08 / 1 s |
| P0 位置 / Z 容差 | 1 / 1 mm |
| 返回水平 / 垂直速度 | 27 / 18 mm/s |
| 返回抬升距离 | 30 mm，仍需每次核验整条路径 |
| 工作空间 / 自定义看门狗 | 已复用现有 workspace 四角 XY 边界 / 看门狗沿用原关闭值 |

显示最终解析的参数，不连接设备：

```bash
.venv/bin/python run_single_point_contact.py --config config.yaml --show-config
```

接触前/停稳后的记录时间、探针直径、障碍盒等参数也单独保存。
无颗粒背景、不调用连续跟踪或恢复策略、力阈值触发停止和 Enter 确认仍按单点实验流程执行。
运动前继续显示完整路径并要求原有 Enter 确认，不要求填写文字核验记录。

P_ref 的 XY 仅用于确定接近方向，其 Z 作为统一实验高度。机器人沿标定方向持续接近，
不计算到达 P_ref，不使用 P_ref 距离、倍数或候选行程终止运动。唯一正常结束条件是有效 Base Fxy 达到接触阈值。

`single_point_experiment.yaml` 已按操作人员要求复制 `workspace/config/workspace_calibration.yaml` 的
原始 P0→P1→P2→P3 Base XY 四角，启用 `workspace_enabled: true`、`workspace_verified: true`。
`workspace_limits` 保存四角的 XY 外接范围，`workspace_xy_polygon` 保留原始倾斜边界；
路径必须同时通过范围、倾斜边界、探针半径及间隙检查，不能只使用更宽的外接矩形。
工作空间文件已做离线几何一致性检查，标定机器人 IP 和配置 TCP 与当前工程相符；
原文件中的 `active_tcp_verified: false` 保持原样，实际 TCP 仍须在运行时通过原有只读一致性核对。
角点 Z（约 263 mm）及 `average_z` 是标定平面的高度，不是 Z 安全上下限，不复制为运动 Z 范围。
扫描 Z 仍为 `P_ref.z`，准备/返回沿用原有有限抬升、下降和路径确认；共享控制器原有 XYZ 盒检查关闭，
其原有四角 XY 检查独立启用，固定扫描 Z/姿态保护保持不变。默认离线演示生成在该边界内部的模拟点。
未启用、未验证、缺失或无效的边界仍会拒绝真机实验；无需填写文字依据，也不增加 Enter 步骤。
单点工作空间配置不由连续跟踪覆盖，原始 workspace 和 P_ref/P0 标定文件保持不变。
每次发送速度前检查当前实际位置至下一步及制动余量的路径、工作空间和障碍盒。
`max_approach_time_sec` 保持独立的有限运行时间上限（默认 110 秒）；超时、边界或数据异常均报告异常停止，
不会报告接触完成。Remote 检查和原有运动前 Enter 确认保留。

## 实机操作命令

以下命令由操作人员在前台终端执行，不能通过管道提前输入 Enter。

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean

# 标定 P_ref、添加/修改 A/B/C、选择实验组和返回 P0 均在同一个实机菜单
.venv/bin/python run_single_point_contact.py --execute \
  --config config.yaml --settings single_point_experiment.yaml \
  --calibration single_point_calibration.yaml --output data

# 直接选择已有 P0-A；未指定 --speed 时继承原默认接近速度
.venv/bin/python run_single_point_contact.py --execute --group A

# 选择 B 或 C，明确指定与原程序相同的 18 mm/s
.venv/bin/python run_single_point_contact.py --execute --group B --speed 0.018
.venv/bin/python run_single_point_contact.py --execute --group C --speed 0.018

# 本次独立实验也可以显式选择较低速度；不会改写原 config.yaml
.venv/bin/python run_single_point_contact.py --execute --group A --speed 0.0005
```

首次先在菜单选 **1** 保存 P_ref，再反复选 **2** 添加 A/B/C；选 **5** 启动实验。
标定时在示教器选择 **Local**，人工移动 TCP；自动实验和返回 P0 前由操作人员切换 **Remote**。
不要求提前手动调到选定 P0 的高度和姿态。按提示核验并确认准备路径、上方空气零偏、速度和开始接近。
运行时 Q/Esc/Ctrl+C 停止。
返回指定 P0：重新进入上述实机菜单，选 **6**，选组、检查展示路径，再按 Enter；选择组本身不运动，无需填写文字依据。
准备路径等待确认时只读取机器人状态，不运行力限值检查。准备完成后，等待最终 Enter 的静止记录
只采集力数据，不用接触阈值、实验力矩限值或 P0 无接触判定中断等待；最终 Enter 后重新读取数据，
验证起点无接触并启用正式实验判定，随后开始接近。日志记录 `experiment_checks_active` 及其启用时间。
自动准备和菜单 6 返回 P0 都属于真实运动，各自的路径、Remote、力/力矩硬保护与断流制动仍然有效；
返回 P0 不使用 1 N 接触阈值判断实验完成。

## 多组 P0 标定

1. 共享参数继承原 `config.yaml`；当前默认速度 18 mm/s，允许输入范围 0.1–30 mm/s。
   真机实验使用独立配置且现有标定的 Base 四角 XY 工作空间；未启用或未验证时拒绝启动。运行期间仍检查时间和已配置障碍盒。
2. 运行真机菜单，选择 **1**。在示教器选择 Local，人工将 TCP 移到几何参考位置并停稳，允许参考姿态不竖直。
   按 Enter 后，程序依次显示连接 RTDE Receive、读取实际 TCP、停稳检查、只读 TCP 配置核对和读取的坐标。
   随后保存 P_ref、完整参考姿态、时间、机器人及 TCP 配置，重新读取 YAML 并比较完整内容及原始精度。
   只有回读校验通过才显示“标定成功：P_ref”和绝对保存路径；失败会显示具体原因，取消会明确提示，然后返回菜单。
   不激活 Control，不发送运动命令，不要求为了只读标定切换 Remote。
3. 手动将 TCP 移到第一组 P0 的 XY 位置，Z 和姿态可以任意不同。选 **2**，输入 **A**，
   按 Enter 读取完整原始 TCP 位姿；实验起点只使用其中的 XY。软件计算 `normalize(P_ref.xy-P0.xy)`，
   同时显示最终起点 `[P0.x, P0.y, P_ref.z, downward_orientation]`。
   坐标、距离、接近方向和 TCP 配置检查通过后直接保存，并回读验证，显示“标定成功”。
   标定不要求填写路径说明或额外确认；实际运动前仍会显示路径、执行约束检查并要求 Enter 确认。
4. 按同样步骤新增 **B、C、D**。菜单 **3** 查看所有组；**4** 逐组重新采集或确认删除。
   P_ref 已有 P0 时，菜单 **1** 先让操作人员选择保留所有 P0 的 XY、清空所有 P0 或取消。
   选择后才进入只读采集，并显示新旧参考坐标、每组方向变化或即将清空的名称，最后再次用 Enter 确认保存。
   保留 P0 时原始位姿和历史元数据完整保留，方向按新参考点重算；
   新参考 Z 和参考姿态用于生成统一高度与竖直姿态。采集或确认取消不会修改文件。

独立参数 `min_approach_calibration_distance_m` 默认 **0.005 m（5 mm）**，可按实验需求调整，
不继承连续跟踪参数。仅检查 XY 距离，原始 P0 的 Z 和姿态仍可任意不同。
新增或重新采集的 P0 距离不足时，显示实际距离和最小距离（mm），拒绝保存并提示移远后重采。
旧文件中的近距离 P0 仍可查看，表格显示“距离过近，需要重标定”，不显示虚假的方向，禁止自动实验及返回。
重标定参考点后变得过近的 P0 保留原始记录，方向置为无效，必须重新采集。
所有保存先在内存中构建数据，备份原文件后通过临时文件替换，并回读比较完整精度；保存或回读失败会恢复原文件。

P0 使用九步标定提示、保存和回读校验，新增组会保留已有参考点和其他组的完整数据。
旧文件中的 `path_review` 字段仅作为历史元数据保留，不用于判断是否允许实验；新组不再生成此字段。
全局文字核验配置已移除；旧配置中的同名字段不会阻止实验。
菜单 **3** 显示 P_ref XYZ 和 P0 数量；表格仅包含名称、X、Y、XY 距离、相对 Base +X 的接近方向角和标定状态。
正常界面的坐标统一使用 **mm，三位小数**；姿态旋转向量使用 rad，方向角使用度。
内部几何、YAML、日志和快照继续使用 SI 单位和完整精度。查看完整原始数据需主动使用离线调试命令：

```bash
.venv/bin/python run_single_point_contact.py --show-calibration-json \
  --calibration single_point_calibration.yaml
```

此命令只读取文件，不连接任何机器人接口。

## Local / Remote 检查

标定通过 RTDE Receive（30004）和 Secondary 只读端口（30012）查询状态/TCP。
UR 官方说明这些状态监控接口可以在 Local 和 Remote 模式使用。
[UR 官方接口说明](https://www.universal-robots.com/articles/ur/interface-communication/remote-control-via-tcpip/)

程序另外通过 Dashboard Server（29999）只发送 `is in remote control` 查询。
`true` 显示“Remote（远程控制）”；`false` 显示“未处于Remote，可能为Local模式或未启用Remote功能”，不会断言一定是 Local。
未知回复、超时、连接失败或不支持该命令都明确显示“状态：未知”及具体原因。
此命令从 e-Series PolyScope 5.6.0 起支持，与 Manual/Automatic 操作模式不是同一个查询。
[UR 官方 Dashboard 命令表](https://www.universal-robots.com/manuals/EN/HTML/SW5_23/Content/prod-dashboard/Dashboard_table.htm)

菜单 **7** 可随时只读查询实际控制模式，不要求 `--execute`，不读取标定文件，也不连接 PX6D 或 RTDE Control。
这是离线菜单中由操作人员明确选择后发起的网络查询。其他离线操作保持不连接设备。
菜单 **1、2** 进入实际采集流程时，在人工移动确认和连接 RTDE 前显示模式查询结果；
只读标定在模式未知时仍可继续读取，不会强制切换 Remote。
自动实验及返回先显示当前模式，之后继续原有路径检查。
自动准备和菜单 **6** 返回则在路径 Enter 确认后、创建 Control 前重新查询模式：
只有明确确认 Remote 才允许后续自动运动；Local 或查询失败时报告原因并拒绝自动运动，操作人员处理后重新核验路径。
程序不发送任何模式切换、启动、上电或解锁命令；模式查询不会代替已有的路径核验和 Enter 确认。

`single_point_calibration.yaml` 与 `scan_calibration.yaml` 完全独立。
P_ref 保存完整 `reference_tcp_pose`，`P_ref` XYZ 字段保持兼容。参考 XY 仅确定接近方向，
参考 Z 是所有组统一扫描高度。参考姿态通过 `downward_probe_orientation()` 推导统一竖直姿态，
把 `calibration.probe_axis_tcp` 指定的物理轴对准 Base -Z，不修改 TCP 偏置。
所有组使用同一个最终姿态；原始 P0 Z 和姿态仅留作标定记录，不参与实验起点计算。
校验会拒绝 XY 重合、非法数字、错误方向及 TCP 不一致，允许原始 Z 和姿态不同。
探针直径 **3 mm**；P_ref 是几何参考，不保证不同方向都接触同一个物理点。
操作人员必须检查路径是否先碰到目标的其他部分。`forbidden_boxes` 用于夹具或其他障碍，
不要把预期接触的目标整体设为禁入盒；软件检查包含探针半径和间隙，但不能替代整套工具的现场检查。

## 选择方向和实验流程

菜单 **5** 选择 P0-A/B/C 等，先显示组名、方向、接近速度、阈值和正常停止条件。选择组本身不运动。
程序只读实际 TCP 位姿并核对 TCP/传感器，使用现有安全返回逻辑生成完整准备路径：

1. 展示实际起点、抬升高度、旋转、高位 XY 转移和下降路径；现场核验后按新的 Enter 授权。
2. 在当前 XY 安全抬升，停稳后自动调正探针，再移动至目标 P0 的 XY 上方。
3. 在高于 `P_ref.z` 的安全位置停稳，操作人员确认完全无接触后采集空气零偏。
   准备运动仅使用现有绝对硬保护，不使用低力/力矩阈值，不在目标接触状态采零。
4. 下降到 `P_ref.z`，确认实际运动完成和停稳；P0 无接触判定不阻止此阶段准备完成。
   准备失败或路径未确认仍拒绝开始接近。
5. 输入本次速度（交互单位 **mm/s**，`--speed` 单位 **m/s**），静止记录至少 1 秒，
   按新的 Enter 授权正式实验后，才进行原有 P0 无接触和实验力保护检查，然后开始单点接近。

原有抬升距离必须经过现场核验，不能假设 30 mm 一定安全；确认期间实际位姿改变需重新规划。
实验沿 Base XY 运动，Z 始终为 `P_ref.z`；不会把 P_ref 的 XY 当作运动终点。

流程为 READY → APPROACH → CONTACT_STOP → SETTLING → FINISHED。
`app/single_point_runtime.py` 的 `SinglePointTrial.approach()` 在第一次有效的滤波 Base Fxy
达到 `contact_threshold_N` 时调用 `_request_stop()`，在任何事件/样本日志写入之前请求制动。
即使未到 P_ref 也正常触发；经过 P_ref 不会停止或判成功。
独立运行时间上限触发时报告 `aborted` / `EXPERIMENT_TIME_LIMIT`；所有异常保护触发均为异常停止。

持续检查固定 Z/姿态、路径、实际速度、真机必需的工作空间、原有原始及处理后力/力矩硬限制。
PX6D 失效、非有限数据、请求超时、过期观测、RTDE 异常、人工 Q/Esc/Ctrl+C 都请求停止。
共享控制器的发送前回调在额外 RTDE 读取之后再检查力数据有效期、独立运行时间和实际路径边界。
自定义看门狗的开关和频率沿用原配置；启用时约束主机失联，保留机器人自身保护。停止指令和实际停车分开记录，
制动后保持读取 TCP 和有效力数据，复用控制器的新鲜 RTDE 包及低速保持确认。
力传感器失效时制动阶段继续记录 TCP，力列记为缺失，绝不重用旧力冒充新样本。
`stop_timeout_sec`（默认 1 秒）从制动请求时间起，仅用于制动和新鲜 RTDE 实际速度的停稳确认。
确认后再启动独立 `post_stop_record_sec`（默认 0.5 秒）记录窗口；最后一个样本允许至多一个 `max_cycle_sec` 周期的有界完成余量。
因此确认停稳耗时 0.7 秒，再记录 0.5 秒可以正常完成，不占用同一个 1 秒期限。
停稳后再次运动、RTDE 反馈失效、SDK 制动异常或记录未完成都会报告失败，保留已确认过停稳的证据。
SDK 返回成功或指令速度为零均不能代替实际停稳反馈。超过独立运行时间时，即使正常停稳，也显示“异常停止：超过最大允许实验运行时间，未完成接触实验。”

## 返回 P0

菜单 **6** 复用 `SafeReturnExecutor` 和 `return_trajectory`。
返回目标同样是由 P0 XY、P_ref Z 和统一竖直姿态生成的六维位姿。
默认抬升距离沿用原 `safe_return.return_lift_distance`，当前为 30 mm；数值不作为路径已经安全的证明。
每次仍需现场核验，展示实际起点、抬升、姿态对齐、高位转移、下降的完整路径，
校验工作空间和障碍盒，保留 Remote 检查及原有 Enter 运动确认，无需填写文字。
路径不可用、未确认 Remote、取消确认或机器人在确认期间位姿改变都会拒绝返回。
路径及确认状态保存到本次独立运行目录的 `return_plan.json`。没有改变原 Reset 或已有返回配置文件。

菜单 5 的自动准备及菜单 6 返回 P0 不使用正式实验的 12 N / 1 N·m 阈值，
也不使用原返回的 8 N / 0.7 N·m 阈值拦截运动；这些低阈值不再作为准备阶段的终止条件。
先执行所展示的有限 Base +Z 竖直脱离段，锁定 XY 和原姿态，实际抬升停稳后再调正和横移。
整个准备/返回阶段仍保留原有硬限制（60 N / 5 N·m），检查原始值及含力臂的 Base 力/力矩。
停稳、抬升高度、分段时间、工作空间、障碍物、通信和人工取消保护保持生效。
空气零偏仍在高位、停稳并经操作人员确认无接触后采集，不在接触位置采零。
P0 的 0.2 N 无接触判定推迟到最终 Enter 授权正式实验后，由原实验启动检查执行，
不阻止准备完成、静止记录和速度选择。正式实验的接触、力/力矩和传感器保护仍使用原逻辑。
全局配置中的数值及共享返回/制动实现保持原样，仅单点准备监控器选择现有硬保护。
返回预检显示原始实测值及含力臂的 Base 力矩，失败时显示阶段、实测值和限值。
`return_force_preflight.json` 与 `return_force_status.json` 保存相应数据及脱离停稳时间。
自动准备对应的 `startup_force_preflight.json` 与 `startup_force_status.json` 同样保存实测载荷和阶段，
预检失败时也保留实测信息，避免只有笼统的 `return torque limit exceeded`。

## 数据和回放

- `samples.csv`、`full_log.csv`：实际六维位姿/速度、PX6D 原始六维力/力矩、Base 原始与补偿后六维力/力矩、
  滤波力、Fxy、状态、方向、P0/P_ref、阈值、名义速度和事件时间。无颗粒背景采集。
- `tcp_commands.csv`：实际发送的六维速度参数、命令序号、接受结果、发送/返回时间，包括零速度制动指令。
  样本中的 commanded_tcp_* 只是最近已发送命令引用，不能按 CSV 行号视为同步。
- `single_point_result.json` 另外记录阈值触发时的实际位姿/速度、原始及 Base 力/力矩、制动请求时间、
  `stop_report` 中的制动及模式退出 SDK 返回值、实际停稳时间及六维速度、停稳后记录起止和截止时间。
  `stop_failure_kind` 区分 SDK 制动失败、未确认停稳、停稳被撤销、记录未完成；`timeout_phase` 明确标识发生在哪个计时阶段。
  `contact_triggered` 独立记录是否已经触发正常接触；`safety_events` 保存接近、制动或停稳后发生的
  力/力矩超限样本、阶段、时间、限值和峰值。有效的超限反馈不会被误报为传感器失效，
  制动后继续读取并记录新鲜力数据和实际 TCP；真实断流、过期或非有限数据才标记为反馈不可用。
  如果已接触并停稳、记录也完整，但发生安全超限，实验仍报告异常并明确显示接触已触发，不能视作无安全异常的正常完成。
  CSV 同步记录阶段、失败类别、超时阶段及确认/记录时间；当前记录配置快照包含独立工作空间、现场验证状态及运行时间上限。
- `actual_tcp_timestamp` 是主机完成 TCP 读取的时间；`rtde_device_timestamp_sec` 是独立设备包时间。
  PX6D 无已验证的设备采集时钟，保留 `force_read_start_sec`/`force_read_end_sec` 主机请求区间，
  不声称与 TCP 或指令硬件同步。
- `single_point_result.json`：阈值前最后一个未达到阈值的有效实际速度、阈值观测时实际速度及其采样时间、
  阈值力样本主机时间、检测时间、制动请求时间、停稳确认时间/位置和滤波前后峰值 Fxy。
  阈值是滤波测量的 crossing，真实物理接触可能更早。
- `config_snapshot.yaml`、`single_point_calibration_snapshot.json`、`single_point_experiment_snapshot.json`、
  `air_zero.json`（真机）：完整有效配置、标定、空气零偏与变换参考信息。
  `startup_plan.json` 保存已确认的完整准备路径，`startup_preparation.json` 保存准备完成后的实际位姿和探针倾角；
  `air_zero.json` 另记录上方采零位置，便于确认采零发生在下降之前。
  六维力矩的参考点由原预处理配置决定，需结合 `air_zero.json` 的变换状态解读。
- `实验回放.html`：实际 XY 轨迹、Base Fx/Fy/Fz 和 Tx/Ty/Tz（图例明确 N/Nm，均可单独开关）、
  Fxy、实际/指令 XY 速度、阈值、触发与实际停车事件。没有目标轮廓。
  使用第一阶段事件选项；查看阈值和停止标记可选择“全部显示”或勾选首次接触／停止事件。

## 目标局部直边与接触力方向测量

使用原实机菜单命令，新增菜单 **8. 标定目标直边**：

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean
.venv/bin/python run_single_point_contact.py --execute
```

在示教器选择 Local，手动将 TCP 放到同一条实际直边的 A 点，停稳后按 Enter 读取；
再以一致的探针接触方式移动到 B 点，停稳后按 Enter 读取。两次均只读实际 TCP，
复用只读停稳和 TCP 配置检查，不创建 RTDE Control，不发送运动命令，不连接 PX6D。
Z 与姿态完整记录，但仅 Base XY 参与方向计算。最小 XY 间距默认 **5 mm**，
可通过 `--min-edge-calibration-distance-m 0.01` 调整为 10 mm；距离过近会说明实际距离并拒绝保存。
探针直径 3 mm，示教的 TCP 中心不一定是物体接触点；不进行半径补偿。

标定保存到独立的 `target_edge_calibration.yaml`，包含 A/B 完整实际位姿、采集及标定时间、
机器人 IP、TCP 配置、Base XY 切向与法向。可通过 `--target-edge-calibration` 指定独立文件。
重复菜单 8 即可重新标定；新点全部有效后备份旧文件，临时文件原子替换，回读确认完整精度。
采集取消、失败或保存/回读失败保留旧文件。不会修改 P_ref/P0 标定。

菜单 5 继续使用原实验流程。每次运行在运动前固定目标直边快照；未标定、文件无效或
机器人/TCP 身份不一致，仅跳过夹角分析，仍按原控制逻辑运行。
首次阈值判定请求制动后，在内存中保存该实际力帧的滤波前 Base 力和判定使用的滤波后 Base 力、
原始 Sensor 力、PX6D 主机请求区间及该次实际 TCP 观测，不插入制动前文件写入。
实验结束后由 `app/contact_force_direction.py` 计算：

```text
t = (B.xy - A.xy) / norm(B.xy - A.xy)
theta = degrees(acos(clip(abs(dot(F, t)) / norm(F), 0, 1)))
perpendicular_error = 90 - theta
```

F 仅使用实际触发帧的滤波后 Base Fx/Fy；同时保存同帧滤波前 Base Fx/Fy 和对应夹角用于比较。
不使用停车后的力、多帧平均或 Sensor XY 替代。数据无效/模长过小则明确报告无法计算。
这里的瞬时帧指阈值判断实际使用的测量帧，不宣称精确捕捉了连续物理时间中的首次接触。
合力可能含摩擦和其他干扰，结果不预设垂直，也不改变测量方向或原有滤波参数。

运行目录增加 `target_edge_calibration_snapshot.json` 与 `contact_force_direction_analysis.json`；
后者包含完整标定、原始触发证据、实际位姿/时间、滤波前后 Base XY 力、夹角与垂直偏差。
`single_point_result.json` 的 `force_at_threshold` 同样保存触发帧滤波前 Base 力及主机请求区间。
HTML 回放在原 Base XY 等比例轨迹图上叠加橙色 A/B 直边和首次阈值触发位置的深紫色力箭头，
箭头长度归一化，仅示意真实方向；标注不表示真实力大小。
新增三个默认勾选的开关：显示标定边界、显示首次接触力方向、显示夹角分析。
分析区显示边界方向、力方向、夹角、理想 90°、偏差、Fxy 及滤波前方向对比。
旧记录缺少分析时照常回放，不使用当前新标定为旧实验补造结果。

## 自动化验证

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q
```

禁用自动加载是因为此环境的 ROS pytest 插件缺少依赖；项目自身测试全部照常收集。
项目 `tests/conftest.py` 禁止实例化真实 RTDE、串口或网络连接。
