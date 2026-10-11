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
现在速度、搜索预算、停稳、时间限制、工作空间开关和返回参数也沿用原值：

| 参数 | 当前原值 |
|---|---|
| 默认接近速度 / TCP 上限 | 18 / 30 mm/s |
| 速度预设 | 6 / 12 / 18 mm/s |
| 接触阈值 | 1 N |
| 最大搜索距离 / 接近时间 | 100 mm / 110 s |
| 控制频率 / 周期预算 | 100 Hz / 0.03 s |
| 停稳保持 / 确认预算 | 0.08 / 1 s |
| P0 位置 / Z 容差 | 1 / 1 mm |
| 返回水平 / 垂直速度 | 27 / 18 mm/s |
| 返回抬升距离 | 30 mm，仍需每次核验整条路径 |
| 工作空间 / 自定义看门狗 | 与原配置相同，当前均关闭 |

显示最终解析的参数，不连接设备：

```bash
.venv/bin/python run_single_point_contact.py --config config.yaml --show-config
```

单点实验新增且原程序没有对应项的参数仍单独保存，如接触前/停稳后的记录时间、探针直径、障碍盒和现场核验记录。
无颗粒背景、不调用连续跟踪或恢复策略、力阈值触发停止和 Enter 确认仍按单点实验流程执行。
复制原值不代表这些值已经针对固定刚性物体验证；真机启动仍检查现场核验记录。

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
不要求提前手动调到选定 P0 的高度和姿态。按提示核验并确认准备路径、上方空气零偏、速度和开始接近。
运行时 Q/Esc/Ctrl+C 停止。
返回指定 P0：重新进入上述实机菜单，选 **6**，选组、核验展示路径、填写依据，再按 Enter；选择组本身不运动。

## 多组 P0 标定

1. 在 `single_point_experiment.yaml` 填写 `site_validation_note`，记录本次速度、阈值、制动距离和路径检查。
   共享参数继承原 `config.yaml`；当前默认速度 18 mm/s，允许输入范围 0.1–30 mm/s。
   若原配置启用了工作空间，沿用其 Base XYZ 边界；原配置关闭时也保持关闭，但仍检查距离、时间和已配置障碍盒。
2. 运行真机菜单，选择 **1**。手动将 TCP 移到几何参考位置并停稳，允许参考姿态不竖直。
   按 Enter 后仅通过 Receive 和只读 TCP 查询保存 P_ref、完整参考姿态、时间、机器人及 TCP 配置。
   不激活 Control，不自动运动到 P_ref。
3. 手动将 TCP 移到第一组 P0 的 XY 位置，Z 和姿态可以任意不同。选 **2**，输入 **A**，
   按 Enter 读取完整原始 TCP 位姿；实验起点只使用其中的 XY。软件计算 `normalize(P_ref.xy-P0.xy)`，
   同时显示最终起点 `[P0.x, P0.y, P_ref.z, downward_orientation]`。
   在统一参考高度和竖直姿态下检查直到最大搜索距离的完整路径，填写现场核验依据，再用 Enter 确认路径核验。
   留空可以保存标定，但禁止该组真机实验。
4. 按同样步骤新增 **B、C、D**。菜单 **3** 查看所有组；**4** 逐组重新采集或确认删除。
   P_ref 有已存 P0 时禁止改写；确需重新标定，先逐组删除，再采集新参考和各组 P0。

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

菜单 **5** 选择 P0-A/B/C 等，先显示原始标定、生成的实验起点和方向。选择组本身不运动。
程序只读实际 TCP 位姿并核对 TCP/传感器，使用现有安全返回逻辑生成完整准备路径：

1. 展示实际起点、抬升高度、旋转、高位 XY 转移和下降路径；现场核验后按新的 Enter 授权。
2. 在当前 XY 安全抬升，停稳后自动调正探针，再移动至目标 P0 的 XY 上方。
3. 在高于 `P_ref.z` 的安全位置停稳，操作人员确认完全无接触后采集空气零偏。
   抬升和调姿前只执行 PX6D 原始力保护，不在目标接触状态采零。
4. 使用空气补偿后的力监控下降到 `P_ref.z`，检查停稳、竖直和 P0 无接触。
   准备失败、路径未确认或 P0 有接触都会拒绝开始接近。
5. 输入本次速度（交互单位 **mm/s**，`--speed` 单位 **m/s**），静止记录至少 1 秒，
   按新的 Enter 开始单点接近。

原有抬升距离必须经过现场核验，不能假设 30 mm 一定安全；确认期间实际位姿改变需重新规划。
实验沿 Base XY 运动，Z 始终为 `P_ref.z`；不会把 P_ref 的 XY 当作运动终点。

流程为 READY → APPROACH → CONTACT_STOP → SETTLING → FINISHED。
`app/single_point_runtime.py` 的 `SinglePointTrial.approach()` 在第一次有效的滤波 Base Fxy
达到 `contact_threshold_N` 时调用 `_request_stop()`，在任何事件/样本日志写入之前请求制动。
即使未到 P_ref 也正常触发；经过 P_ref 不会停止或判成功。
达到距离或时间上限但没有力阈值时报告 `no_contact`，不报告成功接触。

持续检查固定 Z/姿态、路径、实际速度、启用时的工作空间、原有原始及处理后力/力矩硬限制。
PX6D 失效、非有限数据、请求超时、过期观测、RTDE 异常、人工 Q/Esc/Ctrl+C 都请求停止。
共享控制器的发送前回调在额外 RTDE 读取之后再检查力数据有效期和搜索距离。
自定义看门狗的开关和频率沿用原配置；启用时约束主机失联，保留机器人自身保护。停止指令和实际停车分开记录，
制动后保持读取 TCP 和有效力数据，复用控制器的新鲜 RTDE 包及低速保持确认。
力传感器失效时制动阶段继续记录 TCP，力列记为缺失，绝不重用旧力冒充新样本。
停稳后继续记录 0.5 秒；停稳被撤销或未确认不会报告成功。

## 返回 P0

菜单 **6** 复用 `SafeReturnExecutor` 和 `return_trajectory`。
返回目标同样是由 P0 XY、P_ref Z 和统一竖直姿态生成的六维位姿。
默认抬升距离沿用原 `safe_return.return_lift_distance`，当前为 30 mm；数值不作为路径已经安全的证明。
每次仍需现场核验，展示实际起点、抬升、姿态对齐、高位转移、下降的完整路径，
校验工作空间和障碍盒，再要求填写当前路径的现场核验依据及新的 Enter 运动确认。
依据为空、路径不可用或机器人在确认期间位姿改变都会拒绝返回，应手动移动到安全位置。
路径依据保存到本次独立运行目录的 `return_plan.json`。没有改变原 Reset 或已有返回配置文件。

## 数据和回放

- `samples.csv`、`full_log.csv`：实际六维位姿/速度、PX6D 原始六维力/力矩、Base 原始与补偿后六维力/力矩、
  滤波力、Fxy、状态、方向、P0/P_ref、阈值、名义速度和事件时间。无颗粒背景采集。
- `tcp_commands.csv`：实际发送的六维速度参数、命令序号、接受结果、发送/返回时间，包括零速度制动指令。
  样本中的 commanded_tcp_* 只是最近已发送命令引用，不能按 CSV 行号视为同步。
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

## 自动化验证

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q
```

禁用自动加载是因为此环境的 ROS pytest 插件缺少依赖；项目自身测试全部照常收集。
项目 `tests/conftest.py` 禁止实例化真实 RTDE、串口或网络连接。
