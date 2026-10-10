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
.venv/bin/python run_single_point_contact.py --simulate --group B --speed 0.0005
# 真机交互菜单，仅由现场操作人员主动运行
.venv/bin/python run_single_point_contact.py --execute
# 真机选择已有 P0，不自动移到该 P0，仍需多次新的 Enter 确认
.venv/bin/python run_single_point_contact.py --execute --group A --speed 0.0005
```

`--config`、`--settings`、`--calibration`、`--output` 可指定独立文件及输出根目录。
每次运行新建 `data/real/single_point/run_*` 或 `data/simulation/single_point/run_*`。
模拟为简单刚性接触力和加速度有限的 TCP 模型，用于软件验证，不能证明真实碰撞安全。

## 多组 P0 标定

1. 在 `single_point_experiment.yaml` 填写现场检查后的独立 Base XYZ `workspace_limits`。
   填写 `site_validation_note`，记录低速、阈值、制动距离、工作空间和工具扫掠检查。
   配置中初始值只是待验证的低速起点：默认 0.5 mm/s，范围 0.1–2 mm/s；不使用 18 mm/s。
2. 运行真机菜单，选择 **1**。手动将 TCP 移到几何参考位置，探针竖直向下、停稳，
   按 Enter 后仅通过 Receive 和只读 TCP 查询保存 P_ref、完整参考姿态、时间、机器人及 TCP 配置。
   不激活 Control，不自动运动到 P_ref。
3. 手动将 TCP 移到第一组空气中的起点，保持与参考相同的 Z 和姿态。选 **2**，输入 **A**，
   按 Enter 读取完整起始 TCP 位姿。软件计算 `normalize(P_ref.xy-P0.xy)`。
   检查直到最大搜索距离的完整路径，填写现场核验依据，再用 Enter 确认路径核验。
   留空可以保存标定，但禁止该组真机实验。
4. 按同样步骤新增 **B、C、D**。菜单 **3** 查看所有组；**4** 逐组重新采集或确认删除。
   P_ref 有已存 P0 时禁止改写；确需重新标定，先逐组删除，再采集新参考和各组 P0。

`single_point_calibration.yaml` 与 `scan_calibration.yaml` 完全独立。
校验会拒绝 XY 重合、非法数字、错误方向、Z/姿态不一致、非向下探针及 TCP 不一致。
探针直径 **3 mm**；P_ref 是几何参考，不保证不同方向都接触同一个物理点。
操作人员必须检查路径是否先碰到目标的其他部分。`forbidden_boxes` 用于夹具或其他障碍，
不要把预期接触的目标整体设为禁入盒；软件检查包含探针半径和间隙，但不能替代整套工具的现场检查。

## 选择方向和实验流程

菜单 **5** 选择 P0-A/B/C 等，只选择和显示参数，不自动移动。
人工将机器人放在选定的 P0 并停稳，确认 TCP 和传感器正常、探针未接触目标，再采集现有空气零偏。
输入本次速度（交互输入单位 **mm/s**，命令行 `--speed` 单位 **m/s**），检查有效范围。
静止记录至少 1 秒，再按新的 Enter 开始。

流程为 READY → APPROACH → CONTACT_STOP → SETTLING → FINISHED。
`app/single_point_runtime.py` 的 `SinglePointTrial.approach()` 在第一次有效的滤波 Base Fxy
达到 `contact_threshold_N` 时调用 `_request_stop()`，在任何事件/样本日志写入之前请求制动。
即使未到 P_ref 也正常触发；经过 P_ref 不会停止或判成功。
达到距离或时间上限但没有力阈值时报告 `no_contact`，不报告成功接触。

持续检查固定 Z/姿态、路径、实际速度、独立工作空间、原有原始及处理后力/力矩硬限制。
PX6D 失效、非有限数据、请求超时、过期观测、RTDE 异常、人工 Q/Esc/Ctrl+C 都请求停止。
共享控制器的发送前回调在额外 RTDE 读取之后再检查力数据有效期和搜索距离。
机器人看门狗约束主机失联；保留机器人自身保护。停止指令和实际停车分开记录，
制动后保持读取 TCP 和有效力数据，复用控制器的新鲜 RTDE 包及低速保持确认。
力传感器失效时制动阶段继续记录 TCP，力列记为缺失，绝不重用旧力冒充新样本。
停稳后继续记录 0.5 秒；停稳被撤销或未确认不会报告成功。

## 返回 P0

菜单 **6** 复用 `SafeReturnExecutor` 和 `return_trajectory`。
默认 `return_lift_distance_m: null`，**拒绝自动返回**；不会假设原有 30 mm 抬升安全。
现场核验后才能设置所需抬升距离。每次展示实际起点、抬升、姿态对齐、高位转移、下降的完整路径，
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
