# 操作

## 菜单与输出

日常只运行 `python run_project.py`。`run_*` 命名使用 UTC 时间，微秒后缀避免同秒覆盖。

| 菜单 | 操作 | 输出 |
|---|---|---|
| 1 | clean 核心测试，禁用真实设备 | pytest 临时目录，无现场 run |
| 2 | 离散正方形整圈离线验证 | `data/simulation/discrete/run_*` |
| 3 | 只读取 PX6D | 终端，无 run |
| 4 | Receive-only 机器人状态 | 终端，无 run |
| 5 | 真实 PX6D 驱动离散 dry-run，机器人模拟 | `data/simulation/discrete/run_*`；metadata.devices 明确 sensor=real |
| 6 | 离散交互仿真，方/圆/三角 | `data/simulation/discrete/run_*` |
| 7 | 保存 P0/P1 扫描标定 | `scan_calibration.yaml`，无 run |
| 8 | 离线检查扫描标定 | 终端；仅显式 `--output` 时保存预览 |
| 9 | 读取 active TCP，按既有菜单意图写入本地配置 | `config.yaml` 与发生改动时的新备份，无 run |
| 10 | 保存绑定 active TCP 的复位点 | `reset_pose.yaml`，无 run |
| 11 | UR-only 返回 reset，缺少 reset 时返回 P0 | `data/real/return/run_*` |
| 12 | 原离散真机扫描 | `data/real/discrete/run_*` |
| 13 | 只读示教箱体四角标定 | `workspace/config/workspace_calibration.yaml`，无 run |
| 14 | 离线检查箱体四角 | 终端，无 run |
| 15 | 连续交互预演，手动结束 | `data/simulation/continuous/run_*` |
| 16 | 连续有限时长离线模拟 | `data/simulation/continuous/run_*` |
| 17 | 连续已有配置与标定离线检查 | 终端，无 run |
| 18 | 连续真机扫描 | `data/real/continuous/run_*` |
| 19 | 明确选择已有 run 后查看/静态回放 | 选择的 run 内 |
| 20 | 明确选择 run 与标定后箱体坐标投影 | 选择的 run 内 `workspace_view_*` |

新运行路径由模式自动决定，日常无须填写输出目录。连续兼容 CLI 的 `--output` 表示可选 data 根，用于隔离离线验收；后面仍强制追加 mode/strategy/run_*，不会混写。测试会覆盖到 pytest 临时 data 根。复位记录在菜单 19 展示 JSON，不作为轮廓处理。

## 菜单 11 完整流程

1. 优先读取 `reset_pose.yaml`，没有时读取绑定 TCP 的扫描 P0；检查机器人地址和配置 TCP 一致。
2. 创建一个 owner，只连接 Receive，确认机器人无急停/保护停，读取新鲜实际 TCP 速度，持续确认静止。
3. 显示当前位置、目标来源、姿态误差和抬升/对正姿态/平移/下降四段目标及速度。
4. 明确显示：

   ```text
   UR-only return:
   没有 PX6D 外部力监控。
   依赖现场确认及 UR 自身安全系统。
   ```

5. 用户输入 `RETURN` 后再确认位置在等待期间没有改变，才创建唯一 Control，验证 active TCP。
6. 共用返回执行器逐段异步 moveL；每段 request_stop，再用新鲜 actual TCP speed 持续确认停稳。
7. 校验最终位置/姿态，记录 `return_status.json`、summary、metadata 和实际 TCP 样本；结束 Control 脚本并断开。Q/Esc/Ctrl+C 或任何错误中止后不开始下一段。

取消确认不会创建 Control。菜单顶层保留 `OPEN_REAL_RESET`/`OPEN_REAL_SCAN`/`OPEN_CONTINUOUS_SCAN` 防误选确认；子任务的路径确认完成前没有 Control。

## 扫描

12 和 18 共用 START 前的 Receive 预检与路径展示。PX6D 已连接，若偏离 P0 或扫描姿态，则在静止处采临时偏置，并用带力监控的同一四段返回执行器回 P0。START 前打印当前姿态、目标扫描姿态、角度误差和四段路径；需要旋转时明确提示仅在安全高度对正。运动确认只发生一次，Control 不重建。18 默认关闭自定义 watchdog，周期/时效/日志检查只记 warning；UR 自带停止状态、设备错误和全局速度/沙箱边界仍有效。

四段为 `VERTICAL_RETREAT → ALIGN_PROBE_ORIENTATION → MOVE_ABOVE_START → DESCEND_TO_START`。抬升保持当前姿态；严格停稳后才在安全高度旋转到保存的 `fixed_orientation`，然后保持该姿态平移、下降至完整 P0。已经对正则明确打印 `ALIGN_PROBE_ORIENTATION: already aligned; skipped`。菜单 11 共用该执行器，不读取 PX6D；存在 RESET 时使用保存的 RESET 姿态，否则使用 P0。水平返回为 27 mm/s，垂直 18 mm/s，全局最大速度 30 mm/s。

1 mm/s / 0.01 rad/s 只用于运动前的预检与 bias。返回每段、FIRST_CONTACT 和最终 STOP 均需实际线速度 ≤0.1 mm/s、角速度 ≤0.005 rad/s 连续 80 ms；宽松 preflight 通过不等于运动后已经停稳。FIRST_CONTACT 成功转 tracking 时仅打印一次 Fxy、normal n、tangent t、follow hand 与 force_direction_sign。

18 的 Q/Esc/Ctrl+C 均原地停止，不自动返回。12 的 Q 正常停止按 `safe_return.auto_return_after_normal_stop` 返回，Esc/Ctrl+C 和异常不返回。停止是否成功看日志中的新鲜速度确认，不只看 SDK 返回值或进程退出码。

18 连接设备前打印 `Search geometric distance to sandbox boundary`、`Search stopping margin`、`Effective TARGET_SEARCH distance`，单位 mm。它们来自扫描 P0、搜索方向和原始沙箱四边形，不再受历史 100 mm 字段限制。当前已保存数据计算为 766.0762 / 3.7125 / 762.3637 mm；现有边界/停车余量未变。真实 continuous 默认 `max_runtime_sec: null`，搜索和 tracking 均不因运行时间结束；`--duration` 仅限仿真/预演。

到达搜索终点且无接触时，原因是正常 `STOP_SEARCH_LIMIT`。samples 中可看到 STOPPING 的新鲜速度下降及连续静止确认，随后才结束脚本。FIRST_CONTACT 与返回每段也按周期观察停稳。API anomaly 和物理停稳分开记录；超时未停稳为 STOP_MOTION_ERROR。默认关闭自定义 watchdog；原搜索、跟踪速度/力公式及标定保持不变。

真实 continuous 启动预检/bias 允许 1 mm/s、0.01 rad/s，bias 漂移和 P0 启动位置容差为 1 mm；轻微噪声只 warning。返回 processed F/T 超限只 warning，raw 60 N / 5 Nm 仍 STOP。默认 local reacquire 已开启：短暂低力先停车，持续失联后按原圆弧恢复，恢复接触并停稳确认后继续 tracking。reacquire 时间/方向质量只诊断，耗尽已有空间预算才结束。overload stall 时切向为零、继续向外修正；实际 Z/姿态超过旧容差只 warning，超过 5 mm / 5° 仍 STOP。二维速度命令和 FIRST_CONTACT 停稳要求不变。

真实 SEARCH：先检查 raw finite、60 N / 5 Nm 原始上限、设备状态、全局速度和 polygon，再用已建立零偏的 Fxy 判断接触。processed 12 N / 1 Nm、force-rate、方向质量、阶段速度、周期/时效/日志抖动仅记录 software_warnings。零偏建立前不调用 process(raw)；例如恒定 raw=15 N 可以采集并置零。

离线预演窗口需要图形桌面/可交互 Matplotlib backend；无桌面使用菜单 16。测试覆盖预演模型和 Agg 静态绘制，未声称实测桌面拖拽事件或硬件。

## 只读 PX6D 力方向检查

在 clean 工程目录中运行（现有 1～20 菜单编号不变）：

```bash
../.venv312/bin/python tools/check_force_direction.py
```

工具不连接任何 UR 接口、不发送机器人运动，不保存配置或标定。先由操作者将探针置于保存的扫描姿态，保持空载、静止；默认按配置采集 100 个 raw 样本建立仅存在内存中的软件零偏，然后提示从 Base +X、+Y 方向轻推并观察 Fx/Fy 正负。默认每秒显示 5 次 processed Fx/Fy、Fxy、normalized 和 configured n；normalized 未乘 sign，configured n 已乘当前 force_direction_sign。零力时方向显示 `[0, 0]`。Q/Esc/Ctrl+C 退出。

当前 rotation_sensor_to_base 和 force_direction_sign 会先打印出来；数据只是按这套配置换算，并不代表 Base/sign 已经实测正确。需要人工据已知 Base 方向判断，两者都不会自动改写。`--samples 100` 可有限采样；`--bias-samples 0` 跳过新零偏采集，适用于明确要检查未置零结果的场景；`--display-hz 10` 只改变终端刷新率。力方向、坐标变换与 sign 的正确性仍需现场验证。

## 已执行的迁移验证

- 四段返回 / 停稳判据分离 / 力方向入口：完整离线 pytest **199 项通过**（29.06 秒）。新增 20 项覆盖安全高度旋转、已对正跳过、抬升/ALIGN 失败后不继续、0.8 mm/s 与角速度过高时严格停稳等待、startup 0.3 mm/s 通过、水平返回 27 mm/s、手动共用路径、PX6D 的 3/4/5 单位方向和 sign/坐标变换显示。已有首次接触重试测试同时验证 FIRST CONTACT CONFIRMED 只打印一次。未连接真实 UR/PX6D；现场旋转净空、停稳噪声和 Base/sign 尚未验收。
- startup / tracking 限制清理：完整离线 pytest **169 项通过**（26.77 秒）。新增 29 项覆盖启动与返回交接的 0.3 mm/s 噪声、0.5 mm P0 偏差、明显运动拒绝、返回 processed 9 N warning 与 raw 极限、接触丢失后恢复、超过 120 s 的 fake runtime、overload 向外修正、轻微/明显位姿漂移及方向重试。真实驱动构造受测试 fixture 禁止；未连接 UR/PX6D。扫描停车状态转换和 FIRST_CONTACT gate、导纳/切向/法向公式、保存的标定和 reset 文件与本轮基线一致。
- 本轮软件限制简化：完整离线 pytest **140 项通过**。覆盖 35/60 ms 周期抖动、低接触力高 force-rate、processed 超阈值、默认 watchdog 零调用、raw=15 N bias、raw 60/65 N 与 torque 5 Nm、polygon 边界、全局超速、UR 停止状态、PX6D/RTDE 异常、日志积压及 FIRST_CONTACT；同时对照导纳输出与原算法一致。原配置数值、标定文件及停车状态机已检查保持不变。
- 本轮搜索范围 / STOPPING 修复：完整离线 pytest **113 项通过**。新增覆盖斜四边形射线、130 mm P1、预测步及执行前新鲜位姿复查、超过 100 ms 的逐周期刹停、watchdog、FIRST_CONTACT、API False、有限停车超时、停止期间力/数据故障和返回段停稳。测试构造真实设备接口会直接失败；未执行真机验收。
- 核心 pytest：94 项通过（12.21 秒）；菜单 1 子进程测试入口也已验证。覆盖唯一 Control、确认前零 Control、TCP 不匹配、取消、传感器/写盘故障、真实分支设备替身、API False/异常与物理停稳分离、冻结 timestamp、慢 SDK 读取、持续运动超时、force-rate 阶段、方向和 reacquire。
- 连续/离散仿真及 fake real continuous/discrete/manual return 均验证分类路径与 metadata。
- 连续 0.2 秒模拟按时间预算正常停止；默认场景 12 秒预算在约 2.75 秒触发原有力安全停止。与旧工程对照的 276 条关键力、位姿、速度、状态及停止原因完全一致。离散正方形整圈完成，52 个边界点、4 次边界恢复、72 个探测全部返回锚点。
- 配置/标定及全部 policy 文件按字节校验；旧工程 620 个源码、配置、文档文件内容核验未改变。17 个 CLI 入口 `--help` 与扫描/工作空间离线标定检查通过。
- 没有运行真实 RTDE Control、真实 PX6D 或机器人运动；没有 commit/push/reset/stash/clean。
