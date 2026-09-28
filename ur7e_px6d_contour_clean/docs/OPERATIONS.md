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
3. 显示当前位置、目标来源、目标姿态、抬升/平移/下降三段目标和速度。
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

12 和 18 共用 START 前的 Receive 预检与路径展示。PX6D 已连接，若离开 P0，则在静止处采临时偏置，并用带力监控的同一三段返回执行器回 P0。运动确认只发生一次，Control 不重建。连续启动返回保留 watchdog、采样时效、日志健康和速度/工作空间边界；正式扫描仍使用原 policy。

18 的 Q/Esc/Ctrl+C 均原地停止，不自动返回。12 的 Q 正常停止按 `safe_return.auto_return_after_normal_stop` 返回，Esc/Ctrl+C 和异常不返回。停止是否成功看日志中的新鲜速度确认，不只看 SDK 返回值或进程退出码。

18 连接设备前打印 `Search geometric distance to sandbox boundary`、`Search stopping margin`、`Effective TARGET_SEARCH distance`，单位 mm。它们来自扫描 P0、搜索方向和原始沙箱四边形，不再受历史 100 mm 字段限制。当前已保存数据计算为 766.0762 / 3.7125 / 762.3637 mm；工作空间与斜边垂直净距可以更早限制运动，时间预算仍有效。

到达搜索终点且无接触时，原因是正常 `STOP_SEARCH_LIMIT`。samples 中可看到 STOPPING 的新鲜速度下降及连续静止确认，随后才结束脚本。FIRST_CONTACT 与返回每段也按周期观察停稳。API anomaly 和物理停稳分开记录；超时未停稳为 STOP_MOTION_ERROR。没有增加现场标定步骤，也没有改 YAML 参数。

离线预演窗口需要图形桌面/可交互 Matplotlib backend；无桌面使用菜单 16。测试覆盖预演模型和 Agg 静态绘制，未声称实测桌面拖拽事件或硬件。

## 已执行的迁移验证

- 本轮搜索范围 / STOPPING 修复：完整离线 pytest **113 项通过**。新增覆盖斜四边形射线、130 mm P1、预测步及执行前新鲜位姿复查、超过 100 ms 的逐周期刹停、watchdog、FIRST_CONTACT、API False、有限停车超时、停止期间力/数据故障和返回段停稳。测试构造真实设备接口会直接失败；未执行真机验收。
- 核心 pytest：94 项通过（12.21 秒）；菜单 1 子进程测试入口也已验证。覆盖唯一 Control、确认前零 Control、TCP 不匹配、取消、传感器/写盘故障、真实分支设备替身、API False/异常与物理停稳分离、冻结 timestamp、慢 SDK 读取、持续运动超时、force-rate 阶段、方向和 reacquire。
- 连续/离散仿真及 fake real continuous/discrete/manual return 均验证分类路径与 metadata。
- 连续 0.2 秒模拟按时间预算正常停止；默认场景 12 秒预算在约 2.75 秒触发原有力安全停止。与旧工程对照的 276 条关键力、位姿、速度、状态及停止原因完全一致。离散正方形整圈完成，52 个边界点、4 次边界恢复、72 个探测全部返回锚点。
- 配置/标定及全部 policy 文件按字节校验；旧工程 620 个源码、配置、文档文件内容核验未改变。17 个 CLI 入口 `--help` 与扫描/工作空间离线标定检查通过。
- 没有运行真实 RTDE Control、真实 PX6D 或机器人运动；没有 commit/push/reset/stash/clean。
