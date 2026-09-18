# UR7e + PX6D 真机链路审计与空气慢速验证

本次检查和开发没有连接或驱动 UR7e，也没有访问 PX6D 串口。空气模式需现场操作者启动；离线测试结果不能写成真机通过。

## A. 真机链路检查结果

实际入口：

```text
run_project.py 菜单 12 / main.py --sensor real --execute
  → app/main.py
  → PX6DReader.read_wrench()
  → WrenchPreprocessor.process()
  → 当前 policy.rule_policy.RuleBasedPolicy.update()
  → URRTDEController.command_planar_velocity() / stop()
  → speedL([vx, vy, 0, 0, 0, 0]) / speedStop()
```

`simulation/simulator.py` 与 `app/main.py` 导入的是同一个 `RuleBasedPolicy`，没有真机专用旧 policy 或真机绕过局部 tracking/recovery 的实现。`sensor/force_features.py` 中的力方向仅作诊断，运动切线仍由接触 TCP 位置的局部几何生成。

工作区有 `ur7e_px6d_contour_review_20260914_lCRt5T` 审查副本，其 policy 已旧于主工程；它没有被主工程入口导入。请从主工程运行。菜单以项目根目录为工作目录，并清理子进程 `PYTHONPATH`。新空气模式会记录实际 policy 源文件路径和摘要，便于核对版本。

### 按一次真实扫描追踪

| 阶段 | 真机实际行为 | 与修复的关系 |
|---|---|---|
| 初始化 | 校验配置、标定、TCP 身份，连接 PX6D 和 RTDE | policy 构造器来自当前共享模块 |
| 启动回 P0 | 如果不在标定 P0，先执行既有三段返回 | 这是原有真机入口行为，发生在输入 START 之前；空气验证不走此路径 |
| bias | 到 P0 后采集 100 次软件零偏，不调用 PX6D 硬件清零 | 原正式采集函数未检查每个样本的 TCP 静止性 |
| search | 方向由标定 P0→P1 归一化给出 | 没有覆盖后续切线/法线算法 |
| 首次 contact | 处理后的 Fxy 越阈值，立即生成停止命令，HOLD 确认 | 发停止命令不代表机器人已物理静止 |
| return anchor | `ProbeEpisode` 指向原 anchor；XY 误差进容差后 DONE | 仍使用当前统一 episode 实现 |
| local initialization | 三点拟合、排序，沿已执行 anchor 折线对齐到 frontier 对应的 anchor | A 已共享生效 |
| boundary tracking | 沿已观测射线退让，再补偿切向位移；保留明确正向进展 | B 已共享生效；debug 同时用于真机 |
| failure classification | 无接触搜索、前进不足纠偏、重复接触调整前进采样、拟合失败局部重初始化 | C 已共享生效 |
| recovery | 使用原射线和确认框架；本次补齐固定 nominal anchor | D 修补后也由真机/仿真共享 |

### 参数比较：未修改任何现有参数文件

用户所称 `retreat_distance` 对应代码中的 `retract_distance`；`init_offset` 对应 `initialization_lateral_offset`。

| 实际配置键 | 默认正方形 simulation | 上轮小步长正方形回归 | 正式 real robot |
|---|---:|---:|---:|
| `tangent_step` | 8 mm | 3 mm | 3 mm |
| `retract_distance` | 4 mm | 3 mm | 3 mm |
| `initialization_lateral_offset` | 3 mm | 1.5 mm | 1.5 mm |
| `contact_threshold` | 0.65 N | 0.65 N | 1 N |
| `position_tolerance` | 0.001 mm | 0.001 mm | 0.2 mm |

默认正方形的 1 µm 位置容差来自 `scene_square.yaml`；simulation 基础配置实际是 0.35 mm，旋转矩形沿用基础值。真机的 0.2 mm 容差是默认正方形的 200 倍，不能把理想回位当成真机回位证据。

其他现有差异：simulation 为 50 Hz、默认 square EMA 为 0；真机为 100 Hz、EMA 为 0.8。正式搜索/探测/切向/返回速度分别为 9/3/6/6 mm/s，未修改。真机默认不按点数或时间完成整圈，仍由 Q 正常停止。

正式执行对 policy 的唯一运行时覆盖是标定的 `search_direction_xy`。标定 Z/姿态用于 controller 约束；dry-run 的点数/时间/probe sign 覆盖不作用于正式 execute。`main.py --sensor mock` 直接载入默认 simulation 场景，不代表验证了根目录 real 参数。

`main.py --sensor real` **只用真实 PX6D，TCP 仍由 SimulatedController 积分，不连接 UR7e**；请勿把它当成下面的真机空气验证。

### 真实执行风险：只报告，未改正式安全机制

- **延迟尚未实测。** 正式主循环是同步 PX6D 读取、滤波、RTDE 读取、policy、速度命令；PX6D 响应等待可达 50 ms，另有 100 Hz 请求节流。末尾只补足剩余周期，没有严格的超周期或样本年龄拒绝逻辑。
- **观测不同步。** Wrench 不带设备采集时间戳；RTDE pose/speed 分别读取，主循环 policy 时间戳在串口读取之前生成。CSV 中的力和 TCP 位置不能视为严格同步采集。
- **EMA 和停止都有滞后。** EMA=0.8 的低频等效延迟约为 4 个采样周期，在实际达到 100 Hz 时约 40 ms；这是滤波计算值，不是实测系统延迟。
- **stop 不是瞬间生效的物理事件。** `speedL` 的加速度为 0.05 m/s²，停止减速度为 0.2 m/s²。按理想恒减速度估算，9 mm/s 的停止距离约 0.203 mm，3 mm/s 约 0.023 mm，尚未包含滤波、网络和调度延迟。
- **命令周期不是位置执行保证。** 本工作解释器安装的 ur-rtde 1.6.5 方法文档将 `speedL(time)` 说明为函数返回前的时间，`speedStop(a)` 则是减速停止。不能把传入 0.01 s 等同于一个必然精确执行、随后自行静止的 10 ms 位置增量。此检查只读取了本地 SDK 文档，没有创建连接。
- **停止确认不足。** 正式 controller 的常规停止路径会吞掉异常，并设 `_stopped=True`；它没有以实测 TCP 速度持续归零来证明停稳，某些返回值也没有作为失败检查。本次未改变该实现。
- **接触点可能早于最终停止点。** `contact_pose` 记录第一次阈值样本位置，后续惯性前进会改变真正最大压入位置，影响 1 mm 前进验收和 0.6 mm 局部拟合阈值。
- **到位只检查位置。** 正式 `ProbeEpisode` 返回及 anchor transfer 只需一帧 XY 进入容差，没有速度归零和持续稳定要求。D 修复避免 nominal anchor 累积漂移，但不保证每次物理到位零误差。
- **现有现场约束仍需现场确认。** `workspace.enabled=false`，初始搜索最大距离为 1.5 m；传感器到 Base 的旋转配置仍是标注为占位的单位矩阵。当前代码不检查机器人其他连杆、箱体、探针或线缆碰撞。

## B. 本次修改范围

- `policy/rule_policy.py`：补齐固定 recovery anchor。容差内仍使用同一个 nominal anchor 创建下一 episode；超出容差时先用现有受力保护 transfer 回该 anchor，未到位不消耗下一射线角度。
- `app/real_validation.py`、`run_real_validation.py`：独立真机空气慢速验证入口，复用原 policy、RTDE controller 和 PX6D reader。
- `tests/test_real_policy_parity.py`、`tests/test_real_validation.py`：共享代码路径、参数覆盖、固定 anchor 及空气模式的无硬件测试。
- 本文档。

没有修改 simulation、正式 RTDE 停止机制、PX6D 协议、正式安全阈值或现有 YAML 参数文件；没有增加探索策略。

## C. 修复覆盖与空气模式验证边界

检查前，A/B/C 已完整共享。D 之前只是保存了只读 anchor，下一 episode 却仍使用最新返回 pose；本次补齐后，A/B/C/D 均位于共享 policy 中，无需复制到真机专用版本。

空气模式的作用是验证同一 policy 生成的轨迹能否被真实 RTDE 执行并返回指定位置。为了不接触实体目标，测试输入明确分开：

- **真实 TCP pose/speed** 决定实际运动和回位误差。
- **真实 PX6D raw/processed wrench** 始终独立监测意外受力和安全阈值；不能被测试信号替代或屏蔽。
- **合成 policy wrench** 只触发状态转换。倾斜虚拟平面用于 acquisition、三次初始化和两次 tracking，随后一次 tracking 人为无接触，触发原 recovery，并执行两条恢复射线及回位。它不是颗粒/目标模型，也不能验证真实接触阈值。

空气模式保持正式 policy 的几何/阈值参数，使用标定搜索方向，但执行速度在独立模式中限幅到不超过 1 mm/s；当前手动放置的空气 TCP 是模式原点，Z/姿态在此锁定。它不会自动到旧 P0，不会主动下降进箱，也不会在结束时自动返回旧 P0。

模式还会约束原点附近的 XY 范围、检查静止 bias、独立监测未滤波增量力，并在停止后读取速度确认停稳：连续 3 次 TCP 线速度不超过 0.1 mm/s，等待上限 3 秒。若停止缓存与实测运动矛盾，只额外调用一次既有强制停止接口。这些是空气验证入口的执行约束，不能据此认为正式扫描入口也已有停稳确认。局部范围是软件采样检查，不能保证通信中断或制动期间完全没有越界。

短浅角射线可能不足以提供完整设定法向间隙，tracking debug 会输出 `clearance_limited=True`。空气验证只验证实际运动，不能据此推断真实接触时的退让间隙足够。

每个周期记录 current TCP、command target、episode anchor、command direction、请求/执行限幅速度、实测 TCP 速度、实际位移、上一周期命令、真实/测试力及采样耗时。实际位移是两次 TCP 观测差，需结合上一周期命令理解，不能当成新发命令的响应。控制阶段变化及低频摘要打印到终端，完整记录写入日志。

输出目录为 `data/air_validation_时间戳/`：`trajectory.jsonl` / `trajectory.csv` 保存逐周期记录，`probe_episodes.json` 保存探测和回位，`summary.json` 保存结果与六项覆盖检查，另有配置、测试输入和 policy 源码 SHA-256 快照。只有初始化、初始化接续到位、anchor return、切向移动、两次 tracking、两条 recovery 全部覆盖且最终停稳检查通过，结果才会标为 `complete`。空气脚本实际触发的是 `NO_CONTACT` 恢复；其他故障分类的真机物理触发尚未验证。

## D. 真机首次测试建议流程

1. 先运行测试和默认预览：

   ```bash
   cd /home/user-linux/robotics_cs/ur7e_px6d_contour
   source ../.venv312/bin/activate
   unset PYTHONPATH
   PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
   python run_real_validation.py
   ```

   不带 `--execute` 只显示模式和参数，不连接机器人或传感器。

2. 用示教器手动把探针放在颗粒箱外的无遮挡空气位置，保持当前探针 TCP，停止其他运动。确认当前 Z 平面附近至少覆盖模式预览中的 XY 范围，机器人本体和线缆也有足够空间。软件的局部 TCP 范围不能证明现场一定无碰撞。

3. 现场监护下启动空气模式（这条命令会建立真实设备连接，口令确认后会运动）：

   ```bash
   python run_real_validation.py --execute
   ```

   核对打印出的实际空气 TCP、policy 源文件、标定方向、范围和速度；程序完成静止 bias 后，输入 `START_AIR` 才开始轨迹。Q/ESC/Ctrl+C 均停止在当前空气位置。

4. 检查输出 summary、逐周期日志和探测记录：初始化是否先回 frontier anchor，斜向退让后的切向位移是否正向，两个恢复回位是否指向同一个 nominal anchor；同时检查实际回位误差、停稳耗时及真实力变化。若程序报告停稳/位置/力/通信异常，本次应视为失败。

5. 空气验证通过以后，再单独评估真实接触、坐标标定和颗粒介质条件。不要直接用原正式入口测试空气路径，因为它可能在 START 前自动返回已标定、可能位于箱内的 P0。

空气模式不是整圈扫描验收，不会声称验证了真实接触检测、探针形变或颗粒阻力。两个 recovery 完成后，原调度器可能已经排入第三条尚未执行的射线；模式应明确记录该排队探测未执行，不能把它伪记为已返回。

## 本次离线验证结果

2026-09-14，在项目现有 `.venv312` 环境执行全套测试：**141 passed，70.82 秒**。其中新增 5 项真机入口/固定 anchor 测试、10 项空气模式测试；原正方形双方向、平移三角形和旋转矩形整圈回归仍通过。

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH= PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg \
  ../.venv312/bin/python -B -m pytest -q -p no:cacheprovider
```

默认 `run_real_validation.py` 预览退出码为 0，并显示当前主工程 policy 和标定方向。测试使用可注入的设备替身，所有验证均未建立 UR/PX6D 连接；真机空气运行与真实接触测试尚待现场执行。
