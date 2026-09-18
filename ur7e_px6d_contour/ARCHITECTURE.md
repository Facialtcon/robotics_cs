# 工程架构（2026-09-11：局部接触几何版本）

普通 Python + ur_rtde，不使用 ROS/MoveIt，没有第二套模拟策略。
操作入口：[操作文档.md](操作文档.md)；算法教学：[POLICY_EXPLANATION.md](POLICY_EXPLANATION.md)。

## 目录与职责

```text
app/main.py                      系统装配、主循环、操作者停止
core/models.py                   Wrench / RobotState / PolicyCommand / BoundaryPoint
robot/rtde_controller.py          RTDE 通信、运动、停止、TCP/Z/姿态检查
sensor/px6d_reader.py             PX6D 串口协议
sensor/force_preprocess.py        零偏、滤波、重力、坐标变换、颗粒基线
sensor/force_features.py          力模长/角度等诊断量，不做边界定位
policy/
  rule_policy.py                 高层状态调度、初始化与确认流程
  probe_episode.py               原子探测：PROBE → HOLD → RETURN → DONE
  local_tracking.py              最近接触窗口、下一 anchor 的短路径
  local_recovery.py              固定已知 anchor、递进局部角域、候选初筛
  boundary_estimation.py         PCA/两点拟合、残差、历史目标侧/手性
calibration/scan_calibration.py   双点 P0/P1 数据、方向与姿态校验
safety/force_guard.py             raw/processed F/T 上限和力上升率
safety/safe_return.py             既有受监控三段返回，不参与轮廓策略
experiment_logging/
  data_logger.py                 逐周期、边界点、事件与恢复射线 CSV
  probe_log.py                   真机/模拟共用 episode 与力时间序列格式
config/loader.py                 配置读取；根目录 config.yaml 路径不变
simulation/
  simulator.py                   数据源/执行器装配与独立结果评分
  geometry.py                    环境几何（policy 不可见）
  simulated_force_sensor.py       柔性探头感力包络、背景、噪声模型
  simulated_robot.py             执行命令积分，不裁剪碰撞、不生成轨迹
  physical_validation.py         连续运动线段穿透、anchor 返回独立审计
  loop_completion.py             正方形覆盖、环绕、闭环和接触位置评分
  top_view.py                    唯一 XY 主视图、结果图、动画
  scene_square.yaml              默认 100 mm 方形实验
tests/                           无硬件单元/回归/故障注入测试
old/                             历史平铺工程与旧策略的两个 tar.gz（不再导入）
```

日志包命名为 `experiment_logging`，避免遮蔽 Python 标准库 `logging`。
旧 EDGE/CORNER 策略及对应旧测试完整保存在
`old/policy_before_local_tracking_20260911.tar.gz`，不是当前可执行算法。
原根目录平铺兼容文件在 `old/legacy_flat_project_20260911.tar.gz`；两个包都已校验可列出。

## 数据与权限边界

```text
PX6D / simulated sensor
    ↓ raw wrench
zero bias → filter → gravity correction → sensor-to-base → granular baseline
    ↓ processed wrench             actual TCP pose / speed
    └───────────────────────────────┬───────────────────────┘
                            RuleBasedPolicy
                                  ↓ PolicyCommand
                    RTDE controller / simulated robot
                                  ↓
                             next observation

observations / episodes → logger / XY plot / simulation physical audit
```

raw 同时送入安全后备检查与日志；接触几何只来自 TCP contact poses。合力参考点平移不改变
力向量，现有完整 wrench 变换的力矩臂能力保留。Tx/Ty/Tz 不进入拟合。

`app/main.py` 不计算边界方向；真实与模拟都只实例化 `RuleBasedPolicy(policy_config)`。
目标几何仅在模拟传感器、独立验证器和绘图使用，不能传入策略或修改策略的下一步命令。

## 状态和每次调用

高层只有 TARGET_SEARCH、LOCAL_INITIALIZATION、BOUNDARY_TRACKING、BOUNDARY_RECOVERY、
BOUNDARY_CONFIRMATION，以及 STOP/正常停止返回控制状态。PROBE/HOLD/RETURN 是 episode 子状态，
TANGENT_STEP 等是受力保护的 anchor 转移子状态。

`update()` 做四件事：检查安全 → 执行一个转移增量或 episode 增量 → 若已返回则分派完成结果
→ 返回命令。它不包含几百行嵌套状态分支；单次探测生命周期在独立模块。
所有决策处理函数在返回后运行，不能从接触点直接开始切向步进。

正常 Q：STOP_SCAN → RETURN_TO_START → STOP，仍由主程序和 safe_return 编排。
安全/通信异常、搜索失败、初始化失败、恢复耗尽：停止；不强制完成探测返回。
正方形 SUCCESS 由模拟外层判定，须全部探测已返回；不会伪造 policy 的 DONE。

## 修改入口

改 PCA/点间距：boundary_estimation / local_tracking；改射线搜索预算：local_recovery；
改触力停止/原路返回：probe_episode；改初始化/确认状态衔接：rule_policy；
改串口/预处理/RTDE/安全返回：对应职责目录。单元测试不能连接真实设备。

## 配置和旧字段

新有效参数包括 initialization_*、local_fit_*、recovery_sector_extents_deg、
recovery_angular_resolution_deg、confirmation_contact_count，以及既有速度/阈值/最大距离。
已删除无效的旧 force-normal 滤波、固定 corner anchor、15°起始角等配置；不静默把旧角点参数
转译为新几何策略。部分历史 CSV 的 corner_id/possible_corner 名称为离线兼容保留，
possible_corner 恒 false，含义不再是几何角点分类。

硬件 IP、串口路径、TCP、标定文件、运动速度及安全返回参数没有因本次重构而更改。
默认 mock 和菜单 2 仍是离线正方形验证；菜单 6 / run_simulation.py 的同一 XY axes
支持 square/circle/triangle 选择与鼠标拖动。拖动仅改变合成传感器、绘图和独立评分器的
target，然后重建实验；不把 target 对象、形状或中心传给 policy。所有 simulation 都不证明真机可用。
