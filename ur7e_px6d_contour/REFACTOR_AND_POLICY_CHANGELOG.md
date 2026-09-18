# Refactor and Policy Changelog

日期：2026-09-10

## 2026-09-11：正方形整圈模拟与单图界面

- 新默认场景 `simulation/scene_square.yaml`：100 mm 正方形，无噪声、摩擦或颗粒扰动。
- 保持同一个 `policy.rule_policy.RuleBasedPolicy`；此次未修改 policy 或真实硬件参数。
- 新增 `simulation/loop_completion.py`：仅 simulation 中判断足够边界路径、四面接触、
  同向环绕进展、返回首次接触附近和 tangent 一致，并要求四次恢复成功。
- `simulation/top_view.py` 统一实时主图、结果图和动画回放；恢复辅助元素只在恢复时显示。
- 默认只保存一张 XY 结果内容到 `simulation_outputs/full_loop_square.png`，每次运行在
  时间戳子目录保存同一结果及原始日志。旧 outputs 保留，高级调试图需显式开启。
- 菜单 2、tools/run_mock_visualized.py、main.py 默认 mock 均运行正方形无窗口整圈验证；
  菜单 6 和 run_simulation.py 显示实时单图动画。
- 成功明确打印 FULL LOOP COMPLETED；LOST、超时、提前停止保留实际失败轨迹、状态和原因。
- Python 3.12 下 61 tests passed；本次正方形实际运行成功，47 个边界点、4 次确认恢复，
  平均几何边界距离 0.484 mm。此为理想接触模型结果。
- 排查 PX6D FileNotFoundError：本机实际路径前缀为 usb-28e9，旧配置为 usb-GigaDevice，
  序列号均为 1B556A774A92。本次没有自动改写真实串口参数，操作文档补充核对说明。

## 2026-09-11：收纳旧文件

按用户要求，将根目录 11 个兼容模块移入 `old/`，添加归档说明，并把 tests/tools 的旧导入改为
当前包路径。常用的五个运行入口、配置、真实硬件参数、data 和 simulation_outputs 保留。
以下阶段 A/B 内容记录的是此前的重构过程；当前布局以 ARCHITECTURE.md 为准。

## 目标与阶段纪律

本次先完成纯架构迁移并跑通原 50 项测试、import、CLI、标定只读检查和三种 simulation，之后才
改变 policy 行为。没有修改机器人 IP、PX6D 串口、active TCP、真实 P0/P1、固定 Z/姿态或
safe-return 参数；没有删除 `data/` 或 `simulation_outputs/`。

## 阶段 A：架构重构

实现代码从根目录移到职责目录：

- `main.py` → `app/main.py`
- `operator_input.py` → `app/operator_input.py`
- `models.py` → `core/models.py`
- `config_loader.py` → `config/loader.py`
- `px6d_reader.py`、`force_preprocess.py`、`force_features.py` → `sensor/`
- `ur_rtde_controller.py` → `robot/rtde_controller.py`
- `rule_policy.py` → `policy/rule_policy.py`
- `scan_calibration.py` → `calibration/scan_calibration.py`
- `safe_return.py` → `safety/safe_return.py`
- `data_logger.py` → `experiment_logging/data_logger.py`

根目录同名 Python 文件保留为薄兼容层，所以旧测试、脚本和用户命令仍能 import。根目录
`main.py` 保留可执行 wrapper。`run_project.py`、`run_simulation.py`、`run_calibration.py`、
`return_to_start.py` 保持原入口位置。配置 YAML 没有移动，避免真实标定出现两个可写副本。

阶段 A 验证结果：50 passed；新旧 import 全通过；四个入口 `--help` 正常；标定只读检查正常；
三个 simulation 均可启动和生成结果。此时矩形基线仍会在多轮后进入 LOST，确认架构迁移没有
掩盖原算法限制。

## 阶段 B：Policy 修改

新增模块：

- `policy/policy_geometry.py`：纯方向、旋转、角度、前进量和历史距离。
- `policy/edge_follow.py`：角度 hysteresis、每次更新最大转角、平滑、点间距。
- `policy/boundary_recovery.py`：通用 boundary recovery 的固定 anchor fan 几何。
- `policy/edge_confirmation.py`：候选旧边/反向检查和第二接触检查。
- `safety/force_guard.py`：raw/processed 阈值和 force-rate 计算。

状态词汇从把失败直接叫 corner 改为：

```text
EDGE_FOLLOW → BOUNDARY_LOST → BOUNDARY_RECOVERY
            → EDGE_CONFIRMATION → EDGE_FOLLOW
```

旧 `State.CORNER_SEARCH`、`State.NEW_EDGE_CONFIRM` 名称只作为 Python Enum 兼容别名保留；新日志
输出 `BOUNDARY_RECOVERY` 和 `EDGE_CONFIRMATION`。

具体行为变化：

1. 普通 direction update 增加固定手性、角度 hysteresis、单次最大方向变化和最小边界点间距。
2. 普通 probe、恢复 ray、确认 probe 一达到 contact threshold 就停止向内运动，静止完成
   `contact_hold_time`，避免为了消抖继续推进目标。
3. force-rate guard 覆盖普通 probe、恢复 ray 和确认 probe。
4. probe 到 `max_probe_distance` 先明确进入 `BOUNDARY_LOST`，下一周期才建立恢复几何。
5. 恢复仍使用可靠的 fixed-anchor fan：每条 ray 独立、失败回同一 anchor、角度按手性单向递增。
6. 候选点会对完整历史（排除最后少量邻接点）做距离检查，并检查 old tangent 前进量和反向切向。
7. 候选必须 retract → tangent step → guarded probe → second contact；位置、前进方向、间距和
   interaction-direction consistency 全部通过才成为新边。
8. simulation/debug 可选闭环检测会在足够路径长度、点数、位置接近和 tangent 一致时进入
   `LOOP_COMPLETE`。真机 `config.yaml` 中默认关闭，真实运行仍由 Q 停止。
9. 每次 recovery 生成 `boundary_recovery_<id>.png`；CSV 改为
   `boundary_recovery_rays.csv`。旧数据文件仍可由可视化工具读取。
10. 周期日志增加 interaction direction、policy sub-state、recovery/ray/theta、candidate 状态和
    rejection reason。

## 自动测试与 simulation 结果

最终自动测试覆盖 SEARCH→CONTACT、普通沿边、BOUNDARY_LOST→RECOVERY、固定 anchor、失败 ray
回 anchor、旧边拒绝、反向切向拒绝、两点确认、恢复后回沿边、固定 follow hand、日志诊断字段，
以及三个场景闭环。

在当前确定性合成力模型与默认 simulation 配置下：

| 场景 | 结果 | 边界点 | 已确认恢复 | 平均 ground-truth 边界距离 |
|---|---:|---:|---:|---:|
| axis-aligned rectangle | LOOP_COMPLETE | 49 | 4 | 0.645 mm |
| rotated rectangle 30° | LOOP_COMPLETE | 47 | 4 | 0.644 mm |
| circle | LOOP_COMPLETE | 56 | 0 | 0.651 mm |

这些结果验证软件结构和合成场景，不构成真机安全或性能证明。

## 已知限制

- force direction 受颗粒阻力、摩擦、传感器安装与滤波影响，只是 interaction estimate。
- fixed-anchor fan 是局部凸边界恢复方法；暂不支持复杂 concave、孔洞或多个相邻物体。
- simulation 是二维点 TCP 和简化接触模型，不包含物体真实移动、颗粒动力学或 UR 动力学。
- loop closure 默认不用于真机；真实扫描仍需操作者 Q 停止。
- 真实阈值、frame transform、重力和 granular baseline 必须现场重新确认。

## 真机前检查

依次确认 PX6D raw/processed 符号和噪声、sensor→base rotation、gravity/bias 是否重复扣除、颗粒
baseline、contact/force/torque/force-rate 阈值、probe/recovery 距离、所有速度、follow_hand、
probe_direction_sign、P0→P1、fixed Z/orientation、active TCP、UR safety limits、急停、工作区和
safe-return 三段路径。必须先通过当前 tests 与三个 headless simulation，再做低速、有人监护、
可立即急停的小范围真机验证。
