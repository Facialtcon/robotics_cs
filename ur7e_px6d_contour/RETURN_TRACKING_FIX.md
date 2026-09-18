# 回撤完成与 tracking 净空接续修复

本文记录上一轮修复。2026-09-14 后续新增的转移停稳、首次接触局部返回和速度翻倍，
见 [SCAN_EXECUTION_UPDATE.md](SCAN_EXECUTION_UPDATE.md)；下文的配置及测试结果为上一轮记录。

针对 `data/run_20260914_104553_980818` 的故障，修正了共享 policy 中的两个接续条件。真机和仿真仍调用同一个 `RuleBasedPolicy`；启动方式仍为 `python run_project.py`。

## 原因与修复

原程序在距原 anchor 0.191 mm、TCP 仍以约 6.36 mm/s 回撤、处理后 Fxy 仍为 1.039 N 时，仅按 0.2 mm 位置容差判定返回完成。下一帧 Fxy=1.0125 N 触发 transfer 的 1 N 接触保护。第二次切向移动尚未执行。

现在 `ProbeEpisode` 保持 RETURN 状态，直到以下条件同时持续满足：

- XY 在原 anchor 的既有位置容差内。
- 实测 TCP 三维线速度不高于 0.5 mm/s。
- 处理后的 Fxy 严格低于原 `contact_threshold`。
- 连续保持原 `contact_hold_time`，当前真机配置为 0.05 s。

位置已到位但尚未稳定时输出零速；漂出位置容差则继续返回同一个不可变 anchor。首次到位后超过 2 s 仍未完成连续确认，停止并记录 **STOP_ANCHOR_ERROR**，详情含位置误差、速度、力及等待时间。该 2 s 不限制尚未到位的原长距离搜索回撤。原始力、处理后安全力、力变化率、工作区及急停保护保持有效。

本次 tracking 探测射线只有 0.409 mm。旧规划截断退让后，法向净空只有 0.3495 mm，却仍排入普通转移。现在净空不足会抛出明确几何异常，不返回可直接执行的短净空路径。

如果该 probe 存在与之绑定的、实际已执行切向段，就反向返回这段路径的实测起点 Q，日志阶段为 `CLEARANCE_ROLLBACK`。只有 Q 在当前拟合法向下仍满足原 `retract_distance`，才从 Q 沿当前切向生成下一 anchor。找不到匹配的已知路径或该点仍无足够净空，则 STOP_ANCHOR_ERROR。这里不延长未探测射线，也不改探测方向选择、边界拟合算法或 recovery 搜索算法。

Q 与 probe ID、原始 anchor 同时绑定；新初始化/recovery probe 不复用前一次 tracking 的旧 Q。回退经过原 `_execute_transfer()`，Fxy 超接触阈值仍立即 STOP_UNEXPECTED_CONTACT。实际回退使用已执行段的实测端点和现有位置容差，不声称逐采样点精确反演轨迹。

## 距离与参数

切向补偿公式保持不变：

```text
next_anchor = Q + tangent * (required_progress - dot(Q - contact, tangent))
```

其中 `required_progress` 保留原重复/后退接触修正逻辑。公式确保最终 anchor 相对接触点的正向切向投影；debug 的 `actual_tangent_progress_mm` 是生成目标的几何量，不是机器人已执行的位移。

为减少位置误差及新接触拟合波动消耗退让距离，射线上的规划净空采用：

```text
planned_clearance = retract_distance + position_tolerance + local_fit_max_residual
```

这两项余量沿用已有配置的误差尺度，不是对真实误差的上界保证。当前真机为 `3.0 + 0.2 + 0.6 = 3.8 mm`；最低允许几何净空仍是原 3.0 mm。规划仍受已观察探测射线长度限制：无法达到规划余量但满足最低净空时可用，低于最低净空则必须走上述回退/停止路径。

未修改任何配置文件、接触阈值、运行速度或原安全限值。新增就绪条件可选键的默认值如下，两种执行入口完全一致：

| 可选 policy 键 | 默认值 |
| --- | --- |
| `return_settle_hold_sec` | 原 `contact_hold_time` |
| `return_settled_speed_mps` | 0.0005 m/s |
| `return_settle_timeout_sec` | 2.0 s |

当前无需添加这些键。正式 policy 每帧传入实际 `RobotState.tcp_speed`；仅为兼容旧的离线几何调用，直接调用 `ProbeEpisode.tick()` 未传速度时按零速处理，仍必须经过连续确认窗口。

## 验证与修改范围

业务代码仅改 `policy/probe_episode.py`、`policy/local_tracking.py`、`policy/rule_policy.py`。控制器、传感器处理、simulation 实现、workspace、旧 calibration 和配置文件均未改。

离线测试覆盖本次真实 TCP/速度/原始力/处理后力输入、持续就绪及超时、回退来源绑定、保护条件、净空补偿和原简单形状整圈扫描。历史记录止于停止帧；测试后续的卸载稳定数据为明确标记的合成输入，不能作为真机完成后续扫描的证据。没有连接机器人运行测试。

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv312/bin/python -m pytest -q
```

原测试中单帧假定返回完成的辅助函数已改为提供连续稳定样本；方形结果文件检查允许先前已实现的可选 `workspace_contour.png`，不依赖现场是否已存在 workspace 标定。

本次完整检查结果：**370 passed，97.64 s**。包括方形顺/逆时针、两个平移三角形、旋转矩形闭环，以及此次真实日志的 34 帧输入回放。回放后的释放与转移使用合成输入，真机尚未执行修改后的完整扫描。
