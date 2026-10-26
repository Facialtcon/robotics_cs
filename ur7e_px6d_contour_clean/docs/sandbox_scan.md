# 固定起点、全沙箱覆盖搜索

在 `ur7e_px6d_contour_clean` 目录运行（已激活原工程 Python 环境）：

```bash
python run_sandbox_scan.py
```

新入口提供「加载已有标定并开始」「重新标记 P0 / 四角并开始」「离线生成预览」。原 `run_project.py`、`run_continuous_tracking.py`、`run_fixed_target_tracking.py`、`main.py` 的命令和默认行为不变。

## 标记、保存和加载

选择重新标记，在示教器手动移动并停稳，依次按 Enter 读取初始点 P0，以及沙箱四个角 C0、C1、C2、C3。四角按周界连续排列，顺时针或逆时针均可。程序复用只读 RTDE、停稳判定、active TCP 绑定、现有 `fit_workspace` 矩形规整和 YAML 保存方式，不发送标记运动。

标定默认保存在 `calibration_data/sandbox_scan.yaml` 的 `sandbox_scan` 字段，包括原始 P0 位姿、四角、规整矩形、独立的扫描高度和 TCP 身份。重新标记完成后才原子替换文件，保留带时间戳备份；中途取消保留原文件。加载直接复用已有文件。默认新标定文件尚不存在时，直接使用原工程已确认的扫描 P0 / fixed_z 和箱体四角，在内存中组合规划，不写回旧文件，也无需重复标记；新标定保存后优先加载新记录。可用 `--calibration 路径` 选择另一份标定。

旧 `scan_calibration.yaml`、单点标定和 `workspace/config/workspace_calibration.yaml` 不会被默认新流程覆盖，不要求原模式重新标定。单独标记而不启动扫描：

```bash
python run_sandbox_scan.py --recalibrate
```

扫描高度在新标定时依次取原 `calibration.file` 的 `fixed_z`、原 `robot.fixed_z`，两者都未提供时按原 P0 约定取 P0.z。四角高度只记录，不决定扫描高度。P0 示教高度单独保留，实际水平搜索使用扫描高度。

## 参数

继续使用原 `config.yaml`；缺少 `sandbox_scan` 整段也能运行。只有新入口读取该可选段，不影响旧模式：

```yaml
sandbox_scan:
  line_spacing: 0.010  # m，默认 10 mm，最大相邻行距
  # scan_z: 0.0267893  # m，可选；未提供时使用新标定保存的 scan_z
  # boundary_margin: 0.010  # m，可选；覆盖搜索矩形的向内余量
  # calibration_file: calibration_data/sandbox_scan.yaml
```

余量默认继承原连续搜索的停车余量计算，并加本次矩形规整的最大 XY 残差和数值舍入余量；当前原配置的搜索停车余量为 3.7125 mm。显式设置的余量只影响覆盖规划。所有搜索点仍须满足原工程运动范围及原搜索停车余量；范围冲突会在连接设备前明确报错，不扩大旧边界。

搜索速度沿用 `continuous_tracking.search_speed`。折返前按原加速度参数减速并通过原执行器停稳后进入下一段。接触后速度仍用原 `continuous_tracking.tangential_speed` 及启动时原倍率选择，力处理、控制公式、阈值、跟踪和圆弧恢复参数保持原样。抬升高度、转移速度和下降速度仍由原 `safe_return` 段配置。

## 预览与执行

```bash
python run_sandbox_scan.py --preview
python run_sandbox_scan.py --execute
```

`--preview` 纯离线，不连接机器人或传感器。完整打印规划与参数，默认写入 `data/sandbox_preview/sandbox_scan_plan.json` 和 `sandbox_scan_preview.png`。可用 `--preview-dir` 更改预览目录。实际执行同样先生成预览并尝试显示窗口，再进入原运行器的 Enter 路径确认、空气采零、速度选择、下降和颗粒背景流程，不额外增加重复确认。

预览按 Base XY 等比例绘制，X/Y 使用相同网格间距，图例和参数放在轨迹外侧。显示原四角、规整矩形、有效区域、P0、进入段、起点、全 Z 字路线、方向及终点。JSON 的 `entry_xyz` 给出名义进入路线高度；实际当前位置及完整启动路线由原运行器在运动前显示。

路线在矩形局部坐标内生成后转换到 Base。比较两个主方向、两个行序和两个首行方向，共八个候选；优先选 P0 到首点最短的候选，距离相同再选总路线较短者。行数由最大行距向上取整，实际行距均分，因此两侧均有扫描线，末端不会留下大于设置行距的漏扫带。

P0 在有效区域内：原启动流程抬升、采零后下降至 P0 的扫描高度，再以带原接触检测的平面进入段前往第一条扫描线。P0 在有效区域外：复用 `SafeReturnExecutor` 抬升、调正探针，经 P0 上方平移至覆盖起点，再下降。新入口的启动转移允许从标记后的箱角或箱外位置抬升，始终沿用原配置的全局 workspace 保护；箱体平面多边形在启动目标上方、下降前恢复，整个搜索及后续跟踪/恢复沿用原有运动边界。不会在扫描深度从箱外横穿箱壁。

首次接触前执行覆盖路线；候选接触、立即停止、停稳和力方向确认仍由原 `ContinuousTrackingPolicy` 完成。路径策略只提供当前段起点及实际运动方向，原状态机从 `FIRST_CONTACT` 直接进入 `CONTINUOUS_TRACKING`，丢边时调用原 `_recover` 圆弧恢复。确认后不再推进覆盖索引，也不在跟踪完一个目标后返回覆盖搜索。

遍历全部覆盖点仍未确认目标时，通过原停止流程正常结束，报告「区域扫描完成，未检测到目标」。Q / Esc / Ctrl+C、传感器故障和原保护的停止清理均沿用原运行器。

## 记录与离线验证

运行目录继续使用原 `data/real/continuous/run_*` 规则，`--output` 沿用原输出选项。原 `samples.csv` / `tcp_commands.csv` / `policy_waypoints.csv` 保留实际 TCP、力、接触位置及状态；原 summary / termination 保留具体停止原因。新加：

- `sandbox_scan_plan.json`：完整规划、标定几何及参数。
- `sandbox_scan_result.json`：已到达覆盖点数、是否覆盖完成、首次确认接触位置及退出原因。
- `sandbox_scan_actual.png`：规划和实际 TCP 叠加图，标记首次确认接触；原轨迹图及回放继续生成。

定向验证命令：

```bash
python -m pytest -q tests/test_sandbox_scan.py
```

验证旋转与带误差矩形、顺逆四角顺序、高度差独立性、不同 P0 最近进入端、不可整除行距、扫描与连接段在有效区内、带加速/延迟模型的完整无目标覆盖、标记中途取消、旧配置与标定、默认策略命令一致性，以及原 RTDE 执行器下的箱内/箱外进入、首次接触、跟踪和丢边圆弧恢复。测试由现有 `conftest.py` 禁止真实硬件和网络连接。
