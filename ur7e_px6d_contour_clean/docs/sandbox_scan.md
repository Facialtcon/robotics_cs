# 固定起点、全沙箱覆盖搜索

在终端直接复制下面三行，依次进入项目目录、激活现有 Python 环境并打开新模式菜单：

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean
source /home/user-linux/robotics_cs/.venv312/bin/activate
python run_sandbox_scan.py
```

每次新开终端都需要重新激活环境。激活后，`python` 使用 `/home/user-linux/robotics_cs/.venv312/bin/python`。如果出现“找不到命令 python”，先执行上面的 `source` 命令。

也可以直接使用现有环境的 Python 完整路径启动，无需先激活：

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean
/home/user-linux/robotics_cs/.venv312/bin/python run_sandbox_scan.py
```

新入口提供以下菜单。原 `run_project.py`、`run_continuous_tracking.py`、`run_fixed_target_tracking.py`、`main.py` 的命令和默认行为不变。

| 选择 | 操作 |
| --- | --- |
| 1 | 加载已有标定、预览，再按原流程确认开始 |
| 2 | 仅重新标定 P0，保存并重新生成预览 |
| 3 | 仅重新标定沙箱四角，保存并重新生成预览 |
| 4 | 重新标定全部，保存并重新生成预览 |
| 5 | 只加载、离线生成预览 |
| 0 | 退出 |

## 标记、保存和加载

选择菜单 **2** 时，只需在示教器手动移到新 P0，停稳后按一次 Enter 读取；已有四角、扫描高度及其他设置保留。选择菜单 **3** 时，只读取四角，保留已有 P0 和扫描高度。选择 **4** 时依次读取 P0、C0、C1、C2、C3，已有扫描高度仍保留。四角按周界连续排列，顺时针或逆时针均可。标定模块只复用只读 RTDE、停稳判定、active TCP 绑定和 YAML 保存方式读取及保存原始点，不发送标记运动，也不在标定文件中计算或缓存拟合结果。

标定默认保存在 `calibration_data/sandbox_scan.yaml` 的 `sandbox_scan` 字段，包括原始 P0 位姿、四角原始位姿、独立的扫描高度和 TCP 身份。重新标记完成后才原子替换文件，保留带时间戳备份；中途取消保留原文件。旧文件即使带有先前的矩形拟合缓存，新模式也只读取原始点并重新计算，无需重新示教或迁移标定。默认新标定文件尚不存在时，直接使用原工程已确认的扫描 P0 / fixed_z 和箱体四角，在内存中组合规划，不写回旧文件；新标定保存后优先加载新记录。可用 `--calibration 路径` 选择另一份标定。

旧 `scan_calibration.yaml`、单点标定和 `workspace/config/workspace_calibration.yaml` 不会被默认新流程覆盖，不要求原模式重新标定。已有默认旧记录也可以直接用于单独重标，结果保存到新的沙箱标定文件。只重标 **P0** 的完整命令：

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean
source /home/user-linux/robotics_cs/.venv312/bin/activate
python run_sandbox_scan.py --recalibrate-p0
```

同一环境中，仅重标四角使用 `python run_sandbox_scan.py --recalibrate-corners`，重标全部使用 `python run_sandbox_scan.py --recalibrate`。这三种操作都在所有所选点采集成功后保存，并自动重新拟合、规划和打开预览；默认不开始扫描运动。采集中 Ctrl+C、EOF 或取消会保留原文件；四角只采集了一部分也不会写入。如需重标后直接进入原启动确认流程，可在所选标定命令后加 `--execute`，不增加额外确认。

扫描高度只在**首次创建**新标定时依次取原 `calibration.file` 的 `fixed_z`、原 `robot.fixed_z`，两者都未提供时按原 P0 约定取 P0.z。已有文件重新标定时保留原 `scan_z`，新 P0 的高度不会覆盖它。四角高度只记录，不决定扫描高度。P0 示教高度单独保留，实际水平搜索使用扫描高度；配置中的可选 `sandbox_scan.scan_z` 仍可独立覆盖执行高度。

## 参数

继续使用原 `config.yaml`；缺少 `sandbox_scan` 整段也能运行。只有新入口读取该可选段，不影响旧模式：

```yaml
sandbox_scan:
  line_spacing: 0.010  # m，默认 10 mm，最大相邻行距
  # scan_z: 0.0267893  # m，可选；未提供时使用新标定保存的 scan_z
  # boundary_margin: 0.010  # m，可选；覆盖搜索矩形的向内余量
  # calibration_file: calibration_data/sandbox_scan.yaml
```

余量默认继承原连续搜索的停车余量计算，当前原配置为 3.7125 mm；不再把拟合残差叠加为余量。原始四边形的每条边向内偏移该距离，拟合矩形也向内保留该距离，再在两者内部计算有效扫描矩形。显式设置的余量只影响覆盖规划。所有搜索点仍须满足原工程运动范围及原搜索停车余量；范围冲突会在连接设备前明确报错，不扩大旧边界。

## 几何计算与兼容

`calibration/sandbox_scan.py` 只读写原始点。`workspace/workspace_calibrator.py` 负责拟合和有效区域；`workspace/sandbox_scan.py` 只接收已计算的区域及规划参数生成路线。

新模式显式使用 `fit_workspace(..., method='least_squares')`：保持 C0/C1/C2/C3 对应关系，联合优化矩形中心、朝向和长宽，使四角 XY 平方误差总和最小。中心取四点均值，其余变量通过 2×2 特征值问题得到全局最小二乘解，不由首边或 Base 坐标轴指定朝向。原始四角及其 Z、姿态保持原样，扫描高度仍独立确定。

有效区域固定拟合中心和朝向，通过原始四边形各边的半平面约束，计算与拟合矩形内缩区域相交后可容纳的最大面积矩形。它保证整块有效矩形及其内部扫描线、转弯连接段都在原始四边形内，并满足边界余量；不通过添加大圈余量掩盖拟合偏差。

公共 `fit_workspace()` 的默认算法和旧标定加载保持原样，只有新模式显式选择四角最小二乘。轮廓跟踪、圆弧恢复、P0 进入方式和扫描高度约定不变。

当前已保存四角的离线对比（单位 mm，RMS 为四个角点二维位移模长的均方根）：

| 拟合方法 | 角点最大偏移 | RMS |
| --- | ---: | ---: |
| 原首边定向算法 | 24.942 | 23.109 |
| 四角联合最小二乘 | 8.898 | 7.932 |

搜索速度沿用 `continuous_tracking.search_speed`。折返前按原加速度参数减速并通过原执行器停稳后进入下一段。接触后速度仍用原 `continuous_tracking.tangential_speed` 及启动时原倍率选择，力处理、控制公式、阈值、跟踪和圆弧恢复参数保持原样。抬升高度、转移速度和下降速度仍由原 `safe_return` 段配置。

## 预览与执行

仅生成离线预览：

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean
source /home/user-linux/robotics_cs/.venv312/bin/activate
python run_sandbox_scan.py --preview
```

加载标定、生成预览后，按原 Enter 确认流程执行真机搜索：

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean
source /home/user-linux/robotics_cs/.venv312/bin/activate
python run_sandbox_scan.py --execute
```

`--preview` 纯离线，不连接机器人或传感器。直接使用已保存原始点重新拟合和规划，完整打印规划与参数，默认写入 `data/sandbox_preview/sandbox_scan_plan.json`、独立的 `sandbox_scan_geometry.json` 和 `sandbox_scan_preview.png`，不写回原始标定文件。可用 `--preview-dir` 更改预览目录。实际执行同样先生成预览并尝试显示窗口，再进入原运行器的 Enter 路径确认、空气采零、速度选择、下降和颗粒背景流程，不额外增加重复确认。

执行及标定完成后的预览窗口独立运行，终端等待 Enter 或程序执行时仍能正常重绘、缩放和拖动。窗口缩小时文字、箭头、标记和图例同步缩放，兼容 2 倍 DPI 显示；参数和折返放大图分别排版，避免文字重叠或裁切。关闭预览窗口只关闭图形，原终端确认和停止方式保持不变。也可直接打开 `data/sandbox_preview/sandbox_scan_preview.png` 查看保存的完整轨迹。

预览按 Base XY 等比例绘制，X/Y 使用相同网格间距，图例、完整参数和顺序说明放在轨迹外侧。保留原始四边形、拟合矩形和有效扫描区域；P0 为紫色星形、覆盖起点为绿色方形、终点为红色叉形，紫色点划线为进入路线。前四条长扫描段分别用红、紫、青、棕色突出，每条扫描线中部都有方向箭头；换行连接段为橙色，绘制在区域边界上方。①～⑦按实际执行顺序标出“扫描1 → 换行 → 扫描2 → 换行 → 扫描3 → 换行 → 扫描4”，右侧同时显示每段在 Base XY 的实际方向。首个折返处单独等比例放大，显示到达端点、横移换行和反向扫描。

路线说明直接取自执行器接收的同一份有序 `coverage_xy`，不另生成展示路线，也不根据候选选择标志猜测方向。长边/短边按实际有效矩形的边长判断，逐条推进方向、总行数和实际间距也从该路径读取。当前保存的点位对应：沿长边往返、向右偏上推进，共 **24 条线**，实际间距 **9.888 mm**；先向下偏右扫描，再向右偏上换行，再向上偏左扫描，如此往返。重标后依旧按原选择规则自动计算，以新预览为准。

预览文字会区分 P0 的三种位置：「原始沙箱外」「沙箱内，但在有效扫描区域外」「有效扫描区域内」。这只是说明，不增加启动限制，进入方式仍沿用下述流程。JSON 的 `entry_xyz` 给出名义进入路线高度；实际当前位置及完整启动路线由原运行器在运动前显示。

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
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean
source /home/user-linux/robotics_cs/.venv312/bin/activate
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_sandbox_geometry.py tests/test_sandbox_scan.py tests/test_sandbox_operations.py
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 避免终端中 ROS 的外部 pytest 插件干扰此 Python 环境，不修改环境或项目依赖。验证旋转与带误差矩形、顺逆四角顺序、高度差独立性、不同 P0 最近进入端、不可整除行距、扫描与连接段在有效区内、带加速/延迟模型的完整无目标覆盖、独立重标不改变其他数据及扫描高度、部分采集后取消、旧标定直接单独重标、菜单及命令行保存后重画、三种 P0 位置、预览与执行点逐段一致、往返箭头与顺序、参数文字不越界、旧配置与标定、默认策略命令一致性，以及原 RTDE 执行器下的箱内/箱外进入、首次接触、跟踪和丢边圆弧恢复。测试由现有 `conftest.py` 禁止真实硬件和网络连接。
