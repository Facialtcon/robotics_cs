# 沙箱坐标标定、旁路记录与俯视图

本功能只读取现有观测、转换坐标并保存图片。主扫描仍使用原 robot base frame；workspace 的矩形不代替原有安全工作区，也不向 policy 或控制器提供目标点。

## 使用

在 `ur7e_px6d_contour` 目录、已安装项目依赖的 Python 环境运行：

```bash
python calibrate_workspace.py
```

程序仅建立 **RTDE Receive** 连接。用示教器将同一个探针 TCP 尖端依次移到沙箱四个相邻角 P0 → P1 → P2 → P3，每点静止后按 Enter。P0→P1 定义正 X，P0→P3 定义正 Y。这里的四个 workspace 点与旧扫描方向标定中的 P0/P1 是不同数据。

四次采样后显示原始完整 TCP pose、理想矩形、尺寸和拟合误差；核对后输入 `SAVE`。默认保存到：

```text
workspace/config/workspace_calibration.yaml
```

再次标定会先备份旧文件。程序拒绝覆盖原 `config.yaml`、`scan_calibration.yaml` 或 `--config` 指定的文件。取消或通信失败不会写入未完成的标定。读取到非有限 pose、机器人正在运动或处于保护/急停状态时不接受该采样。程序不发送运动、停止、TCP 设置或脚本上传命令。

使用示教器核实正在使用正确的探针 TCP，并在四角采样和后续扫描中保持一致。Receive 接口不能回读 active TCP offset，因此保存的 `configured_tcp_offset` 仅是配置声明，`active_tcp_verified` 明确为 false。

之后仍按原方式启动扫描：

```bash
python run_project.py
```

标定文件存在且有效时，当前实验目录自动增加：

| 文件 | 内容 |
| --- | --- |
| `workspace_scan.csv` | TCP 样本、接触记录和已接受轮廓点的 base/workspace 坐标 |
| `workspace_calibration_snapshot.yaml` | 本次扫描固定使用的标定快照 |
| `workspace_contour.png` | 黑色工作区边框、TCP 轨迹、接触点、轮廓点和顺序连线 |
| `workspace_summary.json` | 标定 ID、样本/接触/轮廓计数和字段语义 |

没有标定文件时，不加载 workspace 模块、不产生附加日志，原扫描行为保持不变。坏标定或旁路写入/绘图错误仅报告 `[workspace]` 信息并停用附加输出。原日志、终止原因和扫描决策仍由原工程管理。

真机在现有日志写入点旁路记录，每 20 个 TCP 样本刷新一次，轮廓事件立即刷新；结束时关闭 CSV 并生成图片。仿真在原结果文件写完后离线导出相同 schema，不改变仿真步进。正常退出及现有异常清理路径会保存已记录数据；进程被强制杀死时无法承诺最后一段缓冲区或图片已写完，可用已有原始日志离线重建。

## 坐标约定

原始点保存 `x,y,z,rx,ry,rz`，单位 m / rad。拟合只使用 XY，TCP 姿态不参与计算；所有理想四角的高度为四个实测 Z 的平均值。

```text
x = normalize(P1.xy - P0.xy)
v = P3.xy - P0.xy
y = normalize(v - dot(v, x) * x)
L = (norm(P1.xy-P0.xy) + norm(P2.xy-P3.xy)) / 2
W = (norm(P3.xy-P0.xy) + norm(P2.xy-P1.xy)) / 2
R0.xy = mean(P0.xy, P1.xy, P2.xy, P3.xy) - (L*x + W*y)/2
R0.z = mean(P0.z, P1.z, P2.z, P3.z)
```

采用四角中心拟合原点，避免让某一次 P0 示教误差独自决定整个矩形位置；因此 R0 不必与原始 P0 完全重合。R1/R2/R3 分别由 L、W 和正交轴生成。尺寸采用对边的 XY 长度平均，Z 高差不会增大边长。原始畸变和拟合残差保存到 `diagnostics`，不会通过重排角点隐藏错序。

`rotation_workspace_to_base` 的列依次为 x、y、z 轴，`z = cross(x,y)`；`translation_workspace_to_base = R0`。对应变换为：

```text
p_workspace = R.T @ (p_base - R0)
p_base = R @ p_workspace + R0
```

逆时针示教时 Z 指向 base +Z；顺时针示教时 Z 指向 base −Z。两种情况下都是右手系，X/Y 始终指向所示教的 P1/P3，不会为了强制 +Z 而偷偷翻转 Y。图中 X 向右、Y 向上，四角固定为 `(0,0),(L,0),(L,W),(0,W)`。

`z_workspace` 保留实测 TCP 相对平均平面的有符号高度，不把真实测量改成零。俯视图只绘 workspace XY，相当于投影到统一水平面。二维仿真原日志不保存 Z，其机器人定义在 base Z=0，因此仿真导出明确使用 0 并在摘要中说明。真实日志缺 Z 则拒绝转换，不填造高度。

API 示例：

```python
from workspace.workspace_transform import WorkspaceTransform

frame = WorkspaceTransform.from_file()  # 默认标定路径
point = frame.base_to_workspace([0.61, 0.15, 0.093])
# 同时支持 N×3 数组。六维 pose 请显式取 pose[:3]。
```

## 日志与图的含义

`workspace_scan.csv` 的 `record_type` 区分：

| 类型 | 来源与顺序 |
| --- | --- |
| `sample` | 原 `samples.csv` / `simulation_log.csv` 的 TCP 顺序，含搜索、探测、退回与安全返回轨迹 |
| `contact` | 原 `probe_episodes.json` 中 outcome 为 CONTACT 的 `contact_pose`，含未被接受为边界的接触 |
| `boundary` | 原 `boundary_points.csv`，保留 policy 接受后的点序，不按接触时间重新排序 |

图中的 `Robot trajectory` 只显示扫描过程：保留初始搜索、局部探测、probe 回撤、tracking 和 recovery。
启动定位/正常安全返回的高层 `RETURN_TO_START`（以及 `STARTUP_RETURN`、`SAFE_RETURN`）样本
仍完整保存在 CSV，但不投影进扫描图，避免高空平移看起来像第二条箱内扫描路径。
隐藏的定位段前后不会跨段直线连接；没有高层状态的旧记录保持原显示。图例固定在坐标轴右侧外部。

`contact_state` 保留样本的原 `contact_flag`；`contact_point=1, contact_source=threshold_detection` 表示该 probe 的首次阈值检测，无 probe ID 的旧日志按上升沿记录。持续受力不重复生成阈值标记。中途 ABORTED 或不稳定接触不会升级成已确认接触。

结束时读取现有 probe 文件追加接触事件；图中优先显示同一个 probe 的已确认接触位置，其余阈值检测用淡色标记。旧 probe 文件没有独立接触时间/三轴力；仅当同 probe 阈值样本的位置吻合时复用这些字段，否则留空，不能把探测结束时刻冒充首次接触时刻。

`boundary_state=ACCEPTED` 才是已接受边界。边界事件的时间可能早于当前样本时间（初始化排序会出现此情况）；未知的历史 policy state/UTC 时间留空。轮廓按接受顺序连接，不补造闭合边，也不做新的轮廓重建。

力 `Fx,Fy,Fz` 原样复制已有日志的 `dfx,dfy,dfz`，`force_frame=existing_processed_wrench`。这里只转换位置，不改变 PX6D 处理或假定力已属于 workspace frame。

`inside_workspace` 仅是 XY 是否落在拟合矩形内的诊断标记。扫描观察中的越界点保留真实坐标、完整绘出，不裁剪、不停止机器人。标定 ID 绑定几何；绘图会拒绝 CSV 与输入标定不匹配的组合。重画旧实验应使用该实验的快照。

## 离线处理已有实验

不连接机器人、不运行 policy：

```bash
python visualize_workspace.py \
  --run-dir data/run_20260914_023254_164222 \
  --calibration /path/to/measured_workspace_calibration.yaml \
  --output-dir data/my_workspace_replay
```

默认只读取 `samples.csv`（否则读取 `simulation_log.csv`）、边界 CSV 和现有 probe JSON；不会把规划 waypoint 当作真实轨迹。再次导出到同目录需显式 `--overwrite`，只替换 workspace 输出。

只从已有 workspace CSV 重画：

```bash
python -m workspace.workspace_visualizer \
  --csv data/my_workspace_replay/workspace_scan.csv \
  --calibration data/my_workspace_replay/workspace_calibration_snapshot.yaml
```

导入已保存的四个完整 TCP pose（JSON/YAML；仍需 `SAVE`）：

```bash
python calibrate_workspace.py --points-file four_poses.yaml --output /path/to/workspace_calibration.yaml
```

真机与仿真共用变换/图层，但它们的 base 坐标不自动配准。合成场景需要对应其场景坐标的四角标定；将真机沙箱标定用于原点附近的仿真场景，越界显示是正确的坐标结果，不会影响仿真算法。

## 文件改动边界与验证

新增 `workspace/` 数学、日志、绘图模块，独立 `calibrate_workspace.py`、`visualize_workspace.py` 入口及 `tests/test_workspace_*.py`。已有代码只在 `experiment_logging/data_logger.py` 加可选旁路调用、在 `simulation/simulator.py` 最终导出阶段加可选调用。原 calibration、policy、RTDE、PX6D、运动规划和扫描参数未修改。

离线测试覆盖 XY/Z/TCP 姿态误差、旋转/顺逆时针/负坐标、正交矩形与变换往返、错序与退化拒绝、标定一致性、假 Receive 交互、旁路错误隔离、接触去重和历史实验回放。绘图使用独立 Figure/Agg canvas，不切换调用方的全局 Matplotlib backend。

历史实验 `run_20260914_023254_164222` 的回放使用**合成测试四角**，用于验证 18,165 个 TCP 样本的转换及已记录接触/轮廓位置。它不能证明真实沙箱尺寸，也不能将该次只有少量已接受边界点的实验解释成完成了一圈扫描。项目不附带启用的默认标定，须由真实示教产生。

本次实际回放输出在 `data/workspace_offline_validation_20260914/`：`workspace_contour.png`、`workspace_scan.csv`、快照及 `offline_validation.json`。其中 13 个确认接触和 3 个已接受边界点均来自原实验日志；坐标变换往返的最大误差为 `5.55e-17 m`。图标题明确标注 synthetic workspace，此目录中的标定不会自动启用到真机。

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv312/bin/python -m pytest -q tests/test_workspace_*.py
```

这里禁用环境自动加载的 ROS pytest 插件，仅为离线测试环境隔离；不改变项目运行配置。
