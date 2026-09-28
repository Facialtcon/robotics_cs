# UR7e + PX6D clean

后续开发目录是 `ur7e_px6d_contour_clean/`。这是同一 Git 仓库中的独立 Python 工程，运行不导入旧目录，也不读取旧实验数据。旧工程保留作参考和回退。

日常入口不变：

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean
source /home/user-linux/robotics_cs/.venv312/bin/activate
python run_project.py
```

菜单仍为 1～20；11 返回 reset/P0，12 原离散真机，18 连续真机。完整编号和输出见 [OPERATIONS.md](OPERATIONS.md)。代码职责、设备所有权和停止判定见 [ARCHITECTURE.md](ARCHITECTURE.md)。

本目录只需要 Python 环境和 `requirements.txt` 的依赖，无 ROS、无嵌套 Git、无指向旧工程的软链接。离线模式默认运行，不创建 RTDE 或串口连接。

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
python run_continuous_tracking.py --check-calibration
python run_continuous_tracking.py --dry-run --duration 0.2
MPLBACKEND=Agg python run_simulation.py --no-gui
```

默认连续场景与当前 18 mm/s SEARCH 参数组合会在约 2.75 秒触发原有力上限停止。它验证了停止和日志路径，不代表完成连续轮廓；本次保留原行为。离散正方形场景使用其独立、原样继承的仿真参数，可完成整圈。

现场 `config.yaml`、`scan_calibration.yaml`、`workspace/config/workspace_calibration.yaml`、`requirements.txt` 按字节复制；本地原先没有 `reset_pose.yaml`，不生成替代标定。连续 policy 仅增加真实沙箱几何搜索预算和执行层停稳确认门槛；传感器预处理及导纳、方向、force-rate、reacquire 算法保持原代码。连续 SEARCH 18 mm/s、接触阈值 1 N、切向 1 mm/s、法向上限 0.5 mm/s、参考力 1.5 N 均未调整。

数据从空目录开始。新实验由 `experiment_logging/paths.py` 分配到 `data/{real|simulation}/{continuous|discrete|return}/run_*`；每个 run 先写 `metadata.json`。默认相对路径不受启动时的工作目录影响。数据、Python 缓存和 pytest 缓存由本目录 `.gitignore` 排除；现场配置和标定正常受版本管理。

迁移验收只使用设备替身、仿真、离线回放和 pytest。真实 Control/PX6D、实际停止距离、watchdog 实际响应、30012 只读状态读取和现场三段返回均尚未真机验证。
