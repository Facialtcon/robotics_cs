# PX6D 自动多姿态重力标定

**现场准备**：将工具移到空旷、关节舒展处，保持无接触、静止，确认工具与传感器连接牢固，线缆全程不拉拽、卡住或擦碰；确认基座水平（重力取 Base −Z），TCP 与配置一致。`wide` 为 20°/45° 粗标定，按当前 TCP 偏置，法兰相对起点可能移动约 197 mm；须检查整套工具、法兰、连杆和线缆的倾斜及返回空间，不能只看 TCP。保留机器人安全限制，急停可用；预检拒绝后人工调整起点，不自动扩大范围或换档重试。

**启动命令**（大幅度粗标定预览，不连接设备、不运动）：
```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean
/home/user-linux/robotics_cs/.venv312/bin/python tools/sensor_mount_calibration/calibrate.py --profile wide
```
真机独立入口：
```bash
/home/user-linux/robotics_cs/.venv312/bin/python tools/sensor_mount_calibration/calibrate.py --profile wide --execute
```

**Enter 开始**：核对当前实际 TCP、档位、完整路径及限制后按 Enter；自动预检、低速运动、停稳采样。`wide` 仍采 17 个姿态、每次回中，目标最大 45°、实测硬限 46°、关节相对起点上限 1.5 rad；速度、2 mm 平移和时间上限保持原值。回中力差容限 0.25 N；预检趋势 >0.30 N/min 且首尾差 >0.15 N 时拒绝，超过保守档但仍在粗标定预算内会提示。终端持续输出预检、姿态/总数、移动、采样、回中及计算进度。省略 `--profile wide` 则保持原 12°/25° 保守档。正常完成回起点，不进入扫描。

**退出方式**：Q / Esc / Ctrl+C；超时、原始力/矩超限或机器人异常会立即请求停止，异常后不自动返回。仅接受前台终端的新 Enter，不接受管道或残留换行。

**结果保存**：独立目录 `calibration_data/sensor_mount/mount_*` 保存原始力/矩、实际姿态、时间、档位、计划和诊断；后台日志异常会停止，关闭设备并确认落盘后才解算。`wide` 要求拟合 RMSE ≤0.20 N、独立姿态 RMSE ≤0.25 N，还须通过相对误差、符号和 ≤5° 局部 95% 角度不确定度估计检查；该估计不是绝对精度保证。显示误差、矩阵和配置差异后，再按 Enter 才备份并更新安装旋转。零偏、重量、作用方符号仅作诊断，不采零、不扣漂移，不改变 TCP、负载或扫描补偿。拒绝只留数据和诊断。离线复算沿用原记录档位，旧数据默认保守档，不写配置：
```bash
/home/user-linux/robotics_cs/.venv312/bin/python tools/sensor_mount_calibration/calibrate.py --offline calibration_data/sensor_mount/mount_目录/raw.jsonl
```
