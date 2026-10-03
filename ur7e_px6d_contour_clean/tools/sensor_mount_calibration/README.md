# PX6D 自动多姿态重力标定

**现场准备**：将工具移到空旷处，保持无接触、静止，确认工具与传感器连接牢固，线缆全程不拉拽、卡住或擦碰；等待原始力读数稳定，确认基座水平（重力取 Base −Z），TCP 与配置一致。检查整套工具、法兰、机械臂各连杆及线缆在 ±25° 倾斜与返回路径上的空间，不能只看 TCP 尖端。保持机器人原有安全限制，急停可用。

**启动命令**（默认只看离线示例，不连接设备、不运动）：
```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour_clean
/home/user-linux/robotics_cs/.venv312/bin/python tools/sensor_mount_calibration/calibrate.py
```
真机独立入口：
```bash
/home/user-linux/robotics_cs/.venv312/bin/python tools/sensor_mount_calibration/calibrate.py --execute
```

**Enter 开始**：程序显示当前实际 TCP、12°/25° 计划及运动限制。确认基座水平和整机空间后按 Enter；自动预检、低速运动、停稳采样。终端持续显示预检进度、姿态编号/总数、移动误差、采样帧数、回中复测及解算结果。预检检查原地力趋势，每次回中复测原始力重复性；出现漂移会停止，不逐姿态采零。正常完成返回起始标定姿态，不进入扫描、不自动扩大范围。

**退出方式**：Q / Esc / Ctrl+C；超时、原始力/矩超限或机器人异常会立即请求停止，异常后不自动返回。仅接受前台终端的新 Enter，不接受管道或残留换行。

**结果保存**：独立目录 `calibration_data/sensor_mount/mount_*` 保存原始力/矩、实际姿态、时间、计划和诊断；`timing.jsonl` 记录读取、看门狗、日志入队及落盘耗时。日志后台写入，写入失败或持续积压会停止；结束后确认全部落盘，再验证和保存。拒绝时终端报告回中变化及异常路段；`metadata.json` 的 `diagnostics.failure_analysis` 保存参考历史、持续趋势或突变候选时刻及力/矩变化，供现场排查，不代替物理原因确认。通过覆盖、稳定性、符号可辨识性及完整留出姿态验证后，显示误差、矩阵和配置差异；再次按 Enter 才备份并更新必要安装旋转。零偏、重量和作用方符号仅作诊断，不改变 TCP、负载或扫描补偿。拒绝的数据不安装结果。离线复算（不写配置）：
```bash
/home/user-linux/robotics_cs/.venv312/bin/python tools/sensor_mount_calibration/calibrate.py --offline calibration_data/sensor_mount/mount_目录/raw.jsonl
```
