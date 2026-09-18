# Workspace 四角标定文件

本目录不附带模拟的有效 `workspace_calibration.yaml`。默认标定文件应由现场采样生成；离线导入文件也必须来自可信的实测四角。

从项目根目录运行：

```bash
python calibrate_workspace.py
```

程序只建立 RTDE Receive 连接，读取 TCP pose/speed。按箱体四边依次手动定位 **workspace P0、P1、P2、P3**，每次静止后按 Enter。P0→P1 定义 X，P0→P3 定义 Y；这些标签独立于旧扫描方向标定的 P0/P1。发现移动或无效数据会拒绝该次采样，可重采；输入 `ABORT` 取消。机器人始终由示教器操作，程序不发送运动或停止命令。

四点完成后先显示原始点、拟合矩形、尺寸、原点、旋转和误差；输入 `SAVE` 才写入本目录的默认 YAML。已有文件会先保留带时间戳的备份。工具不修改 `config.yaml`、`scan_calibration.yaml`、扫描策略或安全工作空间配置。

可用 `--robot-ip` 指定 Receive 地址、`--output` 指定独立文件。已有四个完整 TCP pose 可离线导入：

```bash
python calibrate_workspace.py --points-file /path/to/measured_corners.yaml --output /path/to/workspace_calibration.yaml
```

JSON/YAML 支持 `P0`、`P1`、`P2`、`P3` 映射，每项为 `[x,y,z,rx,ry,rz]`，也支持按该顺序排列的四行六列数组。坐标单位为米，旋转向量为弧度。离线模式不连接设备，仍需检查预览并确认 `SAVE`。

`raw_points` 保留实测六维 TCP 位姿；`rectified_points` 是拟合矩形，不能替代原始测量。坐标转换用于独立显示/导出，不会变更正式扫描的控制坐标。

当前 Receive 接口不能读取 active TCP offset。工具只将配置中的 TCP 写为声明信息，明确记录 `active_tcp_verified: false`，不会额外建立控制接口进行查询。示教前应在示教器核对正确的探针 TCP，并在四点间保持一致。单次 pose/speed 读数不证明长期静止；箱体移动、TCP 改变或重新安装后需要重新采样。
