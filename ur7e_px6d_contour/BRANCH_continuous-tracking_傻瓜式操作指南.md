# 【分支 experiment/continuous-tracking】连续贴边：傻瓜式操作指南

本指南只适用于 `experiment/continuous-tracking` 连续贴边实验分支。`main` 保留原离散探测路线；这里不把它称为已通过真机验证的稳定版。本地仍是同一个项目文件夹，切换分支会切换受 Git 管理的代码。原来的《从这里开始》《操作文档》用于原路线，不要混用入口。

## 先打开纯仿真预演：最短路线

**1．打开终端并进入项目。** 在 Ubuntu 按 `Ctrl+Alt+T`，复制：

```bash
cd /home/user-linux/robotics_cs/ur7e_px6d_contour
pwd
git rev-parse --show-toplevel
git branch --show-current
git status
```

应看到项目路径 `/home/user-linux/robotics_cs/ur7e_px6d_contour`、仓库根目录 `/home/user-linux/robotics_cs`、分支 `experiment/continuous-tracking`。Git 可以在仓库内的子目录运行；下文 Python 命令都在上述项目目录运行。

若路径不存在，先在文件管理器核对实际项目位置。若分支不符，先停在这里，核对负责人指定的分支，**不要强制切换**。若 `git status` 显示修改或未跟踪文件，先用 `git diff` 和文件清单辨认、保留已有工作；其中可能就是前一轮修复。确认这些是要预演的修改后继续，不要 reset、stash 或删除它们。切分支前先结束正在运行的实验，再处理未提交改动。

**2．检查已有 Python 环境。** 本项目可用环境位于 `/home/user-linux/robotics_cs/.venv312`，不是项目子目录内的 `.venv312`。复制：

```bash
../.venv312/bin/python --version
../.venv312/bin/python -c "import sys, numpy, yaml, matplotlib; print(sys.executable); print('环境导入成功')"
```

应看到 Python 3.12、上述环境的 Python 路径和“环境导入成功”。这样直接指定解释器即可，不必先激活或重装全部依赖；缺失时先核对环境位置。

**3．打开窗口。** 复制这一条纯离线命令：

```bash
../.venv312/bin/python run_continuous_tracking.py --preview --output simulation_outputs/continuous_preview
```

应出现标题为 **SIMULATION / SYNTHETIC FORCE** 的窗口：左侧约 70% 为运动图，右侧为分开的力、速度面板，顶部状态为 `READY`，默认 `Target` 目标局部视角。此时尚未开始仿真，不创建真实控制器、不打开串口。没有图形桌面时改用下文的无窗口命令。

**4．摆放目标。** 点 `1 Square`、`2 Circle` 或 `3 Triangle`；在左图目标内部按住鼠标左键，拖到新位置后松开。虚线是待放置轮廓，松开后才应用。点 `[ -15 deg` 或 `] +15 deg` 调整朝向。下面显示中心、尺寸和旋转角；尺寸单位为 mm。

应看到目标改变，**P0、P1、Start 和搜索线保持原位**。非法位置会显示 `Rejected / stopped` 并撤销该次编辑，不会自动缩小或挪动目标。先用默认圆形、不拖动，可以直接观察首次接触。

**5．开始和停止。** 点 `Start`；左右图应同步推进。可点 `5x` 加快预演。点 `Pause/Continue` 暂停，再点同一按钮继续。点 `Stop` 结束这一轮。正常预算结束也会显示 `ENDED` 及具体停止原因。

**6．找到结果。** 每次真正开始时，终端打印以 `Continuous preview run_dir:` 开头的一行，后面是本轮的完整目录。停止后点 `Save PNG`，应打印 PNG 的完整路径。保留这行路径；下一节说明如何打开。每次 `Reset` 后再次开始都会生成新目录，不覆盖旧日志。

## 窗口具体怎么用

| 操作 | 按钮 / 快捷键 | 结果 |
|---|---|---|
| 三种形状 | `1 Square` / `2 Circle` / `3 Triangle`，或 `1` / `2` / `3` | 方形使用场景已有 width 作为边长；圆为半径，等边三角形为外接圆半径；尺寸显示在左图下方 |
| 移动目标 | 目标内部左键拖动、松开 | 只改中心；拖动期间冻结播放，松开后校验；移出容器或包住起点会拒绝 |
| 朝向 | `[ -15 deg` / `] +15 deg`，或 `[` / `]` | 每次旋转 15°；圆的外观不受朝向影响 |
| 开始 | `Start` 或 `Enter` | 从 READY 开始；ENDED 后先 Reset；输入框有焦点时快捷键用于输入文本 |
| 暂停 / 继续 | `Pause/Continue` 或空格 | 同一场景保留策略、滤波、接触记忆；仿真时钟冻结，不累计墙钟等待 |
| 播放速度 | `1x` / `5x` / `10x` | 只改变固定仿真步的推进频率；默认 dt=0.01 s，控制参数不变 |
| 重置 | `Reset` 或 `r` | 结束并保留旧轮次，清空历史，原场景回到 READY |
| 停止 | `Stop` 或 `q` / `Esc` | 结束本轮并记录停止原因；要再开始须重置 |
| 关闭 | 窗口关闭按钮 | 当前轮次记录 WINDOW_CLOSED / 用户停止；不代表轮廓完成 |
| 全局 / 目标局部 / 探针附近 | `Global` / `Target` / `Probe` | 全局看容器；目标局部放大目标；探针附近看接触细节；始终 XY 等比例 |
| 自动跟随 | `Follow` | 开启后跟随探针；再点关闭。顶部显示 follow=True/False；滚轮或工具栏手动缩放平移会关闭跟随 |
| 手动缩放、平移 | 左图滚轮；窗口工具栏的缩放、平移按钮 | 暂停或停止后仍可用；启用工具栏平移时不会误拖目标；刷新不覆盖手动视窗 |
| 合成力分量 | `Components` | 切换 RAW 合成分量箭头，颜色见窗口说明；只用于仿真诊断，不传给策略 |
| 静态图片 | 结束后 `Save PNG` | 保存本轮同步视图 `continuous_preview.png`；重复保存使用新的文件名 |

键盘操作前点一下绘图区空白处，退出 `Scene YAML` 文本输入焦点。停止按钮始终可直接点击。终端 `Ctrl+C` 也会结束预演并收尾日志。

**运行中或暂停后编辑场景**：有效形状、位置、旋转或加载操作会结束旧一轮，记录 `SCENE_CHANGED`，保留旧日志，然后用新的策略、滤波、时钟和接触记忆回到 READY。不能拖动物体后接着沿用旧接触。无效编辑被拒绝；拖动被取消或拒绝后原轮次保持暂停。

**界面不会保证沿边走完。** 默认恢复关闭，丢边即停。搜索线没有经过目标时，按搜索预算失败是有效结果。不要为了动画跑完调大安全阈值。若显示 `Playback throttled`，实际倍速低于请求值；每个传感器／控制／执行步仍完整计算。默认刷新 10 fps。

左侧 XY 坐标严格等比例，灰色是已知合成目标；探针圆使用合成模型的实际半径，醒目的 `+` 只是放大的 TCP 中心标记。路径分为 SEARCH、RELIABLE、UNCERTAIN、RECOVERY，表示已执行运动，不是重建出的真实轮廓。蓝色是 processed 测得合力，固定 **8 mm/N**；黑色是命令速度、紫色是实测 TCP 速度，固定 **12 mm 显示长度 / (mm/s)**，左图分别显示 1 N、1 mm/s 标尺。箭头不再归一化成等长，零命令不画箭头。绿/红只是有效切向/sign 修正向内估计的 8 mm 方向辅助箭头；暂停确认或方向无效时隐藏。速度单位是 mm/s，**不是机械臂驱动力 N**。

右上 Fx、Fy、Fxy、F_ref、F_error 单位 N；右下命令速度大小、实际 TCP 速度大小、命令切向 v_t / 法向 v_n 单位 mm/s。命令和实际速度向量在左图用不同颜色显示。三处使用同一仿真时间，只显示已经算出的数据。F_error 是目标载荷减测得力幅值，不能当作额外计算出的驱动力。默认 F_ref=1.5 N。力曲线保留最近 30 秒，降采样保留力极值；左图保留本轮全程的降采样路径，最多 4096 个点，合并跨状态段按 UNCERTAIN 显示。完整采样在 CSV 中。这里的 1 mm 探针接触包络、弹性接触、噪声和简单阻力都是合成假设，不是现场探针规格或颗粒物理验证。

### 看到 DIRECTION_RECONFIRM 时怎么办

这是“方向暂停重确认”，不是结束，也不是自动判断了某种角。程序先随测量转速、估计滞后和可信度降低切向速度，必要时请求零运动，等实测 TCP 连续停稳、新接触方向连续稳定后再建立方向。确认当帧仍停止，之后低速恢复并验证一小段前进；Base、force_direction_sign 和 follow_hand 不会自动翻转。

仍需看具体原因：`STOP_DIRECTION_UNCONFIRMED` 表示方向确认超时；`STOP_DIRECTION_REVERSAL` 表示接近反向、符号无法解释；`STOP_DIRECTION_NO_PROGRESS` 表示重复确认或恢复运动后缺少前进；`STOP_STALE_DATA` 表示数据/周期过期。力硬限仍是 `STOP_FORCE_LIMIT`，持续丢边仍按原 CONTACT_LOST 和默认关闭的恢复设置处理。安全或人工 STOP 不会自动恢复。出现 STOP 先看本轮 `termination.json`，不要盲目改阈值。

`Components` 显示的橙色是**模型目标作用分量（仿真约定，采用向内符号）**，棕色表面摩擦、灰色背景阻力、粉色噪声。四者之和等于未处理 raw wrench 的 XY 分量；processed 还经过零偏、滤波和坐标处理，不能要求它等于这四个原始分量之和。它们不是实测分离出的真实边界反力。真实 PX6D 的这些分量和未知目标轮廓显示 unavailable，不伪造。

## 保存 / 加载独立场景

窗口底部 `Scene YAML` 框默认给出 `simulation_scenes/continuous_日期时间.yaml` 的独立路径。

1. 摆好目标后点 `Save scene`；成功后界面和终端显示完整保存路径。只写该独立 YAML，不写 `config.yaml` 或任何现场标定文件。
2. 同名文件已存在时拒绝覆盖。在框中改一个新文件名，再保存。
3. 加载时将该完整路径粘贴到框中，点 `Load scene`。运行中的旧轮次会结束，新场景待开始。

文件包含容器、目标尺寸/中心/朝向、P0/P1/Start、合成力参数与随机种子等。需改尺寸时可在**独立场景副本**中编辑 `target_width` / `target_height` / `target_radius`（文件单位 m），再加载校验。P0/P1 必须有限且不同，目标全部在容器内，起点在目标及探针接触包络外。连续控制参数来自入口加载的配置，不因选图形或倍速改变；场景里的旧离散策略字段不会启动旧策略。

## 查看日志、无窗口模拟和显式回放

按快速开始命令运行时，结果在 `simulation_outputs/continuous_preview/run_日期时间微秒/`。若省略 `--output`，当前配置默认在项目 `data/`。**以终端的本轮 run_dir 行为准**，不要误把上一轮当作本轮。

停止后，在终端复制下面两条；提示输入时，粘贴本轮 `run_dir:` 后面的完整目录，不带引号：

```bash
read -r -p '粘贴本轮完整 run_dir 路径后按回车：' CONTINUOUS_RUN_DIR
xdg-open "$CONTINUOUS_RUN_DIR"
```

应打开该目录。双击 `continuous_preview.png` 看图；若尚未保存 PNG，可用下面静态回放命令生成 `continuous_summary.png`。找不到终端那行时，先列出最近目录再选择正确一轮：

```bash
ls -dt simulation_outputs/continuous_preview/run_*
```

主要文件：`samples.csv` / `full_log.csv` 是完整逐步数据；`policy_waypoints.csv` 是事件；`config_snapshot.yaml` 包含实际连续控制参数、场景、随机种子、模拟执行延迟/加速度和 Git 分支/提交/dirty 状态；`summary.json` / `termination.json` 记录停止原因；`scan_stop_snapshot.json` 记录最终停止观测。Git dirty 表示代码仍有未提交修改，不能只凭提交号重现所有本地内容。

**纯无窗口模拟**（不会弹窗，不会连接设备）：

```bash
../.venv312/bin/python run_continuous_tracking.py --dry-run --duration 12 --output simulation_outputs/continuous_preview
```

终端打印 `Continuous run:` 后的完整目录。12 秒是缩短总预算，默认圆形通常已进入首次贴边；停止原因为 `STOP_TIME_LIMIT`，不是一圈完成。`--duration` 只能缩短配置预算。

若要查看刚做的无窗口这一轮，先再次执行前面的 `read` 命令，把新的 `Continuous run:` 路径赋给 `CONTINUOUS_RUN_DIR`；否则变量仍指向原预演目录。然后**显式**生成静态图：

```bash
../.venv312/bin/python tools/visualize_continuous_run.py "$CONTINUOUS_RUN_DIR" --format none
xdg-open "$CONTINUOUS_RUN_DIR/continuous_summary.png"
```

要视频时再执行，默认运行不会自动编码视频：

```bash
../.venv312/bin/python tools/visualize_continuous_run.py "$CONTINUOUS_RUN_DIR" --fps 10 --format auto
```

终端打印实际产物路径；有 ffmpeg 时尝试 MP4，没有时回退 GIF，帧数受工具预算限制。需要看目标局部用 `--view target`、探针附近用 `--view probe`，全局用 `--view global`；`--local-xy` 仍按已记录轨迹裁剪。真实日志没有目标时 `target` 视角按轨迹取景。需要记录中的合成分量时加 `--components`，不可用时明确标记 unavailable。回放属于已有数据展示，不能用来证明闭环控制成功。**本轮结束、失败停止、通过独立验收是三件不同的事**；退出码 0、ENDED、关窗、保存成功都不表示完成轮廓。

### 复查这次三角形及首次转折验证

原失败记录仍保存在 `simulation_outputs/continuous_preview/run_20260920_235436_498310/`。其独立场景已保存为 `simulation/scene_direction_triangle.yaml`，中心和旋转来自原 config_snapshot。打开该场景：

```bash
../.venv312/bin/python run_continuous_tracking.py --preview --scene simulation/scene_direction_triangle.yaml --output simulation_outputs/direction_preview
```

保持默认 120 秒预算即可观察本次原失败位置附近的方向重确认。无 GUI 时把 `--preview` 改为 `--dry-run`。原停在 102.02 秒，诊断为估计滞后 46.34°，相邻测量仅变化 2.84°；不是已发现的中心穿透。

需要重跑圆形、方形、三角形、平移旋转和无目标的独立几何验证，用这个纯离线命令：

```bash
../.venv312/bin/python -m simulation.continuous_validation --directions --output simulation_outputs/direction_revision
```

它仅把离线验证时长延至 180 秒，不提高速度或放宽硬保护；主入口 `--duration` 仍只能缩短预算。终端打印各轮目录和汇总 `direction_reports.json` 路径，每轮另有 `direction_geometry_report.json`，报告首次转折后的净前进、载荷、暂停/恢复、中心及线段穿透、异常压缩和真实终止原因。当前三角形已通过原处转折，180 秒内转折后净前进约 65.8 mm；**不是整圈完成或真机验证**。报告中任何穿透都算失败，不能以动画看起来连续为准。

用实际本轮目录显式导出新回放（先按前面的 read 命令设置目录变量）：

```bash
../.venv312/bin/python tools/visualize_continuous_run.py "$CONTINUOUS_RUN_DIR" --format none --view target --components
```

会打印 `continuous_summary_target.png` 路径。右侧分区显示 N 和 mm/s；原始力另列，仿真 raw 按模型 Base 坐标标注，真机 raw 按传感器坐标标注，processed 才是处理后的 Base 力。未知真机目标和分离力保持 unavailable。

## 真机章节：与上述纯仿真分开

本次开发和文档核验没有连接机器人或传感器，没有执行真机命令。预演不是运行许可。真机相关流程只由具备现场条件的操作人员实施，详细约束见 [连续跟踪修复说明](CONTINUOUS_TRACKING.md)。

开始前必须现场核对：

- 实际安装工具与 TCP offset；标定文件的机器人 IP、P0/P1、固定 Z/姿态与当前设备对应。
- PX6D 到 Base 的旋转/变换及 `force_direction_sign`，实际空载零偏；不能从合成模型的正负号推断现场方向。
- 已明确且有限的局部 XY 允许区域、边界停止余量、低速限制、力/力矩阈值。
- 急停、停止指令及实际停稳、时序超时与机器人侧 watchdog 已在现场核实。代码检查 SDK 合同和配置，不能代替现场停止距离验证。共享策略的新方向重确认参数也必须现场验证；配置摘要包含这些参数，旧绑定记录不自动放行新配置。
- `site_verification` 的操作员、时间、核对项与当前配置/标定绑定记录真实有效。默认记录为空会在设备构造前拒绝；修改工具、变换或配置后需重新核对。默认关闭恢复，真机丢边即停，不能用离线 `--enable-reacquire` 放行真机。

下面是**会连接设备、确认后可能运动**的单独命令示例，仅供完成上述现场工作后识别入口；本次没有运行它：

```bash
../.venv312/bin/python run_continuous_tracking.py --execute
```

该入口要求当前 TCP 已位于标定 P0 且静止，并在核对后输入 `START`；不要把 START 当作绕过前置检查的开关。`--preview` 与 `--execute` 互斥。

**本连续入口的 Q / ESC / Ctrl+C 是原地请求停止，不自动回 P0、不执行旧离散入口的自动返回。** 停止请求与实际停稳分别记录。停稳观测失败或设备/时序异常时不能把日志里的停止请求当作已停稳；按现场处置程序人工检查。原离散文档的返回流程不适用于此入口。

## 常见问题：先做一个明确动作

| 现象 | 下一步 |
|---|---|
| `cd` 或 Python 路径不存在 | 在文件管理器核对项目与 `/home/user-linux/robotics_cs/.venv312` 的实际位置 |
| 分支不对或改动来源不明 | 停止实验，查看 `git status` / `git diff` 并保留已有工作，先确认分支和修改归属 |
| 无 GUI / `No interactive Matplotlib backend` | 使用上面的 `--dry-run` 与静态回放；要交互窗口则回到有可用 Matplotlib 图形后端的桌面终端 |
| `Rejected / stopped` / 非法场景 | 看窗口中的具体字段；将目标放回容器内、避开 Start 接触包络，再开始 |
| 找不到目标 / `STOP_SEARCH_LIMIT` | 看搜索线是否经过目标；仅修改仿真场景后重新开始 |
| `STOP_TIME_LIMIT` 或恢复预算耗尽 | 打开本轮 `termination.json` 看原因和已走路径，不据此认定轮廓完成 |
| 力、时序、边界等安全停止 | 查看 `termination.json` 与 `scan_stop_snapshot.json`，人工检查原因，不盲目增大阈值 |
| 真机前置检查拒绝 | 按错误项与《连续跟踪修复说明》核对现场记录或配置，保留拒绝，不跳过检查 |
| 快捷键没有反应 | 点绘图区空白处退出路径输入焦点，再操作；也可直接点 Stop |

核验证据范围：本轮最终全量回归 **730 passed in 171.40s**。无硬件 CLI、固定步长闭环一致性和可调用按钮/拖动回调已自动测试，静态 PNG 已离屏渲染检查；预演、无窗口及真机入口设备替身在同一观测流上逐条比对，真实控制主循环没有加入绘图。三种窗口尺寸的 XY 等比例也已离屏检查，完整结果与图像路径见 [CONTINUOUS_TRACKING.md](CONTINUOUS_TRACKING.md)。**未手动点击真实桌面窗口，未验证真机物理行为。**

## 需要保存代码时：由你检查后手动提交

以下仅为操作示例，Codex 本次没有执行 add、commit 或 push。先结束程序，在项目目录检查范围；仓库还含项目之外的文件，要核对完整清单：

```bash
git branch --show-current
git status --short
git diff --stat
git diff --check
git diff
```

确认要将前一轮控制修复与本次预演作为同一批分支工作保存后，逐项暂存下面代码/配置/测试/文档。若文件清单与现场不同，先核对；不要用 `git add .` 把数据或无关改动一起加入：

```bash
git add -- CONTINUOUS_TRACKING.md BRANCH_continuous-tracking_傻瓜式操作指南.md config.yaml run_continuous_tracking.py experiment_logging/data_logger.py policy/continuous_tracking.py robot/rtde_controller.py simulation/simulated_robot.py simulation/continuous_validation.py simulation/continuous_session.py simulation/continuous_preview.py simulation/continuous_view.py simulation/scene_direction_triangle.yaml simulation/simulated_force_sensor.py experiment_logging/termination.py tools/visualize_continuous_run.py tests/test_continuous_run.py tests/test_continuous_tracking.py tests/test_continuous_execution.py tests/test_continuous_geometry.py tests/test_continuous_revision.py tests/test_continuous_preview.py tests/test_continuous_direction.py
git diff --cached --name-only
git diff --cached --stat
git diff --cached
```

检查暂存区没有实验日志、独立场景记录、PNG、GIF、MP4 或无关文件，也没有漏掉依赖的前轮修复。确认之后才手动执行：

```bash
git commit -m "Fix continuous tracking guards and add offline interactive preview"
git push -u origin experiment/continuous-tracking
```

推送被拒绝时先看原因并与协作者核对，不强推、不 reset，也不删除实验数据。
