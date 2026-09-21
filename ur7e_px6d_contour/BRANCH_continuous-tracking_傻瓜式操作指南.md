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

应出现标题为 **SIMULATION / SYNTHETIC FORCE** 的窗口：左侧约 75% 为简洁运动图，右侧默认只有“Control feedback load（控制反馈载荷）”曲线，顶部状态为 `READY`，默认 `Target` 目标局部视角。此时尚未开始仿真，不创建真实控制器、不打开串口。没有图形桌面时改用下文的无窗口命令。

**4．摆放目标。** 先点 `Settings` 展开设置，再点 `1 Square`、`2 Circle` 或 `3 Triangle`；在左图目标内部按住鼠标左键，拖到新位置后松开。虚线是待放置轮廓，松开后才应用。点 `[ -15 deg` 或 `] +15 deg` 调整朝向。需要查看中心、尺寸、搜索线时打开 `Debug`；尺寸单位为 mm。

应看到目标改变，**P0、P1、Start 和搜索线保持原位（默认隐藏，Debug 中可见）**。非法位置会显示 `Rejected / stopped` 并撤销该次编辑，不会自动缩小或挪动目标。先用默认圆形、不拖动，可以直接观察首次接触。

**5．开始和停止。** 点 `Start`；左右图应同步推进。可在 `Settings` 中点 `5x` 加快预演。点 `Pause/Continue` 暂停，再点同一按钮继续。默认显示“仿真时间：xxx秒｜手动停止模式”，没有总运行时限；不会在 120 秒或绕完一圈时自动结束。点 `Stop`、按 `Q` / `Esc`、终端 `Ctrl+C` 或关闭窗口结束。保护触发仍会停止并显示具体代码/原因。

**6．找到结果。** 每次真正开始时，终端打印以 `Continuous preview run_dir:` 开头的一行，后面是本轮的完整目录。停止后点 `Save PNG`，应打印 PNG 的完整路径。保留这行路径；下一节说明如何打开。每次 `Reset` 后再次开始都会生成新目录，不覆盖旧日志。

## 调整表面摩擦系数 μ

主窗口底部右侧新增 **摩擦设置** 小按钮，旁边 `μ=...` 表示当前真正生效的值。新建默认连续场景是 **μ=0.20**；公共离散仿真配置仍为 0.03。显式加载已保存场景时使用文件中的 μ（例如原三角形复现场景仍为 0.03），不会强行改成 0.20。

1. 点击 **摩擦设置**，右侧下方展开紧凑输入区；再次点击收起。不会缩小 XY 绘图区。Debug 下打开它会暂时隐藏速度曲线，关闭后恢复。
2. 在“表面摩擦 μ（待应用）”中准确输入 `0.03`、`0.20` 等非负有限数值；也可点 `−` / `+`，每次调整 **0.01**。编辑和按 Enter 都不应用；底部当前 μ 保持原值。
3. 点击 **应用并重置本轮** 才生效。READY 时重建会话并继续等待；运行中或暂停时先结束、保存旧轮次，旧日志记录 `FRICTION_CHANGED` 及 μ 前后值，然后清空策略、滤波与接触记忆，回到 READY。**必须再点 Start** 才开始新一轮。
4. 目标形状、中心、朝向、起点、随机种子及其他环境参数保持不变。特别是 `granular_drag_force` 颗粒背景阻力不会跟着修改，本轮没有颗粒参数按钮。
5. 空输入、负值、NaN、无穷或非数值会在输入区显示提示，原场景/运行不变。保存场景只保存真正生效的 μ，不保存未应用的草稿；加载会同步输入框和当前值。很长的数值在底部用 `≈` 简写，输入框保留完整可往返数值。

μ 直接进入既有合成表面摩擦公式，改变 raw/processed 控制反馈和物理诊断分量，不是只倾斜箭头。预演和 `--dry-run --scene <保存文件>` 使用相同参数、同一策略。仿真 μ 不代表已经标定的真实摩擦系数，真实设备参数与导纳公式没有修改。

**已发现的模型限制：** 新默认 μ=0.20 的 180 秒圆形、方形及旋转方形闭环中，独立评分检测到约 **0.0322–0.0328 mm 中心穿透**，压缩约 **1.0322–1.0328 mm**，超过模型 1 mm 接触包络。因此这三项几何验收失败，虽然最终代码是 STOP_TIME_LIMIT，也不能称为几何跟踪成功。本轮按要求没有调控制律、速度或安全阈值来消除失败；完整失败日志保留，详见当前修订报告。可显式输入历史 μ=0.03 对照，但默认值不会自动回退。

## 预演不限时与有限测试

`--preview` 会在本次配置副本中把 `continuous_tracking.max_runtime_sec` 设为 Python `None`，日志快照写作 YAML `null`。不写回 config.yaml，不采用巨大秒数，也不修改真机预算或现场验证。共享策略只跳过总时限比较；力/力矩、无效/过期数据、方向确认、低力/丢边、局部搜索、workspace 和其他保护仍生效。没有增加绕圈完成自动退出。

需要有限预演，例如自动测试或固定截点演示：

```bash
../.venv312/bin/python run_continuous_tracking.py --preview --duration 180 --scene simulation/scene_direction_triangle.yaml --output simulation_outputs/continuous_preview
```

时长必须有限且大于零；到期显示 STOP_TIME_LIMIT。自动测试必须给出有限时长、有限步数或注入明确停止操作，不能等待手动模式自行结束。无窗口批量运行仍用下文带 `--duration` 的命令。真机入口拒绝 null / None 总预算，且保留原配置绑定和现场检查。

长时间预演只保留最近 30 秒采样、最多 4096 个显示路径点及 200 个显示事件；完整采样持续写入 CSV，不累积全部历史帧，不自动编码视频。停止后照常保存完整日志和停止原因；磁盘写入失败按异常流程停止和关闭日志。

底部说明统一为随窗口宽度换行的独立文本区；展开 Settings / Debug 时自动预留高度，不再将几何、事件与说明堆在同一位置。手动缩放不因设置切换重置。

## 窗口具体怎么用

| 操作 | 按钮 / 快捷键 | 结果 |
|---|---|---|
| 设置区域 | `Settings` | 展开/收起形状、旋转、倍速、场景文件、Global、Follow、Components；主画面常用按钮始终可用 |
| 表面摩擦 | `摩擦设置`，旁边当前 μ | 编辑值与生效值分开；应用并重置后需重新 Start；不改颗粒背景阻力 |
| 调试显示 | `Debug` | 默认关闭；开启后显示共享箭头图例、估计方向、命令/实际速度及 Fx/Fy/误差；轨迹仍为深灰细线 |
| 三种形状 | `1 Square` / `2 Circle` / `3 Triangle`，或 `1` / `2` / `3` | 方形使用场景已有 width 作为边长；圆为半径，等边三角形为外接圆半径；尺寸在 Debug 中显示 |
| 移动目标 | 目标内部左键拖动、松开 | 只改中心；拖动期间冻结播放，松开后校验；移出容器或包住起点会拒绝 |
| 朝向 | `[ -15 deg` / `] +15 deg`，或 `[` / `]` | 每次旋转 15°；圆的外观不受朝向影响 |
| 开始 | `Start` 或 `Enter` | 从 READY 开始；ENDED 后先 Reset；输入框有焦点时普通快捷键用于输入文本；Q / Esc 始终停止 |
| 暂停 / 继续 | `Pause/Continue` 或空格 | 同一场景保留策略、滤波、接触记忆；仿真时钟冻结，不累计墙钟等待 |
| 播放速度 | `1x` / `5x` / `10x` | 只改变固定仿真步的推进频率；默认 dt=0.01 s，控制参数不变 |
| 重置 | `Reset` 或 `r` | 结束并保留旧轮次，清空历史，原场景回到 READY |
| 停止 | `Stop` 或 `q` / `Esc` | 结束本轮并记录停止原因；要再开始须重置 |
| 关闭 | 窗口关闭按钮 | 当前轮次记录 WINDOW_CLOSED / 用户停止；不代表轮廓完成 |
| 全局 / 目标局部 / 探针附近 | `Global` / `Target` / `Probe` | 全局看容器；目标局部放大目标；探针附近看接触细节；始终 XY 等比例 |
| 自动跟随 | `Follow` | 开启后跟随探针；再点关闭。滚轮或工具栏手动缩放平移会关闭跟随 |
| 手动缩放、平移 | 左图滚轮；窗口工具栏的缩放、平移按钮 | 暂停或停止后仍可用；启用工具栏平移时不会误拖目标；刷新不覆盖手动视窗 |
| 真实切向对照 | Debug + Settings 中“真实切向对照（仅仿真）” | 默认关闭；短灰虚线与策略绿色切向独立显示，只用于诊断 |
| 合成力分量 | `Components` | 位于 Settings；开启时同时打开 Debug，显示 RAW 合成分量；只用于诊断，不传给策略 |
| 静态图片 | 结束后 `Save PNG` | 保存本轮同步视图 `continuous_preview.png`；重复保存使用新的文件名 |

键盘操作前点一下绘图区空白处，退出 `Scene YAML` 文本输入焦点。Stop、Q / Esc 始终可停止，输入路径时也不要按 Q 作为普通字符；终端 `Ctrl+C` 会结束预演并收尾日志。重复 Stop 或关窗安全无副作用。

**运行中或暂停后编辑场景**：有效形状、位置、旋转或加载操作会结束旧一轮，记录 `SCENE_CHANGED`，保留旧日志，然后用新的策略、滤波、时钟和接触记忆回到 READY。不能拖动物体后接着沿用旧接触。无效编辑被拒绝；拖动被取消或拒绝后原轮次保持暂停。

**界面不会保证沿边走完。** 默认恢复关闭，丢边即停。搜索线没有经过目标时，按搜索预算失败是有效结果。不要为了动画跑完调大安全阈值。若显示 `Playback throttled`，实际倍速低于请求值；每个传感器／控制／执行步仍完整计算。默认刷新 10 fps。

左侧默认只画灰色目标、小实心圆点 TCP、单色已执行轨迹及最多两根非零力箭头。圆点只是位置符号，不改变模型探针半径；Debug 也不叠加十字事件、P0/P1、半径包络或状态色轨迹；事件和模型尺寸仍可在调试文字与日志中查看。两根力箭头都从同一 TCP 位置出发，共用固定 **8 mm/N** 标尺和一个 1 N 标记，不归一化、不按每帧最大值缩放。0 N 隐藏箭头但数值显示 0.00 N；未知值标为 unavailable，不能当成零。

- **蓝色 法向反力 / Normal reaction**。只表示物体对探针的物理法向接触分量，读取日志 `sim_normal_physical_fx/fy`。方向是接触模型的物理外法向：圆形沿圆心向外，直边垂直该边，尖角采用模型的有效法向，不假设唯一表面法向。长度为实际法向载荷，不是包含摩擦的边界合力模长，也不绑定 F_ref。无接触时蓝箭头为零，即使仍有颗粒阻力或测量噪声。默认不另画摩擦箭头，底部注明“法向反力示意；摩擦分量未绘制”。
- **红色虚线：机器人作用力估计（准静态）**。按 `-(物理边界力 + 颗粒背景阻力 + 其他已知平面外力)` 计算；本模型没有其他平面外力项，measurement noise 不参与平衡。表示夹具传给探针的等效平面合力，画在 TCP 只是示意，不代表真实施力点或杆身弯曲/力矩计算。不是速度命令、力控输出或独立测量。

旧模型的控制合成信号依旧为 `object_force（向内）+ friction + background + noise`。复用已有诊断：`normal_physical = -object_force`，`boundary_physical = normal_physical + friction`，`robot_estimate = -(boundary_physical + background)`。蓝色只取 normal_physical；红色仍包含完整摩擦和背景阻力，不能简单取蓝色反向。raw/processed、零偏、变换、force_direction_sign、摩擦参数和策略完全保留。两套力的不同含义记录在配置快照的 `force_display`（schema_version、force_convention、force_source、estimate_method），CSV 另记物理分量及可用性。旧仿真日志若缺少可靠法向分量，蓝色显示 unavailable，不从合力猜测法向。

理想直边匀速、接触压缩和阻力不变时，两箭头可以基本稳定；只有摩擦及背景阻力均可忽略时才可近似等大反向。这是数据和准静态假设的结果，不是人为抖动或定长。接触建立、加减速、急转或停止等阶段显示 `Transient: estimate approximate`；即使未触发提示，也始终注明忽略惯性，不能视为动态驱动力重建。两箭头也不能证明跟踪成功，仍须独立检查载荷误差、前进、穿透和停止原因。

右侧默认只显示实际进入策略的 processed **Fxy** 与 **F_ref**，标题为控制反馈载荷，不是纯边界载荷。Debug 中才展开 Fx/Fy、误差和独立的 mm/s 速度面板；辅助速度仍采用另一套标尺，不标成 N。所有显示取同一时间戳记录，不额外读传感器。曲线保留最近 30 秒及极值，预演执行轨迹最多 4096 点；完整数据保留在 CSV。

**真机回放**的蓝标签自动改为 `Measured environment resultant（测得环境合力）`，不能声称分离了目标与颗粒。只有独立记录确认了 Base 坐标、环境对探针的物理正负定义和标定来源，才可显示 processed 合力及其反号的准静态估计；噪声和未知外力的不确定性保留。现有现场 `force_sign_checked` 是控制方向核对，不能自动等同这项物理确认，因此新真机日志默认明确记录 unconfirmed，隐藏两箭头。可信离线记录的 schema 需要 `frame=Base`、`force_convention=environment_on_probe`、`force_source=processed_wrench`、两项 physical_sign_confirmed/base_frame_confirmed、calibration_reference；这不是新的上机放行方式。缺少元数据的旧日志仍能打开，但物理箭头显示不可用，不猜符号。

### Debug 图例和真实切向对照

点 **Debug** 后，左图上方出现独立预留的图例区，箭头、轨迹、右侧曲线及分量说明都复用 `simulation/continuous_view.py` 的样式定义。关闭 Debug 后恢复简洁画面；Components 关闭后，其图例和箭头一起消失。图例只在启用项目集合变化时重建，普通刷新更新数值和状态；零值或不可用项会标注“零 / 不可用”，不画虚假箭头。

| 样式 | 含义与单位 |
|---|---|
| 蓝色实线箭头 | 法向反力 [N]；真机回放为测得环境合力，沿用原物理符号可信检查 |
| 红色虚线箭头 | 机器人作用力估计 [N] |
| 橙色实线箭头 | 处理后力信号 [N]；仿真注明“仿真合成”，不是物理反力；真机注明 processed Base |
| 绿色箭头 | 策略输出的估计切向 [单位方向] |
| 青色箭头 | 策略输出的估计朝内方向 [单位方向] |
| 黑色实线箭头 | 命令速度 [mm/s] |
| 紫色实线箭头 | 实际 TCP 速度 [mm/s] |
| 深灰色无箭头细线 | 已执行轨迹 [mm] |
| 短灰色虚线段（可选） | 真实切向对照（仅仿真），不是运动箭头 |

力共用 **8 mm/N** 标尺，速度使用独立 **12 mm/(mm/s)** 标尺；单位方向只用固定 8 mm 示意长度，不把方向长度当作力或速度。Components 的四种分量配色见下文，均采用同一 N 标尺。

**预演操作**：先点 Debug，再展开 Settings，右侧点“真实切向对照（仅仿真）：关”切换为开。默认关闭。灰色虚线只在已有日志的物理法向非零、有限且来源可信时显示，由接触模型有效法向转 90° 得到；尖角也用模型有效法向。绿色箭头始终采用策略 tangent，即使偏离边界也不修正。底部显示无方向直线夹角 **0～90°**，正反切向同为 0°，因此不能用它判断倒退。

**回放操作**：Debug 打开后，底部可切换 Components 和“真实切向对照（仅仿真）”；导出时可加 `--debug --components --true-tangent`。真机回放隐藏真实切向按钮并注明不可用；旧仿真日志若没有可靠法向也不猜测。所有开关只读取当前样本，不推进仿真、不采新噪声、不改变反馈或控制。

### 看到 DIRECTION_RECONFIRM 时怎么办

这是“方向暂停重确认”，不是结束，也不是自动判断了某种角。程序先随测量转速、估计滞后和可信度降低切向速度，必要时请求零运动，等实测 TCP 连续停稳、新接触方向连续稳定后再建立方向。确认当帧仍停止，之后低速恢复并验证一小段前进；Base、force_direction_sign 和 follow_hand 不会自动翻转。

仍需看具体原因：`STOP_DIRECTION_UNCONFIRMED` 表示方向确认超时；`STOP_DIRECTION_REVERSAL` 表示接近反向、符号无法解释；`STOP_DIRECTION_NO_PROGRESS` 表示重复确认或恢复运动后缺少前进；`STOP_STALE_DATA` 表示数据/周期过期。力硬限仍是 `STOP_FORCE_LIMIT`，持续丢边仍按原 CONTACT_LOST 和默认关闭的恢复设置处理。安全或人工 STOP 不会自动恢复。出现 STOP 先看本轮 `termination.json`，不要盲目改阈值。

`Components` 显示的土黄色点线是**模型法向合成分量（仿真约定，采用向内符号）**，棕色虚线是表面摩擦、灰蓝色点划线是颗粒背景、粉色点线是噪声。四者之和等于未处理 raw wrench 的 XY 分量；processed 还经过零偏、滤波和坐标处理，不能要求它等于这四个原始分量之和。它们不是实测分离出的真实边界反力。真实 PX6D 的这些分量和未知目标轮廓显示 unavailable，不伪造。

## 保存 / 加载独立场景

打开 `Settings` 后，`Scene YAML` 框默认给出 `simulation_scenes/continuous_日期时间.yaml` 的独立路径。

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

终端打印 `Continuous run:` 后的完整目录。12 秒是缩短总预算，默认圆形通常已进入首次贴边；停止原因为 `STOP_TIME_LIMIT`，不是一圈完成。无窗口与真机的 `--duration` 仍只能缩短已有有限预算；默认无窗口仍保留配置的 120 秒。

若要查看刚做的无窗口这一轮，先再次执行前面的 `read` 命令，把新的 `Continuous run:` 路径赋给 `CONTINUOUS_RUN_DIR`；否则变量仍指向原预演目录。然后**显式**生成静态图：

```bash
../.venv312/bin/python tools/visualize_continuous_run.py "$CONTINUOUS_RUN_DIR" --format none
xdg-open "$CONTINUOUS_RUN_DIR/continuous_summary.png"
```

要视频时再执行，默认运行不会自动编码视频：

```bash
../.venv312/bin/python tools/visualize_continuous_run.py "$CONTINUOUS_RUN_DIR" --fps 10 --format auto
```

终端打印实际产物路径；有 ffmpeg 时尝试 MP4，没有时回退 GIF，帧数受工具预算限制。需要看目标局部用 `--view target`、探针附近用 `--view probe`，全局用 `--view global`；`--local-xy` 仍按已记录轨迹裁剪。真实日志没有目标时 `target` 视角按轨迹取景。默认仅导出 PNG。需要调试图时加 `--debug`；需要记录中的旧合成分量时加 `--components`（同时启用调试），不可用时明确标记 unavailable。回放属于已有数据展示，不能用来证明闭环控制成功。**本轮结束、失败停止、通过独立验收是三件不同的事**；退出码 0、ENDED、关窗、保存成功都不表示完成轮廓。

### 复查这次三角形及首次转折验证

原失败记录仍保存在 `simulation_outputs/continuous_preview/run_20260920_235436_498310/`。以下历史三角形场景显式保留 μ=0.03，不会被新默认 0.20 覆盖。其独立场景已保存为 `simulation/scene_direction_triangle.yaml`，中心和旋转来自原 config_snapshot。打开该场景：

```bash
../.venv312/bin/python run_continuous_tracking.py --preview --scene simulation/scene_direction_triangle.yaml --output simulation_outputs/direction_preview
```

默认预演不限总时长，可观察原失败位置后继续运行，再手动停止。要复现旧版 120 秒截点，明确加 `--duration 120`。无 GUI 时把 `--preview` 改为 `--dry-run`，并为批量任务明确设置有限时长。原停在 102.02 秒，诊断为估计滞后 46.34°，相邻测量仅变化 2.84°；不是已发现的中心穿透。

需要重跑圆形、方形、三角形、平移旋转和无目标的独立几何验证，用这个纯离线命令：

```bash
../.venv312/bin/python -m simulation.continuous_validation --directions --output simulation_outputs/direction_revision
```

它仅把离线验证时长延至 180 秒，不提高速度或放宽硬保护；预演的 `--duration` 是显式有限仿真时长，可以超过 120 秒；其他入口仍保留原有限预算规则。终端打印各轮目录和汇总 `direction_reports.json` 路径，每轮另有 `direction_geometry_report.json`，报告首次转折后的净前进、载荷、暂停/恢复、中心及线段穿透、异常压缩和真实终止原因。当前三角形已通过原处转折，180 秒内转折后净前进约 65.8 mm；**不是整圈完成或真机验证**。报告中任何穿透都算失败，不能以动画看起来连续为准。

用实际本轮目录显式导出新回放（先按前面的 read 命令设置目录变量）：

```bash
../.venv312/bin/python tools/visualize_continuous_run.py "$CONTINUOUS_RUN_DIR" --format none --view target
```

会打印 `continuous_summary_target.png` 路径。默认双力箭头和 Fxy/F_ref；`--debug` 才显示左侧箭头图例并分区显示 N 和 mm/s、另列原始力，仿真 raw 按模型 Base 坐标标注，真机 raw 按传感器坐标标注，processed 才是处理后的 Base 力。未知真机目标和分离力保持 unavailable。

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

核验证据范围：前一轮双力显示全量回归 **740 passed in 179.44s**；本轮不限时预演及底部布局的验收见当前修订报告。结果与截图见 [当前修订报告](CONTINUOUS_TRACKING.md)。修改前后原三角形 12,001 个周期的 raw/processed、TCP、状态和命令精确一致，显示开关不推进 RNG。无硬件 CLI、固定步长闭环一致性和可调用按钮/拖动回调已自动测试，静态 PNG 已实际查看；预演、无窗口及真机入口设备替身在同一观测流上逐条比对，真实控制主循环没有加入绘图。窗口缩放保持 XY 等比例。**未手动点击真实桌面窗口，未验证真机物理行为。**

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
