# 【分支 experiment/continuous-tracking】连续贴边：傻瓜式操作指南

本指南只适用于 `experiment/continuous-tracking` 连续贴边实验分支。`main` 保留原离散探测路线；这里不把它称为已通过真机验证的稳定版。本地仍是同一个项目文件夹，切换分支会切换受 Git 管理的代码。日常唯一主指南为 [从这里开始.md](从这里开始.md)，两种策略统一从 `python run_project.py` 选择。第 12 项仍为原离散，第 18 项才是连续真机；本文保留连续窗口细节。

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

**3．打开菜单并选择 15。**

```bash
source /home/user-linux/robotics_cs/.venv312/bin/activate
python run_project.py
```

输入 `15`，场景提示按回车使用默认场景；输入 `:q` 可取消。

应出现标题为 **SIMULATION / SYNTHETIC FORCE** 的窗口：左侧约 75% 为简洁运动图，右侧默认只有“Control feedback load（控制反馈载荷）”曲线，顶部状态为 `READY`，默认 `Target` 目标局部视角。此时尚未开始仿真，不创建真实控制器、不打开串口。没有图形桌面时选择菜单 16 的有限无窗口模拟。

**4．摆放目标。** 先点 `Settings` 展开设置，再点 `1 Square`、`2 Circle` 或 `3 Triangle`；在左图目标内部按住鼠标左键，拖到新位置后松开。虚线是待放置轮廓，松开后才应用。点 `[ -15 deg` 或 `] +15 deg` 调整朝向。需要查看中心、尺寸、搜索线时打开 `Debug`；尺寸单位为 mm。

应看到目标改变，**P0、P1、Start 和搜索线保持原位（默认隐藏，Debug 中可见）**。非法位置会显示 `Rejected / stopped` 并撤销该次编辑，不会自动缩小或挪动目标。先用默认圆形、不拖动，可以直接观察首次接触。

**5．开始和停止。** 点 `Start`；左右图应同步推进。可在 `Settings` 中点 `5x` 加快预演。点 `Pause/Continue` 暂停，再点同一按钮继续。默认显示“仿真时间：xxx秒｜手动停止模式”，没有总运行时限；不会在 120 秒或绕完一圈时自动结束。点 `Stop`、按 `Q` / `Esc`、终端 `Ctrl+C` 或关闭窗口结束。保护触发仍会停止并显示具体代码/原因。

**6．找到结果。** 每次真正开始时，终端打印以 `Continuous preview run_dir:` 开头的一行，后面是本轮的完整目录。停止后点 `Save PNG`，应打印 PNG 的完整路径。保留这行路径；下一节说明如何打开。每次 `Reset` 后再次开始都会生成新目录，不覆盖旧日志。

## 调整表面摩擦系数 μ

以下模型和控制修订报告中的“本轮/当前”指此前连续预演修订，不是本次菜单改造。参数以所选场景/实际配置为准；历史测试数量不作为本轮结果。

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

日常菜单 15 保持手动停止。自动测试或固定截点演示需要有限预演，见高级调试附录；日常无窗口验证用菜单 16：

（直接脚本示例见文末“高级调试”；日常从菜单选择。）

时长必须有限且大于零；到期显示 STOP_TIME_LIMIT。自动测试必须给出有限时长、有限步数或注入明确停止操作，不能等待手动模式自行结束。日常无窗口运行选择菜单 16；高级批量测试见附录，必须传有限 `--duration`。真机入口拒绝 null / None 总预算，且保留原配置绑定和现场检查。

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

菜单 **15/16** 将输出根目录传为 `simulation_outputs/continuous_preview`。每轮以子程序实际打印的 `Continuous preview run_dir:` 或 `Continuous run:` 完整目录为准，不把旧轮次当成本轮。

主要文件是 samples.csv/full_log.csv、事件 policy_waypoints.csv、实际配置 config_snapshot.yaml、summary.json/termination.json 及 scan_stop_snapshot.json；早期异常可能缺文件。快照中的 Git dirty 说明有未提交改动，提交号不足以复现所有本地代码。

无窗口使用 **16**：菜单内选择场景并输入有限正数秒数，默认 12，只缩短原预算。`STOP_TIME_LIMIT` 不是一圈完成。查看时选 **19** 并选择实际目录，连续工具显式使用 `--format none`，默认只生成静态 PNG；输出以工具打印为准。需要箱体坐标图用 **20**，按提示选择匹配本轮的四角标定。菜单不默认选择最新仿真为真机，不覆盖采样数据。

**运行结束、失败停止、独立验收通过是三件不同的事。** 退出码 0、ENDED、关窗、保存成功都不表示完成轮廓。视频编码、指定视角和独立几何批量验证的直接命令移至文末高级调试，默认菜单不批量编码视频。

### 复查这次三角形及首次转折验证

原失败记录仍保存在 `simulation_outputs/continuous_preview/run_20260920_235436_498310/`。以下历史三角形场景显式保留 μ=0.03，不会被新默认 0.20 覆盖。其独立场景已保存为 `simulation/scene_direction_triangle.yaml`，中心和旋转来自原 config_snapshot。用菜单 **15**，场景提示输入 `simulation/scene_direction_triangle.yaml`；也可在窗口中加载。对应高级调试命令见附录：

（直接脚本示例见文末“高级调试”；日常从菜单选择。）

默认预演不限总时长，可观察原失败位置后继续运行，再手动停止。要复现旧版 120 秒截点，明确加 `--duration 120`。无 GUI 时把 `--preview` 改为 `--dry-run`，并为批量任务明确设置有限时长。原停在 102.02 秒，诊断为估计滞后 46.34°，相邻测量仅变化 2.84°；不是已发现的中心穿透。

需要重跑圆形、方形、三角形、平移旋转和无目标的独立几何验证，使用附录中的纯离线高级验证命令：

（直接脚本示例见文末“高级调试”；日常从菜单选择。）

它仅把离线验证时长延至 180 秒，不提高速度或放宽硬保护；预演的 `--duration` 是显式有限仿真时长，可以超过 120 秒；其他入口仍保留原有限预算规则。终端打印各轮目录和汇总 `direction_reports.json` 路径，每轮另有 `direction_geometry_report.json`，报告首次转折后的净前进、载荷、暂停/恢复、中心及线段穿透、异常压缩和真实终止原因。历史三角形报告曾通过原处转折，180 秒内转折后净前进约 65.8 mm；**不是整圈完成或真机验证**。报告中任何穿透都算失败，不能以动画看起来连续为准。

日常菜单 19 选择实际目录；指定视角的高级命令见附录（先设置实际目录变量）：

（直接脚本示例见文末“高级调试”；日常从菜单选择。）

会打印 `continuous_summary_target.png` 路径。默认双力箭头和 Fxy/F_ref；`--debug` 才显示左侧箭头图例并分区显示 N 和 mm/s、另列原始力，仿真 raw 按模型 Base 坐标标注，真机 raw 按传感器坐标标注，processed 才是处理后的 Base 力。未知真机目标和分离力保持 unavailable。

## 真机章节：连续策略复用已有扫描与沙箱标定

连续入口现在直接读取原工程保存的数据，不需要再填一份默认 `site_verification` 才能读取标定：

- 扫描起点和方向：`config.yaml` 的 `calibration.file`（默认 `scan_calibration.yaml`）。读取扫描 P0、方向参考 P1、P0→P1 初始方向、固定 Z/姿态及绑定 TCP。
- 沙箱四角：`continuous_tracking.workspace_calibration_file`（默认 `workspace/config/workspace_calibration.yaml`）。复用原有校验器检查实测 `raw_points.P0..P3` 和派生变换的一致性。这里的角点 P0 **不是扫描 P0**。
- 两条相对路径均以所用 `--config` 文件的目录为基准。缺失、损坏或已有 IP/TCP 元数据不匹配时，仍在设备构造前报出实际问题，不生成替代标定、不覆盖文件。

在菜单选择 **18**，先核对现场与完整路径，再输入 `OPEN_CONTINUOUS_SCAN`；子程序原 `START` 流程仍保留。输入错误或 `:q` 不启动子程序。

启动打印文件来源、扫描 P0/P1、四角及运行边界，并检查实际 TCP。**不在保存的扫描 P0 时，现在复用原工程的安全返回：输入一次 `START` 后，确认停稳、采集临时返回零偏，按原路径抬升到安全高度、移到 P0 上方、下降到 P0。** 已在安全高度时按原逻辑跳过多余抬升，不叠加高度。返回使用原 `safe_return` 参数和执行器，日志记录 `RETURN_TO_START`，本轮目录写入 `return_status.json`。

返回成功后重新检查 P0、Z/姿态及停稳，重新采集独立的扫描零偏，再进入连续跟踪，不再重复输入 `START`。返回的临时零偏不沿用到扫描，当前点也不会被保存为新 P0。已经在 P0 时跳过返回，仍按原连续入口顺序采集零偏、输入 `START`。首次接触后直接连续贴边，不做原离散三点初始化。

看到确认提示后，输入大写 `START` 并按回车；字符会正常显示，也可以退格修改。确认输入期间用 Ctrl+C 取消，运动期间 Q / Esc 无需回车即可停止。若运行的是此前“输入不回显”的旧进程，先 Ctrl+C 退出后重新执行命令，已运行进程不会自动加载修复。

如果在安全返回前采集临时零偏时偶发 `PX6D response timeout`，程序现在保持停止，清除旧串口缓存、查询版本响应重新同步，再丢弃首个力样本并重新采集完整零偏窗口。最多重试两次，总采集预算为样本数/采样频率加 `confirmation_timeout_sec`（当前默认约 2 秒）；重试记录保存在本轮 `startup_bias_retries.json`。机器人移动、状态过期、力异常、串口断开、持续超时或用户取消仍终止。此处理仅用于返回前已停稳的临时零偏阶段；返回运动、返回后的扫描零偏及连续跟踪不增加重试，50 ms 串口超时及原时序保护不变。

返回期间保留原返回力/力矩、速度和分段超时保护，并检查连续入口的沙箱边界、数据新鲜度和看门狗；不放宽跟踪的固定 Z、姿态或速度阈值。返回失败、日志失败或人工停止时终止本轮，不继续下降或开始扫描。**启动命令不会立即绕过确认执行返回，但输入 `START` 后会产生真实返回运动。** 此前该流程只通过离线设备替身验证，未验证真机返回路径。

每段返回到达目标附近后，连续入口用异步 `stopL` 请求刹停，并在 `*_SETTLE` 阶段继续读力/TCP、检查时序和看门狗；实测速度连续低于原阈值、持续满足原停稳确认时间后，才允许进入下一段。不再用阻塞刹停占用整个采样周期，也没有放宽 30 ms 周期间隔或 50 ms 看门狗预算。未停稳、力异常、采样/日志超时或人工停止仍终止返回。

此前控制修订曾修正下降结束与扫描启动的衔接：最后下降按原扫描要求到达 P0 的 **0.2 mm** 范围再刹停，停稳后仍检查位置；其他返回段及原离散返回继续使用原 0.5 mm 容差。不会把尚差 0.433 mm 的位置当成扫描起点，也不会改写标定。

连续真机入口的采样、事件及返回结果通过有界队列交给独立磁盘线程，保持原 CSV/JSON 格式和顺序；该线程不读设备、不发送运动或喂狗。暂时的磁盘写入延迟不再直接占用采样周期。队列最多 64 条（含正在写的一条），最老记录等待不能超过现有 `confirmation_timeout_sec`（当前 1 秒）；写入错误、队列满或积压超时仍终止，随后尝试保存停止原因与日志。若操作系统写入永久卡住，会明确报告日志清理未完成，不会等待无限时长或假报保存成功。传感器/机器人读取及喂狗返回仍受原时序限制，超时后不能继续发运动命令。启动命令与 START 操作不变；此流程已用替身联调，未完成真机验证。

如果示教器提示“检查现场总线连接”，查看报警编号；UR 的 C207 说明包含 RTDE 看门狗，并不只指 PLC。退出收尾现已修正：停止运动并取得新鲜的低速观测后，先结束本程序的 RTDE 控制脚本，再继续 80 ms 停稳确认、关闭接口，避免 50 ms 看门狗在正常收尾期间自行到期。运动中或停稳状态不可读时不撤掉看门狗。已经触发的报警需由操作者根据示教器提示和现场情况处理；程序不会自动解锁、重启机器人、重传脚本或关闭现场总线配置。连接失败的具体模式/保护状态会打印并保存到 `termination.json` 的 `robot_connection`，标为未验证新鲜度的故障快照，不能当作停稳确认。

沙箱四角的原始 Base XY 构成连续执行的多边形边界，保留 `boundary_margin`；实际 TCP 和预测下一步都检查倾斜边，不只检查轴对齐包围盒。已有 `workspace.enabled=true` 或 `real_test_xy_limits` 限制继续同时生效；两者都没有启用时，从旧四角取得范围。沙箱的拟合矩形、平均 Z、坐标旋转不作为扫描目标或扫描姿态；扫描仍在 Base 坐标使用原 P0 的 Z/姿态。四角几何不用于接触方向估计或找目标边。

两份标定、来源路径、完整四角快照和有效范围会进入本轮配置日志。采样新鲜度、watchdog、力/力矩、停稳、速度、方向重确认和恢复保护不变；缺省恢复仍关闭、丢边即停。若显式提供 `site_verification`，其内容及摘要仍须有效；摘要现在包含两份标定文件。没有把空记录伪造成 `true`，没有把沙箱元数据中的 `active_tcp_verified: false` 改成已验证。

已有标定离线检查使用菜单 **17**（不连接机器人或传感器）。共用检查是 8 扫描标定、14 箱体四角；标定采集为 7 和 13，独立复位保存/返回为 10/11。

该命令打印实际读取的点位及来源，并执行离线配置/SDK 检查。它不是设备运动验收。此前报告只包含离线文件检查和设备替身回归，没有连接硬件。

**返回和跟踪期间，Q / ESC / Ctrl+C 均原地请求停止，不触发再次返回 P0。** 本次接入的是启动返回；停止后仍保持连续实验的原地停止行为。不要把菜单 12 当作连续入口，它运行原离散策略，Q 的返回语义不同。

## 常见问题：先做一个明确动作

| 现象 | 下一步 |
|---|---|
| `cd` 或 Python 路径不存在 | 在文件管理器核对项目与 `/home/user-linux/robotics_cs/.venv312` 的实际位置 |
| 分支不对或改动来源不明 | 停止实验，查看 `git status` / `git diff` 并保留已有工作，先确认分支和修改归属 |
| 无 GUI / `No interactive Matplotlib backend` | 选择菜单 16 与 19；要交互窗口则回到有可用 Matplotlib 图形后端的桌面终端 |
| `Rejected / stopped` / 非法场景 | 看窗口中的具体字段；将目标放回容器内、避开 Start 接触包络，再开始 |
| 找不到目标 / `STOP_SEARCH_LIMIT` | 看搜索线是否经过目标；仅修改仿真场景后重新开始 |
| `STOP_TIME_LIMIT` 或恢复预算耗尽 | 打开本轮 `termination.json` 看原因和已走路径，不据此认定轮廓完成 |
| 力、时序、边界等安全停止 | 查看 `termination.json` 与 `scan_stop_snapshot.json`，人工检查原因，不盲目增大阈值 |
| `continuous measured speed exceeds experimental limit` | 查看本轮 `termination.json` 的 `speed_limit_observation`：实测 XYZ 速度（含 Z）、原门限和故障读取位姿；普通 `tcp_speed` 和停止快照可能属于另一次观测，旧日志缺故障字段时不能推算触发值 |
| 真机前置检查拒绝 | 按错误项与《连续跟踪修复说明》核对现场记录或配置，保留拒绝，不跳过检查 |
| 快捷键没有反应 | 点绘图区空白处退出路径输入焦点，再操作；也可直接点 Stop |

历史核验证据范围（非本轮菜单测试）：前一轮双力显示全量回归 **740 passed in 179.44s**；本轮不限时预演及底部布局的验收见当前修订报告。结果与截图见 [当前修订报告](CONTINUOUS_TRACKING.md)。修改前后原三角形 12,001 个周期的 raw/processed、TCP、状态和命令精确一致，显示开关不推进 RNG。无硬件 CLI、固定步长闭环一致性和可调用按钮/拖动回调已自动测试，静态 PNG 已实际查看；预演、无窗口及真机入口设备替身在同一观测流上逐条比对，真实控制主循环没有加入绘图。窗口缩放保持 XY 等比例。**未手动点击真实桌面窗口，未验证真机物理行为。**

## 本次菜单改造的边界

后续现场记录 `data/run_20260925_110044_430471` 显示启动返回 P0 完成，随后在 TARGET_SEARCH 阶段触发连续实测速度保护，尚未接触目标。本轮配置搜索指令为 1.0 mm/s，原实测 XYZ 门限约 1.342 mm/s。原 termination 中最后有效观测约 1.300 mm/s，触发的是下一次命令发送前的再次读取，旧日志没有保存该次完整值；不能据此确定物理原因。后续代码只补充故障读取诊断，保留原保护条件与原地停止行为，没有放宽门限或修复已证实的物理超速原因。新增字段的时间是主机读取起始时刻及之前的新鲜度检查设备时间，不承诺多个 SDK getter 来自同一个设备数据包。

日常流程与完整菜单编号见主指南：已有环境 → `python run_project.py` → 编号 → 提示 → 实际结果目录。换探针先示教器针尖标定并启用 TCP，再菜单 9 读取并自动保存本地 TCP 配置（保留备份）、核对输出、重采相关 7/13/10 标定、8/14/17 离线检查，最后现场确认与真机验证。读取 active TCP 不能代替针尖标定。

此前真机下降停稳阶段 **45.480 ms 超时**保留为独立问题，本次没有修改时序、看门狗、速度、力阈值、控制/日志线程或停止返回逻辑，也没有宣称已解决。上文的历史 μ=0.20 穿透失败、真机未验证及 GUI 人工验收限制继续有效。本轮菜单离线结果单独记录在 [MENU_OFFLINE_VALIDATION.md](MENU_OFFLINE_VALIDATION.md)。

## 高级调试

日常不需要以下命令；在工程目录使用已有环境，参数仍由原 argparse 处理。有限预演必须给 duration，其他模式保留原预算规则。视频为显式选择，不是菜单默认行为。回放命令中的变量须自行设置为实际目录。

```bash
CONTINUOUS_RUN_DIR="/实际运行目录"
python run_continuous_tracking.py --dry-run --duration 12 --output simulation_outputs/continuous_preview
python run_continuous_tracking.py --check-calibration
python run_continuous_tracking.py --execute
python tools/visualize_continuous_run.py "$CONTINUOUS_RUN_DIR" --format none
python tools/visualize_continuous_run.py "$CONTINUOUS_RUN_DIR" --fps 10 --format auto
```

```bash
python run_continuous_tracking.py --preview --duration 180 --scene simulation/scene_direction_triangle.yaml --output simulation_outputs/continuous_preview
```

```bash
python run_continuous_tracking.py --preview --scene simulation/scene_direction_triangle.yaml --output simulation_outputs/direction_preview
```

```bash
python -m simulation.continuous_validation --directions --output simulation_outputs/direction_revision
```

```bash
python tools/visualize_continuous_run.py "$CONTINUOUS_RUN_DIR" --format none --view target
```

```bash
git add -- CONTINUOUS_TRACKING.md BRANCH_continuous-tracking_傻瓜式操作指南.md config.yaml run_continuous_tracking.py experiment_logging/data_logger.py policy/continuous_tracking.py robot/rtde_controller.py simulation/simulated_robot.py simulation/continuous_validation.py simulation/continuous_session.py simulation/continuous_preview.py simulation/continuous_view.py simulation/scene_direction_triangle.yaml simulation/simulated_force_sensor.py experiment_logging/termination.py tools/visualize_continuous_run.py tests/test_continuous_run.py tests/test_continuous_tracking.py tests/test_continuous_execution.py tests/test_continuous_geometry.py tests/test_continuous_revision.py tests/test_continuous_preview.py tests/test_continuous_direction.py
git diff --cached --name-only
git diff --cached --stat
git diff --cached
```
