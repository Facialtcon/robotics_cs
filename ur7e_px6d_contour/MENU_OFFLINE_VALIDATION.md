# 统一菜单离线验收记录

下文第一轮结果保留为历史记录。后续用户明确要求菜单 9 自动保存 TCP，因此 9 现增加 `--write-config`；其余原编号映射不变。追加验证见文末。

本轮检查时分支为 `experiment/continuous-tracking`，最新提交名称 `9_24`、短 SHA `7d594cf`，初始 `git status --short` 为空。只记录实际检查结果，不根据提交日期推断文件修改日期。本轮没有切分支、commit、push、reset、stash、升级依赖或替换 Python。

## 改动范围

| 文件 | 用途 |
|---|---|
| run_project.py | 保留原 1～12 命令和 --step，追加 13～20；菜单参数选择、确认、日志识别、串行等待与错误处理 |
| tools/check_workspace_calibration.py | 只读 CLI 封装原 load_calibration 校验器，打印箱体四角与派生几何 |
| tests/test_run_project.py | 旧入口兼容、新映射、安全确认、路径/日志错误、离线实际适配器及终端收尾测试 |
| 从这里开始.md | 唯一日常主指南、真实菜单表、换探针顺序、各策略停止语义、高级命令附录 |
| 操作文档.md | 原离散细节与历史故障记录，取消与主指南冲突的入口说明 |
| BRANCH_continuous-tracking_傻瓜式操作指南.md | 保留详细预演操作与历史失败，统一菜单启动，直接命令移入高级调试 |
| MENU_OFFLINE_VALIDATION.md | 本轮实际证据、失败、未验证项目 |

控制/力反馈公式、状态机、时序预算、看门狗、速度/阈值、仿真模型、真机采样和日志线程未改。已有脚本的 argparse 与直接命令保留。菜单 12 仍为 `main.py --sensor real --execute`；连续真机新增为 18。

## 实际测试命令与结果

以下命令在仓库根目录运行；环境是已存在的 `.venv312`。没有创建另一套工程或环境。

入口有限测试（最终版本）：

```bash
PYTHONPATH= PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg MPLCONFIGDIR=/tmp/robotics-menu-mpl .venv312/bin/python -m pytest -q ur7e_px6d_contour/tests/test_run_project.py
```

**57 passed in 10.10s**。覆盖：

- 原 1～12 的 `--step` 命令逐项精确比对，解释器、工程 cwd、子进程环境清理和终端继承；硬件菜单在 Popen 前由替身截获。
- 13～20 的脚本/参数、有限正数时长、预演不传时限、真机预算不覆盖、取消及错误确认不启动子程序。
- 菜单启动不连接设备；实际预演用 Agg 和注入明确停止有限结束；实际无窗口运行 0.05 仿真秒；实际离线检查和连续静态回放。设备构造均被拦截，测试产物在 pytest 的 `/tmp` 临时目录。该组合测试使用离线 SDK 契约替身，独立真实 SDK 检查见下文。
- 含空格路径、无效输入/时长、缺依赖、空目录、空采样、缺回放文件、损坏元数据、模式未知、不自动挑“最新真机”。
- 子程序失败、启动异常、取消、EOF、Ctrl+C 不报告任务成功，不自动开始下一任务；反复 Ctrl+C 后仍等待退出。
- 真实伪终端与无硬件 Python 子进程：START 正常回显，向进程组发送 SIGINT 后，子进程 CLEANUP_DONE 必须先于菜单返回 130；测试自身设有限超时。
- 真实临时连续日志经静态工具生成 PNG，原临时日志逐字节不变，无 MP4/GIF；主指南编号表逐项与源码比对。

开发中入口测试曾暴露两个测试夹具问题：复制的场景缺少相对 base_config、全局 Popen 拦截误阻止 Matplotlib 字体探测；另一次 SDK 构造替身缺方法文档导致契约检查拒绝。已修正临时场景夹具和拦截范围，并把组合测试中的 SDK 契约替身显式标出；没有修改控制实现或放宽验收断言。

必要相关回归（在 `ur7e_px6d_contour` 目录运行）：

```bash
PYTHONPATH= PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg MPLCONFIGDIR=/tmp/robotics-menu-mpl ../.venv312/bin/python -m pytest -q \
  tests/test_continuous_run.py tests/test_continuous_preview.py \
  tests/test_continuous_preview_duration.py tests/test_continuous_saved_calibration.py \
  tests/test_continuous_execution.py tests/test_continuous_startup_return.py \
  tests/test_continuous_connection_cleanup.py tests/test_operator_input.py \
  tests/test_read_active_tcp.py tests/test_scan_calibration.py \
  tests/test_workspace_calibration_cli.py tests/test_workspace_replay.py \
  tests/test_workspace_visualizer.py tests/test_visualize_run.py tests/test_reset_pose.py
```

**1 failed, 215 passed in 27.72s**。失败用例：

`tests/test_continuous_run.py::test_identical_geometric_observation_stream_across_three_entry_adapters`

其 FAKE RTDE 执行适配器触发 `ContinuousLogError: continuous log queue reached capacity 64`，导致观测流一致性断言失败。相关 `run_continuous_tracking.py`、`continuous_writer.py` 和原测试文件与 HEAD 无差异。未删除用例、放宽断言、改日志队列或控制时序。

仅针对该失败做一次单独复查：

```bash
PYTHONPATH= PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg MPLCONFIGDIR=/tmp/robotics-menu-mpl ../.venv312/bin/python -m pytest -q tests/test_continuous_run.py::test_identical_geometric_observation_stream_across_three_entry_adapters
```

结果 **1 passed in 1.40s**。这说明此次复查未重现，不抹去整组回归失败，也不据此宣称问题已修复。复查输出保留在 `/tmp/menu-regression-recheck.log`。

另用临时防硬件封装 `/tmp/menu_guarded_check.py` 调用原 `--check-calibration`，只替换 URRTDEController/PX6DReader 构造器为禁止调用，保留真实已安装 SDK 的契约检查：

```bash
PYTHONPATH= MPLCONFIGDIR=/tmp/robotics-menu-mpl .venv312/bin/python /tmp/menu_guarded_check.py
```

退出码 0，打印 `Offline calibration check only; no device connection or motion.`。输出在 `/tmp/menu-guarded-check.log`。这不是设备连通或运动验证。

`git diff --check` 通过。

## 文件保护与诊断产物

对根目录 YAML、workspace/config、data、simulation_outputs 的 **2,277 个已有文件**建立 SHA-256 清单；最终原文件内容、文件数量均与基线一致，无原配置、标定、site_verification 或实验日志被修改、覆盖或删除。清单与比对脚本分别在 `/tmp/menu-protected-before.json`、`/tmp/menu_protected_manifest.py`。

需要如实记录一个旧回归用例的输出路径问题：其 `force_direction_sign must be +1 or -1` 早期配置拒绝分支曾在工程 `data/termination_20260924_181213_778879/` 新建 termination.json/txt。它们是本轮自动测试诊断，没有硬件采样。已将这两份新文件完整移到 `/tmp/menu-regression-artifacts/termination_20260924_181213_778879/` 保留，工程原实验目录恢复为基线文件集合。未移动或删除任何既有实验日志，也未为此修改原控制/诊断实现。

## 未验证与历史限制

- 未连接机器人或 PX6D，未执行任何真机运动；所有硬件菜单只检查启动契约。
- 未手动操作真实桌面 GUI；预演回调/停止及静态回放做了离线测试，终端做了无硬件伪终端测试。
- 未跑整个仓库全量测试；本轮不是“全绿”报告。主指南原来的固定预期测试数量已移除。
- 历史连续 μ=0.20 几何穿透失败、三角形旧失败、历史 TCP 不匹配及先前验证限制仍保留；本次没有重做物理/几何或真机验收。
- **45.480 ms 真机下降停稳超时未解决，本轮未改该逻辑。**

日常使用流程与完整编号见 [从这里开始.md](从这里开始.md)。最短连续预演：打开统一菜单 → 15 → 场景回车 → Start → 手动停止 → 19 选择实际 run_dir。真机需完整准备和现场确认后选择 18，不能把 12 当作连续扫描。

## 后续调整：菜单 9 自动保存 active TCP

按用户后续明确要求，菜单 9 改为调用 `tools/read_active_tcp.py --write-config`。这是对初始“第 9 项只读、不写配置”要求的明确调整；编号和目标脚本不变，其他编号调用不变。底层工具不加此参数时仍默认只打印。

新行为：读取并校验六个有限数值，断开读取接口后，更新所选配置的 `tcp.offset`；保留其他字段及注释，以原子替换保存，并留下字节一致的原配置备份。完全一致时不改写、不新增备份。读取/断开失败、数据无效或发现配置在读取期间被修改时拒绝覆盖。不会调用 setTcp、发送运动命令或修改已有标定/现场验证记录。

本次增改 `tools/read_active_tcp.py` 和 `tests/test_read_active_tcp.py`，同步菜单、入口映射测试及三份操作指南。测试只使用临时配置及 RTDE 替身；未连接真实设备，实际 config.yaml、扫描/箱体/复位标定均未改写。

执行（仓库根目录）：

```bash
PYTHONPATH= PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg MPLCONFIGDIR=/tmp/robotics-menu-mpl .venv312/bin/python -m pytest -q ur7e_px6d_contour/tests/test_read_active_tcp.py ur7e_px6d_contour/tests/test_run_project.py
```

结果 **69 passed in 13.78s**。新增覆盖自动保存、原命令只读、原配置备份、重复读取不重复备份、读取/断开失败、无效数据、并发编辑拒绝、原子替换失败、块式/行内 YAML 注释保留。`git diff --check` 通过，实际配置和标定文件的 Git diff 为空。此结果不覆盖或改变上文已记录的旧回归失败及真机验证限制。

## 后续修复：示教 P1 后 getTCPOffset 失败

用户现场日志在读取 P1 pose/speed 后报告 `RTDE control script is not running`，随后 `getTCPOffset() function did not succeed`。旧标定工具在示教前建立一个 Control 接口，并跨越 P0/P1 的手动移动及输入持续复用。现场日志能确认读取时脚本已停止，不能仅凭该日志断言具体是哪次示教、模式切换或外部事件终止了脚本。

检查本地已安装 SDK 1.6.5：Receive 类没有 getTCPOffset；Control 构造器默认 flags 为 FLAG_UPLOAD_SCRIPT。接口语义也参考 [SDK 官方接口定义](https://gitlab.com/sdurobotics/ur_rtde/-/blob/master/include/ur_rtde/rtde_control_interface.h?ref_type=heads)。

改动集中在 `run_calibration.py`、新增 `tests/test_scan_calibration_cli.py` 及主指南/操作文档：

- 仅在明确输入 SAVE_P0 / SAVE_DIRECTION 后，为该次点位构造独立 TCP 读取会话，采样后断开；不持有旧控制接口等待示教移动。
- 构造器仍按 SDK 默认加载辅助脚本，没有加入 reuploadScript、自动重试、保护解锁或运动命令。连接前及读取后按原静止/安全规则核对，采用连接后的新位姿。
- P0 的 TCP 绑定在实际保存请求后读取、与本地配置比较；P1 仍与 P0 的 TCP 绑定比较，几何检查不变。
- P1 读取的 RuntimeError 转为本次采样失败，不写 P1，回到输入提示；必须重新输入 SAVE_DIRECTION 才重新采样。CANCEL 或 Ctrl+C 保留未完成标定，不伪造有效数据。
- 完成提示区分原离散 12 与连续 18 的 START/启动返回顺序。

有限离线测试命令（仓库根目录）：

```bash
PYTHONPATH= PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg MPLCONFIGDIR=/tmp/robotics-menu-mpl .venv312/bin/python -m pytest -q ur7e_px6d_contour/tests/test_scan_calibration_cli.py ur7e_px6d_contour/tests/test_scan_calibration.py ur7e_px6d_contour/tests/test_read_active_tcp.py
```

**35 passed in 1.01s**。全部设备接口均使用替身；覆盖示教使旧脚本失效、每个 SAVE 独立会话、失败不自动重试、明确手动重采、TCP 不匹配/非有限值、运动/保护状态拒绝、连接后运动拒绝、取消及断开清理。未连接真实设备；现场是否恢复仍需操作者验证。

本次开始时用户已更新 config.yaml、生成 TCP 备份，并将 scan_calibration.yaml 保存为 confirmed: false 的 P0。已对本次开始时的实际配置/标定/备份内容做 SHA-256 比对，全部保持不变；没有沿用第一轮开发前的配置覆盖用户新数据。旧进程已退出，下一次应从菜单 7 重新采集 P0/P1，而非使用日志中的 P1 数值补造标定。此前 45.480 ms 超时与旧日志队列回归失败不在本次修复范围。

## 后续修复：独立返回在下降前触发 pending measured standstill

现场证据为 `data/run_20260925_104726_978904/`。summary 的 reason 是 `manual reset return`，不是连续扫描；return_status 为 aborted。最后一条 samples 是 `DESCEND_TO_START_PRECHECK`，XYZ 线速度模长约 **1.43255 mm/s**，高于控制器原停稳判据 **0.1 mm/s**。最终记录 Z 为 **0.1336177524 m**，P0 Z 为 **0.0267892995 m**。下降 target 的打印在运动调用之前；控制器在发送 moveL 前拒绝了该调用，不能把该行打印当成下降已执行。

根因：共用 SafeReturnExecutor 的默认 `_stop_at_segment_end` 原先停止后仅读取一次状态，调用方只复查位置/姿态，就进入下一段预检查；有残余速度时，URRTDEController 的原保护正确拒绝下一条 moveL。没有证据证明应放宽速度门限。SDK 的运动/停止调用参考 [官方异步运动示例](https://sdurobotics.gitlab.io/ur_rtde/pages/examples/basic_motion/move_async_example.html)，本次诊断数值及门限均来自本地日志和源码。

改动：`safety/safe_return.py` 默认段尾停止后增加受力监控下的观测等待，使用控制器原 `continuous_settle_speed_mps`/默认 1e-4 m/s 判据；等待使用原单段截止时间，不重置或延长预算。期间沿用原采样周期、力限制和机器人状态检查，日志记录 `*_SETTLE`。只有观测满足判据后才交回原位置/姿态复核；下一段仍执行原预检查和控制器保护。没有清除 pending-stop 标志，没有自动重发 moveL/stopL，没有改速度、返回路径或标定。

`ContinuousStartupReturn` 的异步 `_stop_at_segment_end` 覆盖实现、80 ms 连续低速确认、看门狗和时序检查未修改。本次不处理此前 45.480 ms 连续入口超时。默认 SDK stopL 本身的阻塞行为也未改成新的线程或异步机制。

新增 `tests/test_safe_return_settling.py` 使用真实 URRTDEController 保护逻辑与假的 SDK、传感器、时钟复现残余速度，不连接任何硬件。修改前新测试结果为 **4 failed, 3 passed**，其中正常返回用例重现用户同名错误；之后补充日志失败和新观测再次移动的拒绝测试。

最终相关回归（仓库根目录）：

```bash
PYTHONPATH= PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg MPLCONFIGDIR=/tmp/robotics-menu-mpl .venv312/bin/python -m pytest -q ur7e_px6d_contour/tests/test_safe_return_settling.py ur7e_px6d_contour/tests/test_safe_return.py ur7e_px6d_contour/tests/test_safe_return_termination.py ur7e_px6d_contour/tests/test_reset_pose.py ur7e_px6d_contour/tests/test_continuous_startup_return.py ur7e_px6d_contour/tests/test_continuous_execution.py
```

**88 passed in 6.45s**。覆盖残余速度等待、持续未停稳超时、停止耗时不重置原单段预算、力/传感器/日志故障中止、停后位置漂移、Ctrl+C、新观测撤销停稳条件，以及连续返回和控制器原保护回归。没有通过删除测试或放宽旧断言取得通过。

对本轮开始时的实际配置、TCP/箱体备份、标定及本次失败运行的 **19 个文件**做 SHA-256 比对，均未改变。清单在 `/tmp/return-fix-protected.json`，复现和回归输出在 `/tmp/return-settling-before.log` 与 `/tmp/return-settling-regression.log`。主指南及原离散操作文档已同步等待语义。未连接设备、未执行运动；离线通过不表示现场返回验证通过。

## 后续诊断：连续初始搜索触发实测速度保护

分支仍为 `experiment/continuous-tracking`，HEAD 为 `7d594cf`（`9_24`）；未切分支，没有按提交日期推断修改时间。此前未提交改动全部保留。本次只读取用户提供的 `data/run_20260925_110044_430471`，未连接设备或重试实验。

现场 return_status 为 complete，随后 TARGET_SEARCH 停止。按该运行配置快照与 `prepare_real` 的实际公式，基础连续速度限制为 `max(search_speed, reacquire_speed, hypot(tangential_speed, normal_speed_limit)) = 0.00111803398875 m/s`；原实测检查是 XYZ 模长严格大于基础限制的 1.2 倍，即 `0.00134164078650 m/s`。本次不修改该公式、比较条件、Z 分量、运动参数或采样时序。SDK 速度含义可核对 [官方 RTDE 接口文档](https://gitlab.com/sdurobotics/ur_rtde/-/blob/master/include/ur_rtde/rtde_receive_interface_doc.h)；具体数值取自本地源码及该运行记录。

该运行保存 17 条 TARGET_SEARCH 采样，首末相隔约 0.1607 s，最大已保存 XYZ 实测速度约 1.30645 mm/s；termination 的最后一次有效观测约 1.29968 mm/s。真正的拒绝发生在 `command_planar_velocity` 内部再次 `read_state`，在发送下一条 speedL 前。此读数直接抛出异常，未返回给主循环，旧 CSV/termination 未保留它。不能将已保存的低于门限的观测说成触发值，也不能从停止后的低速倒推触发值。物理超速原因仍未确定。

改动文件及用途：

- `robot/rtde_controller.py`：只在原连续速度保护失败分支构造 RobotError 子类，复制本次已读位姿/六维速度、XYZ 模长、基础限制、1.2 倍门限和时间信息到异常内存；错误文本打印数值。没有新增设备读取、文件写入、控制循环或正常路径计算。
- `run_continuous_tracking.py`：沿用原异常处理，先请求停止，再将异常快照记入 `termination.json` 的 `speed_limit_observation`，保留最后有效状态与停止后快照原含义。故障仍分类为 STOP_MOTION_ERROR，返回失败，不自动返回或重试。
- `tests/test_speed_limit_diagnostics.py`：真实控制器保护逻辑配 SDK 替身、连续入口配控制器/传感器替身，输出只用临时目录。
- 主指南、连续指南及本记录：说明读取时刻的区别、历史数据缺口及未验证范围。

`host_read_start_monotonic_sec` 为该次主机读取起始时间，`last_checked_device_timestamp_sec` 为此前新鲜度检查缓存的设备时间。多个 SDK getter 不保证同一数据包，诊断不伪称同步采样。

先运行新增有限测试：

```bash
PYTHONPATH= PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg MPLCONFIGDIR=/tmp/robotics-menu-mpl .venv312/bin/python -m pytest -q ur7e_px6d_contour/tests/test_speed_limit_diagnostics.py
```

修改生产代码前 **2 failed, 4 passed**，失败体现故障观测缺失；补齐诊断后 **6 passed in 0.24s**。覆盖故障读取复制、无额外读取、禁止下一条 speedL、原严格大于边界、Z 分量参与、返回模式豁免与全局速度保护、先停止再记故障、停止后零速不能覆盖故障速度。

随后相关回归：

```bash
PYTHONPATH= PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MPLBACKEND=Agg MPLCONFIGDIR=/tmp/robotics-menu-mpl .venv312/bin/python -m pytest -q ur7e_px6d_contour/tests/test_speed_limit_diagnostics.py ur7e_px6d_contour/tests/test_continuous_execution.py ur7e_px6d_contour/tests/test_continuous_connection_cleanup.py ur7e_px6d_contour/tests/test_continuous_startup_return.py ur7e_px6d_contour/tests/test_continuous_run.py ur7e_px6d_contour/tests/test_termination.py
```

**140 passed in 13.00s**。测试全部离线，覆盖连接清理、启动返回、停止分类、异常/键盘停止、有限模拟及回放。本次通过不抹去上文已记录的日志队列历史回归失败，也不表示该时序问题已经修复。此前 45.480 ms 超时仍未解决，未做真机物理超速、GUI 人工或现场扫描验收。

本次开始时的实际配置、扫描/箱体标定、用户 TCP/箱体备份及该次运行的 **16 个文件**经 SHA-256 比对全部不变，清单在 `/tmp/speed-fault-protected.json`。`git diff --check` 通过。没有覆盖用户新标定、现有配置或实验日志，没有执行 git commit/push/reset/stash。
