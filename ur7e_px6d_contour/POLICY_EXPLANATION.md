# Local boundary tracking：策略学习说明

当前版（2026-09-11）用接触点几何跟踪局部边界，不判断“直边/90°拐角”。
这里只解释算法，不构成真机运行许可。操作顺序看 [操作文档.md](操作文档.md)。

## 1. 第一次为什么不能直接知道 tangent？

一个接触点 C0 只告诉我们“这里碰到了”。经过同一个点可以画无数条直线，因此一个点不足以
确定切线。P0→P1 是搜索方向，也可能斜着碰到目标。第一次力的方向还会混入摩擦、颗粒阻力，
所以不能把它旋转 90° 当成确定的切线。

首次搜索本身也是一个 probe episode：从搜索起点出发，第一次过阈值立即停止推进，原地等待
稳定接触，然后沿搜索路径回到自己的起点。之后才沿已经走过的自由路径靠近局部 A0。
若搜索路径很长，返回也会很长；这是当前保守实现的时间代价。

## 2. 为什么斜着 probe 也可以？

探测方向只负责把探头从自由空间带到边界，不必垂直于边界。从几个稍有横向差别的 anchor
做平行探测，仍能得到多个边界位置。初始化在 A0 附近做 −offset、0、+offset 三次探测，
每次均使用 `probe_speed`，触力停止并返回自己的 anchor 后才处理结果。
首次 SEARCH 接触 C0 只用于确定这个邻域，不参与拟合：SEARCH 和局部探测速度不同，
真实接触形变与滤波延迟可能使其触发位置不同。
三个新的低速接触点必须有足够间距且近似共线。单轮不通过会触发有限重试，而不是直接结束扫描：
先沿实际走过的 anchor 转移折线返回局部 A0，在同一窗口重测一次；仍失败则分别在
`[+offset, +2offset, +3offset]` 和 `[-3offset, -2offset, -offset]` 窗口各测三次，
尝试避开跨越两个局部面的情况。每轮使用三个全新同速点，不从失败批次中挑点凑直线。
每次仍先回自己的 anchor；anchor 转移仍受力保护。

`initialization_max_retries: 3` 表示最多四轮、十二次局部探测；
`initialization_timeout_sec: 120` 限制初始化总用时（包括转移、回位）。任一轮通过就进入跟踪，
重试耗尽、超时或安全异常才停止。终端显示 `INITIALIZATION_RETRY` 和实际残差；
`probe_episodes.csv/json` 中的 `initialization_round` 记录每条探测所属轮次，失败批次保留拒绝原因。
偏移运动也受力保护，不能因为称为“anchor”就假定新位置一定安全。

## 3. 为什么主要从多个 contact points 估计 tangent？

代码 `policy/boundary_estimation.py` 对最近局部 TCP 接触点做 PCA：找出这些点延伸最明显的
方向，也就是拟合直线的方向。只有两个点时相当于连线；三个点还可检查偏离直线的残差。
普通跟踪默认使用最近三个点，恢复确认成功后重置局部窗口，避免把旧局部段混入新方向。

PCA 的正负方向本来等价。这里用历史成功探测的 anchor→contact 方向确定目标侧，再用固定
`follow_hand` 选择走向；不会因为某一帧力反向就反转扫描。圆弧上，窗口滑动会逐渐改变切线，
但较大曲率、噪声或短点距仍可能让拟合不可靠。

## 4. Force 当前负责什么？

processed Base-frame `Fx,Fy` 的模长负责接触阈值、接触保持和上升率监控。raw/processed 六维
F/T 同时用于安全上限；力矩仍记录，不做接触位置反演。新 policy 不用力向量角度估计边界。
旧 `probe_direction_sign` 不再决定运动方向；它仅是历史配置字段。

预处理链保留：raw → 软件零偏 → EMA → 固定姿态重力项 → sensor-to-base 变换 → 颗粒基线。
单纯平移 wrench 的参考点改变的是力矩（`r × F`），不改变合力。安装坐标系的旋转仍必须正确，
不能把模拟中的方向约定直接当作真实 PX6D 安装标定。

## 5. 什么是 probe anchor？

Anchor A 是一次探测开始时实际测到的 TCP pose。episode 创建后复制并锁定这个值，探测终点、
接触点、后续切线变化都不能修改它。日志同时保存返回时的实际 pose 和误差。

## 6. 为什么每次都必须 return anchor？

如果直接从接触点沿切线走，切线估错就会把探头推入物体。如果从失败射线终点开始另一条射线，
整条路径也不再是原来验证过的路径。因此每条射线接触、无接触或瞬态接触后都先停止，再返回
自己的 A。只有返回完成，调度器才接受点、开始确认、换射线或切向步进。

“同一位置”采用配置的定位容差：正方形理想模拟 1 μm，真机配置仍是 0.2 mm，不能混为一谈。
力安全超限、通信异常或人工急停优先于返回；此时不强行运动，日志明确 `return_completed=false`。

## 7. 正常 tracking 如何循环？

```text
TARGET SEARCH
    ↓
FIRST CONTACT → RETURN TO SEARCH ANCHOR
    ↓
LOCAL INITIALIZATION（附近平行探测，各自返回）
    ↓
BOUNDARY TRACKING
    ↓
PROBE
    ↓
RETURN TO SAME ANCHOR
    ↓
CONTACT?
   /       \
 YES        NO
  ↓          ↓
UPDATE       BOUNDARY RECOVERY
TANGENT       ↓
  ↓         局部射线 → RETURN → 候选？
沿已验证       ↓
射线路径到    BOUNDARY CONFIRMATION
clearance     ├─失败→沿已走过的 anchor 路径退回→继续 recovery
  ↓           └─三点几何一致→BOUNDARY REACQUIRED→tracking
TANGENT STEP
  ↓
NEXT ANCHOR → PROBE
```

返回后先沿刚成功的射线路径靠近到离接触点一个 clearance 的位置，再做小切向步进。
两个转移都受力保护。下一次 probe 方向取局部拟合切线的垂直方向，符号由历史目标侧决定，
不固定使用最初 P0→P1。

## 8. 什么情况下进入 recovery？

达到最大探测距离仍无稳定接触，或新接触无法形成可靠且向前的局部拟合时，先完成返回，再进入
恢复。无接触不产生新的边界点，也不表示已经知道这里是什么几何形状。

## 9. Recovery 为什么不知道全局 outside？

策略只知道刚返回的 anchor 和走过的路径，不知道物体中心、尺寸、角点或下一段边界。
恢复从该 anchor 以预测 probe 方向为参考，按固定手性连续扩大局部搜索扇区，先小范围，再扩大。
`recovery_sector_extents_deg` 是可配置角域预算，不是“下一条边的角度”。每条短射线独立 guarded，
每次都回到相同 anchor；没有长对角线穿越目标，也没有用真实几何算安全外侧。

首次恢复接触只是 candidate。返回后从附近 anchor 做额外探测，默认总计三个候选接触，检查间距、
拟合残差、向前位移与反转，再接受。拒绝旧点/向后采样使用局部历史，不禁止最终回到起始区域。
确认失败先沿此前实际走过的 anchor 折线路径退回恢复起点，继续扩大搜索；耗尽预算则停止。
这是局部凸边界策略，不保证凹槽、多物体或任意形状都能恢复。
对尖锐转折，第一个 candidate 可能落在过渡处，不应强迫它与新局部段共线。
确认器会从最新接触中选残差合格的局部共线后缀，只接受这些点。这是残差规则，不判断“三角形”或固定角度。

## 10. Square 为什么只是测试环境？

`RuleBasedPolicy` 的输入只有 policy 参数、时间、raw/processed wrench 和 TCP 状态。它不能访问
模拟环境，代码没有 square/rectangle、目标中心、尺寸或边编号。

模拟使用 100 mm 方形和明确的 1 mm 柔性探头感力包络：探头中心（TCP）在边界外就能建立力，
力大小来自包络压缩量。TCP 仍按命令积分，绝不投影、裁剪或吸附到边界。接触点是实测 TCP，
并非偷偷替换成物体表面的 ground truth。这不是实际颗粒接触模型，也没有刚性防穿透约束。

独立检查器逐条检查执行线段是否穿过物体内部（不仅看离散端点），并检查触力后是否继续沿射线
推进。任何穿透直接 PHYSICAL FAILURE。关闭探头包络后，当前参数确实会穿透并失败；测试保留
这个反例，不能把碰撞裁剪掉来得到 SUCCESS。

完成还须满足全部 episode 返回、所有接受点在边界附近、四段覆盖、四次恢复确认、足够路径和
同向环绕、回到起始区域且切向一致。评分只在 simulation 外层运行，不给 policy 提供下一步动作。

交互动画还可选圆形和等边三角形，或拖动目标。切换/拖动是环境编辑，完成后重启实验；
policy 仍只获得 wrench、TCP 状态和 P0→P1 方向。普通拖动不改搜索线，Shift+拖动才表示整个实验布局一起平移。

## 日志怎么读？

`probe_episodes.csv/json` 一行/条对应一次 episode，含 probe_id、anchor_pose、方向、开始/结束
时间、最大距离、contact/outcome、contact_pose、峰值力、返回完成、实际返回 pose、接受标记、
状态和拒绝原因。JSON 及 `probe_forces.csv` 包含每次 probe/hold/return 的 Fxy 时间序列。
`samples.csv/full_log.csv` 的 probe_id 可关联实时六维 raw/processed F/T。

先看 summary 的 physical_failure 和 probe_audit，再看失败 probe_id 的 episode，不要只看 PNG。
