[English](README.en.md) | **简体中文**

# FRC 12363 LEOMOTIVE · 2026「REBUILT」机器人代码

![Java](https://img.shields.io/badge/Java-17-ED8B00?logo=openjdk&logoColor=white)
![WPILib](https://img.shields.io/badge/WPILib-2026-6E3496)
![AdvantageKit](https://img.shields.io/badge/AdvantageKit-26.0.2-1D9BF0)
![Phoenix 6](https://img.shields.io/badge/CTRE_Phoenix_6-TalonFX%20%2F%20Pigeon%202%20%2F%20CANrange-D2222D)
![maple-sim](https://img.shields.io/badge/maple--sim-0.4.0--beta-2EA043)

由 FIRST® Robotics Competition 战队 [**12363 LEOMOTIVE**](https://github.com/LEOMOTIVE-12363) 维护的 2026 赛季（游戏 **REBUILT**）机器人代码。本项目以 **8810 Alphabots 的机器人及其开源代码**为基础，由 LEOMOTIVE 持续优化与开发，形成我们自己的版本。技术基础：Java 17 · WPILib 命令式框架 · AdvantageKit TalonFX swerve 模板。

> **项目归属与上游来源**：本仓库属于 **12363 LEOMOTIVE**，是我们独立维护的衍生版本；**8810 Alphabots** 是原始机器人与上游代码的来源。代码 fork 自 [Alphabots-8810/8810-2026-offseason](https://github.com/Alphabots-8810/8810-2026-offseason)。`main` 保留上游竞赛代码基线及本 fork 的许可、文档与仓库卫生改动，`demo` 承载 LEOMOTIVE 当前的纯手动演示版本开发，详见[分支模型](#分支模型)。

## 功能亮点

以下为 `main` 保留的上游竞赛代码能力；LEOMOTIVE 当前在 `demo` 线上进行纯手动演示版本的适配与改进，两条分支的功能范围见分支模型：

- **Swerve 底盘** — CTRE TalonFX + Pigeon 2，硬件常量由 CTRE Tuner X 生成（`generated/TunerConstants`），Phoenix 高频里程计线程，自带 SysId 标定例程
- **FUEL 机构链** — 可展开 intake（位置控制臂 + 滚轮）→ hopper / indexer（CANrange 传感器检测存球）→ Drum 发射鼓 + 角度可调 Hood
- **战术程序** — HUB 射门（支持移动中射门）、中立区 Ferry 吊射、Trench 隧道通过（驾驶辅助版 + [Autopilot](https://github.com/therekrab/autopilot) 全自动版）
- **视觉** — Limelight（MegaTag2）全局位姿估计，融入 swerve 里程计
- **AdvantageKit 三模式** — REAL / SIM / REPLAY 共用一套代码，日志可确定性回放
- **maple-sim 桌面仿真** — 2026 REBUILT 场地（含 HUB / Trench 碰撞体）物理仿真
- **Elastic 自定义仪表盘** — 布局版本化于 `elastic-layout-1027.json`；配套 NetworkTables 调参脚本 `scripts/nt_tune.py`、`scripts/pid_analysis.py`

## 分支模型

| 分支 | 内容 |
|---|---|
| `main` | 上游 Alphabots 8810 竞赛代码基线，保留本 fork 的 MIT LICENSE、双语 README 与私有配置退跟踪改动：完整自动程序（Choreo 轨迹 + PathPlanner）、视觉、战术程序 |
| `demo` | **LEOMOTIVE demo 展示线**（本仓库的日常开发线）：移除全部自动程序与视觉，纯手动驾驶；Elastic 数值输入框直控 Drum 转速 / Hood 俯仰，并新增 demo 调参页；可寻址 LED 灯带（五套《Coco》亡灵节主题＋Off 与纯色/彩虹/追逐/呼吸四个独立模式（2026-10-26 曾精简为三主题＋Off，同日按用户要求全部恢复）；五套主题使用各自独立的《Coco》设计色板（2026-10-27 恢复历史配色，取代 2026-10-26 曾短暂采用的统一橙紫融合方案）；配置总长 7.0 m 对齐实装灯带；`LED Pattern` / `LED Idle Effect` / `LED Brightness (%)` 由 Elastic 选择并断电持久化，Enable、Disable、空闲、吸球、射球、吐球时都保持同一所选样式，Slow Flow / Chase / Breathe 等用户所选动画仍随时间播放；`Follow Ball Flow` 开关已移除；灯带分段与动画参数待实灯核验；CANdle 已选定为默认驱动，PWM 保留为编译期备用；改 `LEDConstants.HARDWARE` 一个常量即可切换，硬件安装参数与实灯表现仍待核验） |

- demo 展示线的功能改动**永不合入 `main`**；每个任务一个独立 PR，base 为 `demo`，自检通过后即合入。仓库共用的中英 README 同步维护：从 `demo` 发起文档改动，再以独立 PR 将相同文案同步至 `main`。
- 查看 `demo` 相对 `main` 的全部差异，请使用 GitHub Compare：[compare/main...demo](https://github.com/LEOMOTIVE-12363/Alphabots-2026-lm/compare/main...demo)（原先长期保持 open 的总览 PR #2 已于 2026-10-23 关闭，无须常驻 open PR）。

## demo 手柄分工

Driver Station 编号 **1 优先于 0**，优先级同时适用于驾驶和所有按键。只要 1 号连接，0 号输入就不生效；1 号断开后切换到 0 号，两者均断开时驾驶输入为零。编号是 Driver Station 槽位，不是电脑物理 USB 插口。

| 功能 | 1 号：展示简化版 | 0 号：完整功能版 |
|---|---|---|
| 行驶 / 转向 | 左摇杆 / 右摇杆横轴 | 相同 |
| 吸球 / 吐球 / 射球 | LT / LB / RB | 相同 |
| 方向键朝向吸附 | 上 / 下 / 左 / 右 | 相同 |
| 收 intake / 重置底盘朝向 | 无 / B | X / B |
| 电流送料射球 | 无 | A |
| 向外校准 intake 并归零射角 | 无 | Y |
| 送料机构局部反转 | 无 | RT |
| 循环 LED 主题 / 待机动画 | Back / Start | 无 LED 切换 |
| 重置仿真场地（实车无效） | 无 | Start |

方向键短按即启动自动朝向吸附（2026-10-28，见 `docs/decisions/0028`），目标角度如下；红方在蓝方目标上统一加 180°，保持相同的驾驶员相对方位，左右不会额外镜像：

| 方向键 | 蓝方目标 | 红方目标 |
|---|---:|---:|
| 上 | 0° | 180° |
| 下 | 180° | 0° |
| 左 | 90° | −90° |
| 右 | −90° | 90° |

松开方向键不会取消转向，命令在既有角度 PID 到达目标后结束并恢复默认摇杆驾驶；右摇杆横轴超过既有死区会立即取消吸附并恢复手动转向，来源手柄断连或失去优先权也会取消。此动作只旋转底盘，不重置位置或里程计。方向键上 / 下不再微调飞轮速度上限，方向键左不再收 intake。0 号 X 现在只执行向内收回及机械归零校准，1 号 X 未绑定；`Manual` 命令本体仍保留，但已无任何手柄绑定，只能由代码直接调度。

1 号 Back 按 Marigold Bridge → Festival Night → Papel Fiesta → Alebrije Glow → Candle Memory 循环五套主题，Start 按 Static → Breathe → Slow Flow → Chase → Color Step 循环五种待机动画；每次按下只前进一项。两者直接操作现有 `LED Pattern` / `LED Idle Effect` chooser，手柄或 Elastic 任一端改变后，另一端同步显示实际选择，并沿用现有断电持久化。切换主题仍自动预选推荐动画，之后可用 Start 或 Elastic 覆盖；Off 与四个独立模式下 Back 进入 Marigold Bridge，Start 不改变任何选择。0 号 Back 未绑定，0 号 Start 仍仅用于仿真场地重置。

LT 为开关式（2026-10-26 起；2026-10-27 起锁存为独立意图，见 `docs/decisions/0023`）：按一下展开 intake 并开始持续吸球，再按一下 LT 停止吸球（intake 保持展开、不自动收回）；射球、Manual 等机构动作只是借用共用机构——动作结束后吸球**自动恢复**，锁存不受干扰；机器人禁用或控制手柄断连/失去优先权时同样停止且不会自行恢复；若停止时 LT 仍被按住，则必须先松开 LT 再重新按下才算一次新的按压（控制权恢复本身不算，2026-10-27 起由真实按键边沿判定，见 `docs/decisions/0022`）。按住 LB 吐球时若吸球仍在进行，会先停止吸球再开始吐球；松开 LB 后吐球停止，吸球不会自动恢复。LB/RT 反转为点动式：反转输出按当前控制手柄的按住状态实时重算——控制手柄按住 LB 时吸球滚轮、Indexer、Feeder 反转（RT 仅 Indexer/Feeder；RT 不打断进行中的吸球），松开、断连、失去优先权且接管手柄未按住同一键、或机器人禁用时归零；反转不占用子系统，因此与射球同时进行时反转写入会被射球循环覆盖（沿用原有行为，本轮有意未改）。

## 行驶起步对齐与直行航向保持

（2026-10-28，见 `docs/decisions/0029`）从真正静止开始的**平移**指令（含满速起步）会让四个转向轮先转到目标角度、驱动电机暂缓输出；全部对齐（5° 容差，按驱动反向优化感知）后放行，最迟 1 秒无条件放行，转向故障不会卡死行驶。起步门只服务平移起步：**手动旋转绝对优先、永不延迟**——纯旋转起步、以及平移+手动旋转的组合起步都不经过门；行驶中改变方向（哪怕中间穿过一拍零指令）以实测车速判定"仍在行驶"，不加等待、仍走原有余弦缩放平滑过渡；机器人禁用时门状态复位，重使能后按当前实际静止状态重新判定（静止阈值 0.20 m/s / 0.40 rad/s，取自实测裕度，见 0029）。纯静止（无平移、无转向输入）不会主动转身。右摇杆横轴超出既有死区时手动转向绝对优先；回中且有平移输入时以陀螺仪闭环保持目标朝向（复用既有 `ANGLE` PID 常量）。松开右摇杆以当时新朝向为目标，禁止回弹旧方向；静止期间与命令重启（含禁用重使能）、B 键重置朝向后都以当前朝向为准，绝不复用旧目标。新增只读日志键 `Drive/StartupAlignHold`、`DriveCommands/HeadingHoldActive`、`DriveCommands/HeadingHoldTargetDeg`；死区、CANcoder offset、电机增益、NetworkTables 键与 Elastic 布局均未改动。桌面仿真已验证；实车转向响应、静止阈值与保持手感待实车确认。

## 演示面板与到速送料门控

在机器人已使能且按键来源手柄拥有控制权时，按住既有射球键会先把飞轮（Drum）与 Hood 驱动到 Elastic 目标，**首次送料前等飞轮实测转速到达限幅后目标 ±3.0 rot/s 并稳定 0.15 秒**（容差放宽原因见 `docs/decisions/0021`）：1 号/0 号 RB 的 `Shooting(false)`（普通速度模式）与 0 号 A 的 `Shooting(true)`（Indexer 与 Feeder 保持 50 A 电流模式）使用同一启动门控，避免满筐球被推到还在加速的飞轮上。`Manual` 命令仍保留相同的启动门控（按其自身 50 rot/s 目标判速），但目前无手柄绑定，只能由代码直接调度。**正常放行后持续送料（2026-10-27，见 `docs/decisions/0024`）**：每颗球造成的转速波动和非 0% 目标调整不再让送料反复停顿，Drum/Hood 继续追踪新目标；这不是全程逐拍到速判断。`Flywheel Speed Limit (%)` 调到 0% 仍当拍停料并撤销本轮放行，调回正值后重新到速、过稳定窗才恢复（已有超时锁存除外，且超时锁存永不越过 0%）。**3 秒超时强制射**：正值下启动或恢复时因未到速产生的累计等待满 3 秒仍未放行，本次按压内强制送料并锁存关闭转速门控直到松键重按（Elastic `Force Feed Timeout (s)` 默认 3 秒、0.5–10 秒、断电持久化，见 `docs/decisions/0022`）；送球、0% 和清堵期间暂停累计但不清零。Hood 位置不参与送料许可，只作日志诊断。射球路径不读取视觉、AprilTag、位姿或瞄准数据。松键、断连、失权、打断或禁用仍停止命令和送料；松键或禁用后 Drum 断电滑行，重新使能不会自行转起（见 `docs/decisions/0015`）。射球结束时 Hood 也不再被自动归零：松键或禁用后 Hood 保持最近的 Elastic 目标，使能空闲时由跟随命令持续施加最新目标（Manual 的独立目标与 Y 键归零不变，见 `docs/decisions/0025`）。

自动清堵的反向输出与时长保持不变（0.5 秒反向），但确认条件已从「仅看电流」改为「堵转证据」（2026-10-28，见 `docs/decisions/0027`）：定子电流持续高于 90 A 且实测转速接近零（|转速| < 5.0 rot/s 预览阈值）满 0.2 秒才确认 UNJAM，窗口内任一拍到得健康转速都会重置计时——转速处于 5.0 rot/s 及以上（|转速| ≥ 5.0 rot/s）时的高电流不再触发反向。0% 暂停和 UNJAM 期间 retract 计时仍冻结。清堵完成后命令经由同一转速门控恢复送料：飞轮若在堵转期间掉速，会先重新到速、过稳定窗再续打，不会把球立即推回可能已掉速的飞轮（若超时锁存已激活，则清堵后直接继续强制送料）。

`Robot Action`、`Feed Allowed` 和 `Feed Blocked Reason` 继续反映命令的实际送料决定；等待送球期间 `Robot Action` 显示 `Spinning Up`，阻断原因显示 `Flywheel not at speed` / `Stabilizing shot` / `Flywheel limit is 0%` 之一；超时强制后与正常送球一致显示 `Feeding`；`hoodReady` 仍按真实测量记录，仅作诊断。Elastic 演示页展示以下只读操作状态：

- `Robot Action` / `Feed Allowed` / `Feed Blocked Reason`
- `Drum Target Speed (rot/s)` / `Drum Actual Speed (rot/s)`

卡顿相关的两处修复属不同故障路径，均需重新 deploy 后做实车验证：`docs/decisions/0024`（#74）取消正常放行后的逐拍到速门控，消除门控引起的送料停顿；`docs/decisions/0027` 修正 UNJAM 的判定证据——此前 Indexer 仍在转动时的持续高电流也会被判为堵转并触发 0.5 秒反向，现须电流 >90 A 且 |转速| < 5.0 rot/s 持续 0.2 秒才确认。两者均不宣称消除所有机械堵转或所有可能的卡顿：大于或等于 5.0 rot/s 的缓慢爬行堵转不会自动清堵（需 0%、松键或手动反转处理）；5.0 rot/s 为预览阈值，实车电流/转速曲线尚未采集。重新 deploy 后应以相同球量复测 25% 与 50%：健康转速（|转速| ≥ 5.0 rot/s）下的高电流不再触发 UNJAM、满足判定证据的真实堵转（高电流且转速低于 5.0 rot/s）仍能清堵、无反复停走；若仍卡顿，检查是否反复出现 `Clearing jam` 并核对实车日志。该实车验收完成前，仅宣称软件修复与本地验证完成。

## Elastic LED 预览

`elastic-layout-1027.json` 的 `LED Preview` 页提供独立于实灯输出的配色与动画预览。使用时必须正在运行机器人代码或桌面仿真，并将 Elastic 连接到该进程的 NetworkTables；仅打开布局不会产生预览数据。

- 顶部 `LED Pattern` 与 `LED Idle Effect` 沿用现有设置键。大的 `Current Selection` 卡片预览当前 Pattern（主题 Pattern 使用当前 Idle Effect）；下方五张固定卡片分别展示 Marigold Bridge / Slow Flow、Festival Night / Breathe、Papel Fiesta / Chase、Alebrije Glow / Slow Flow、Candle Memory / Static 的主题与推荐效果（五套主题使用各自独立色板，可同时比较配色与推荐动画）。
- 所有卡片按固定参考亮度生成，不读取或显示实灯状态或实际亮度；`LED Brightness (%)` 不缩放预览，吸球、射球、吐球等机制动作也不会打断或替换预览。
- 预览用于比较主题与效果，不代表实灯颜色或亮度；屏幕颜色不能用来核验实灯色序、亮度、供电或分段。机器人 `Disabled` 时预览仍可用；`REPLAY` 不发布这些预览话题，因此回放不会填充此页。

## 构建与仿真

```bash
./gradlew build          # 编译 + spotless 格式检查
./gradlew spotlessApply  # 自动格式化（Google Java Format），提交前运行
./gradlew simulateJava   # 桌面仿真（maple-sim 物理）
./gradlew replayWatch    # AdvantageKit 日志回放
./gradlew deploy         # 部署到 roboRIO
```

仓库提供 GitHub Actions 构建工作流，配置为在 push / pull request 时运行 `./gradlew build`；实际是否运行取决于本 fork 的 Actions 启用状态。提交前需在本机完成构建与格式检查。

## 代码架构

- `frc.lib` — 团队通用硬件抽象层：`MotorIO`（电压 / 速度 / 位置 / 力矩电流控制，含 TalonFX 与仿真实现）、`SensorIO`（CANrange）、`MotorSubsystem` 基类；IO 输入以**弧度**记录日志，对子系统 API 以**圈数**暴露。
- `frc.robot.subsystems` — 单例模式（每个子系统 `public static final mInstance`），`X.java` + `XConstants.java` 成对（CAN ID、增益、仿真配置）。
- `frc.robot.commands` — 每个命令独立目录，各带 `*Constants.java`。
- 三种运行模式由 `Constants.currentMode`（REAL / SIM / REPLAY）切换，各子系统按模式选择 IO 实现。
- `elastic-layout-1027.json` — Elastic 仪表盘布局，与代码内 NetworkTables 接口同步维护。

## 开发流程与决策记录

- **提交约定**：遵循 [Conventional Commits](https://www.conventionalcommits.org/)——`feat:` / `fix:` / `docs:` / `chore:`，可带 scope（如 `feat(drive): ...`）。
- **决策记录**：重大改动的原因（为何 demo 只保留手动、为何 LICENSE 常驻 main 等）见 [`docs/decisions/`](docs/decisions/)——每个决策一个短文件（日期 + 状态 + 背景 + 决定）。

## 致谢与参考

- [FRC 8810 Alphabots](https://github.com/Alphabots-8810/8810-2026-offseason) — 本项目所基于的原始机器人与上游代码；感谢其开源工作
- [WPILib](https://docs.wpilib.org) — FIRST® 机器人软件框架
- [AdvantageKit](https://docs.advantagekit.org) / [Team 6328 Mechanical Advantage](https://github.com/Mechanical-Advantage) — 日志与回放框架、swerve 模板出处
- [maple-sim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim)（Shenzhen Robotics Alliance）— 2026 REBUILT 场地物理仿真
- [CTRE Phoenix 6](https://v6.docs.ctr-electronics.com) — TalonFX / Pigeon 2 / CANrange
- [PathPlanner](https://pathplanner.dev) · [Choreo](https://choreo.autos) — 路径规划与轨迹（`main` 线）
- [Autopilot](https://github.com/therekrab/autopilot) — Trench 自动通过运动库（`main` 线）
- [Elastic Dashboard](https://github.com/Gold872/elastic-dashboard) — 操控端仪表盘
- [FIRST® Robotics Competition](https://www.firstinspires.org/robotics/frc/game-and-season) — 2026 REBUILT 赛季与游戏手册

## 许可

本项目采用 [MIT License](LICENSE)，保留其中现有的 FRC Team 8810 Alphabots 版权声明。该声明用于保留来源归属；本衍生版本由 FRC 12363 LEOMOTIVE 维护和继续开发。依赖协议另见 [WPILib-License.md](WPILib-License.md) 与 [AdvantageKit-License.md](AdvantageKit-License.md)。
