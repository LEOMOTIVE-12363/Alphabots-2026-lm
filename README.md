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
- **Elastic 自定义仪表盘** — 布局版本化于 `elastic-layout-1024.json`；配套 NetworkTables 调参脚本 `scripts/nt_tune.py`、`scripts/pid_analysis.py`

## 分支模型

| 分支 | 内容 |
|---|---|
| `main` | 上游 Alphabots 8810 竞赛代码基线，保留本 fork 的 MIT LICENSE、双语 README 与私有配置退跟踪改动：完整自动程序（Choreo 轨迹 + PathPlanner）、视觉、战术程序 |
| `demo` | **LEOMOTIVE demo 展示线**（本仓库的日常开发线）：移除全部自动程序与视觉，纯手动驾驶；Elastic 数值输入框直控 Drum 转速 / Hood 俯仰，并新增 demo 调参页；可寻址 LED 灯带（亡灵节五套主题配色 + 吸球/发球/吐球球路流光，主题 / 待机效果 / 亮度 Elastic 可切换且断电持久化；灯带分段与动画参数待实灯核验；CANdle 已选定为默认驱动，PWM 保留为编译期备用；改 `LEDConstants.HARDWARE` 一个常量即可切换，硬件安装参数与实灯表现仍待核验） |

- demo 展示线的功能改动**永不合入 `main`**；每个任务一个独立 PR，base 为 `demo`，自检通过后即合入。仓库共用的中英 README 同步维护：从 `demo` 发起文档改动，再以独立 PR 将相同文案同步至 `main`。
- 查看 `demo` 相对 `main` 的全部差异，请使用 GitHub Compare：[compare/main...demo](https://github.com/LEOMOTIVE-12363/Alphabots-2026-lm/compare/main...demo)（原先长期保持 open 的总览 PR #2 已于 2026-10-23 关闭，无须常驻 open PR）。

## demo 手柄分工

Driver Station 编号 **1 优先于 0**，优先级同时适用于驾驶和所有按键。只要 1 号连接，0 号输入就不生效；1 号断开后切换到 0 号，两者均断开时驾驶输入为零。编号是 Driver Station 槽位，不是电脑物理 USB 插口。

| 功能 | 1 号：展示简化版 | 0 号：完整功能版 |
|---|---|---|
| 行驶 / 转向 | 左摇杆 / 右摇杆横轴 | 相同 |
| 吸球 / 吐球 / 射球 | LT / LB / RB | 相同 |
| 收 intake / 重置底盘朝向 | 方向键左 / B | 相同 |
| 电流送料射球 / Manual 调参射球 | 无 | A / X |
| 向外校准 intake 并归零射角 | 无 | Y |
| 送料机构局部反转 | 无 | RT |
| 飞轮速度上限微调 | 无 | 方向键上 / 下 |
| 循环 LED 主题 / 待机动画 | Back / Start | 无 LED 切换 |
| 重置仿真场地（实车无效） | 无 | Start |

1 号 Back 按 Marigold Bridge → Festival Night → Papel Fiesta → Alebrije Glow → Candle Memory 循环五套主题，Start 按 Static → Breathe → Slow Flow → Chase → Color Step 循环五种待机动画；每次按下只前进一项。两者直接操作现有 `LED Pattern` / `LED Idle Effect` chooser，手柄或 Elastic 任一端改变后，另一端同步显示实际选择，并沿用现有断电持久化。切换主题仍自动预选推荐动画，之后可用 Start 或 Elastic 覆盖；基础灯效下 Back 进入 Marigold Bridge，Start 不改变任何选择。0 号 Back 未绑定，0 号 Start 仍仅用于仿真场地重置。

LT 同时展开 intake 并吸球，松开后停止吸球但不自动收回。方向键左执行向内收回及机械归零校准。LB/RT 反转为点动式：反转输出按当前控制手柄的按住状态实时重算——控制手柄按住 LB 时吸球滚轮、Indexer、Feeder 反转（RT 仅 Indexer/Feeder），松开、断连、失去优先权且接管手柄未按住同一键、或机器人禁用时归零；反转不占用子系统，因此与射球同时进行时反转写入会被射球循环覆盖（沿用原有行为，本轮有意未改）。

## 演示面板与立即送料

在机器人已使能且按键来源手柄拥有控制权时，按住既有射球键会立即送料：1 号/0 号 RB 的 `Shooting(false)`（普通速度模式）、0 号 A 的 `Shooting(true)`（Indexer 与 Feeder 保持 50 A 电流模式）以及 0 号 X 的 Manual，都在命令第一次 `execute` 就按各自原目标驱动 Indexer 与 Feeder；Drum 与 Hood 同时仍前往原目标，IntakeRoller 保持原运行逻辑。就绪许可只保留 `Flywheel Speed Limit (%) > 0` 这一项：0% 停止送料，调回任意正值的当拍恢复；Drum 转速未到或掉速、Hood 未到或偏离、目标中途改变以及 readiness debounce 都不再暂停、延迟或许可送料。这些射球路径不读取视觉、AprilTag、位姿或瞄准数据。松键、手柄断连、失去控制权、命令打断或机器人禁用仍会停止对应命令并停料。

自动清堵的阈值、确认、反向输出与时长保持不变；0% 暂停和 UNJAM 期间 retract 计时仍冻结。清堵完成后，只要命令仍在运行且飞轮速度上限大于 0%，就立即恢复原送料状态，不再等待转速/Hood 就绪或稳定窗。

`Robot Action`、`Feed Allowed` 和 `Feed Blocked Reason` 继续反映命令的实际送料决定；`drumReady` / `hoodReady` / `stableReady` 仍按真实测量记录，仅作诊断，不参与送料许可。Elastic 演示页展示以下只读操作状态：

- `Robot Action` / `Feed Allowed` / `Feed Blocked Reason`
- `Drum Target Speed (rot/s)` / `Drum Actual Speed (rot/s)`

以上立即送料、清堵恢复行为与面板尚未在实车上验证。

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
- `elastic-layout-1024.json` — Elastic 仪表盘布局，与代码内 NetworkTables 接口同步维护。

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
