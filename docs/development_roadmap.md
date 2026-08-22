# Development Roadmap

状态：M0 已确认；M1 进行中。本地与远程 CI、目标识别、烧录校验、寄存器安全态和板载控制台已通过；仅真实引脚波形仍阻塞 M1 关闭，证据见 [m1_build_evidence.md](./m1_build_evidence.md)。

## M0 — 工程设计（已完成）

交付：可追溯硬件文档、Zephyr ISR/线程架构、软件分层、CI/测试契约、项目开发规则和风险清单。

退出条件：

- 用户确认 `docs/zephyr_architecture.md`。
- 明确电机/母线/电流/转速参数；CAN/CANopen 当前明确延期。
- 接受 R-02 在 M4 前阻断带电闭环，并接受 R-03 在 M2/M3 继续测量和设计。

## M1 — Zephyr 工程骨架与 CI（仍禁止 FOC 算法）

工作：

- 使用本机 `/home/gtc/zephyrproject` 当前 Zephyr `4.4.99/main` 提交 `8dafb9a897da0f79a1a3724a108b21fc28719915` 和 Zephyr SDK 1.0.1；环境升级必须显式更新基线和构建证据。
- 创建 STM32G431RBT6 自定义 board、devicetree、Kconfig 和最小 application。
- 建立 host CMake/CTest 空壳与 ARM `west build` workflow。
- 生成 ELF/BIN/map，建立 Flash/RAM/线程栈预算。
- 上电默认只验证 SD1–SD4 低和 D 相关闭，不输出驱动 PWM。

退出条件：host 与 ARM CI 可重复通过；复位和启动阶段门极保持关闭；heap size 为 0，且没有动态内存 API 引用。

## M2 — BSP 与无功率 HIL（仍禁止 FOC 算法）

工作：

- TIM1 中心对齐 PWM、同步影子更新和安全 disable。
- ADC2 定时触发、固定 `[IA, IC]` 序列和 DMA 周期搬运。
- TIM3 AB 编码器、Z 捕获；停机 SPI 诊断可选。
- DWT 周期/WCET 统计、故障锁存和 watchdog 健康链。
- 使用无母线或低压限流条件验证 PWM、ADC、DMA、Encoder。

退出条件：三项无功率 Hardware 测试有原始证据；所有周期可测量；GPIO/SD 软件安全态已验证。独立硬件关断允许继续作为 R-02 保持阻断，但在关闭前禁止进入 M4 带电闭环。

## M3 — PC 上的 FOC Kernel

开始条件：用户再次确认可以实现 FOC 数学。

工作：

- 冻结坐标、符号、角度、单位和归一化约定。
- 依次实现 Clarke、Park/inverse Park、PI、SVPWM 的最小纯 C 版本。
- 建立 host golden vector、边界、饱和、round-trip 和 sanitizer 测试。
- 在 ARM 上只做 build 与 WCET benchmark，不连接功率输出。

退出条件：host 测试全部通过；ARM WCET 预算通过；FOC Kernel 零 RTOS/板级依赖。

## M4 — 电流环集成

工作：

- 接入电流偏置/增益校准、两分流可观测窗口和第三相重构。
- 接入电角度、限幅、故障和 PWM 原子更新。
- 从低母线、限流、锁轴/空载的分级测试开始，逐步闭合 d/q 电流环。
- 验证 ISR 20 kHz WCET、抖动、丢周期和故障关断。

退出条件：在批准电气边界内稳定运行；零控制超期；过流、编码器故障和通信丢失均进入安全态。

## M5 — Control 与 CANopen（延期，当前不执行）

工作：

- 速度环、可选位置环、模式仲裁和参数提交。
- 基础 CANopen：NMT、Heartbeat、EMCY、SDO、PDO、可选 SYNC。
- 若已确认要求，再实现 CiA 402 状态机和对象字典。
- HIL 验证 PDO 时序、总线错误、bus-off、节点复位和通信 watchdog。

开始条件：用户明确恢复 CAN/CANopen 范围并提供合格 CAN PHY。退出条件：协议互操作测试通过；通信不能绕过安全状态机直接使能功率级。

## M6 — 工业化加固与发布

工作：

- 温度、母线、电流、编码器和电源故障注入；长时间运行和启停循环。
- EMC/ESD、热、振动和边界负载验证；确认编码器与 CAN 隔离等级。
- 静态分析、栈水位、map、覆盖率、SBOM/许可证和可追溯测试报告。
- 校准/参数版本化、受控升级、回滚和生产测试流程。

退出条件：所有 critical/high 风险关闭或正式接受；发布证据能追溯到固件、硬件和测试环境。

## 推荐的首个确认包

确认后只执行 M1，不提前实现 FOC：

1. Zephyr/west 自定义板骨架。
2. Host 与 ARM 两个 CI job。
3. 安全启动固件，所有 SD 保持低。
4. map/size 和静态线程栈预算；DWT 周期测量从 M2 的真实 BSP 周期开始。
