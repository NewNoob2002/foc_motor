# foc_motor 开发规则

## 当前阶段

- 2026-10-14 用户采用初学者路线图 `docs/plan/beginner_roadmap.md` 作为当前执行路径：Stage 0–7，每个 Stage 学一项技能并以可展示的结果结束。`docs/plan/development_roadmap.md` 保留为长期产品阶段门（M0–M6），Stage 与 M 节点的对应见初学者路线图第 5 节。
- Stage 6（不带电机供电的真实 FOC 中断）已完成，2026-10-15 用户确认。**Stage 7（首次带电）已完成，2026-10-15 用户确认（“stage 7 complete”）**，初学者路线图 Stage 0–7 全部完成；之后的工作（如齿槽前馈、M 节点）须用户另行确认并在本节记录日期。**Stage 8（齿槽转矩前馈）于 2026-10-15 经用户确认开始（“start the cogging feed-forward next”）**，分支 `stage/8-cogging-ff`；带电条件沿用 Stage 7（R-02 学习台架接受、15 V、500 mA、OCP、急停、USB 与 DCBUS 不同时连接、带电期间不暂停内核），每次烧录和带电仍需用户确认。Stage 7 进入时（2026-10-15，“confirm and go”）条件均已满足：R-02 已由用户决定（见下方 Stage 7 条目）；母线 15 V、急停为台架电源输出按钮；J-Link 供电已关闭，USB 与 DCBUS 不同时连接（`HW-PWR-003`）；Stage 7 固件 `f2460b5` 已烧录并通过无母线检查，R-18 已用该固件复测通过（2026-10-15，`docs/evidence/lab_notes/r18_boot_capture.md` run 2、`docs/evidence/lab_notes/stage7_first_power.md`）。进入下一个 Stage 前需用户明确确认，并在本节记录日期。
- 已完成：Stage 0（工具链、safe-boot 固件、CI、host FOC 学习）；Stage 1（J-Link 启动安全检查和万用表复位检查，2026-10-14，记录见 `docs/evidence/lab_notes/stage1_boot_check.md`；R-18 波形仍留到 Stage 7 前）；Stage 2（TIM3 x4 编码器读数，每圈 4096 count、方向正确、Z 每圈一次同一 count、无丢数，2026-10-14，记录见 `docs/evidence/lab_notes/stage2_encoder.md`；线束 Z 实接 U7-3/PB3）；Stage 3（ADC2 读 IA/IC/VBUS/TEMP，零电流偏置 IA 1963.5、IC 1997.4 count，噪声 SD 6.3 count，VBUS≈0，换算公式在代码和记录中，2026-10-14，记录见 `docs/evidence/lab_notes/stage3_adc.md`）；Stage 4（TIM1 中心对齐 20 kHz 内部运行，ADC2 [IA, IC] 由 TRGO2 在计数顶点触发，DMA 零延迟 ISR 周期 8500±25 cycle；示波器在 CH3N/PB9 上确认频率、占空比、中心对齐与更新事件，Stage 6 使用 RCR=1，2026-10-15，记录见 `docs/evidence/lab_notes/stage4_pwm.md`；R-18 已对固件 `1d39b8c` 关闭，见 `docs/evidence/lab_notes/r18_boot_capture.md`）；Stage 5（`lib/foc` 编入固件并开启 FPU，64 组固定向量与 host 结果一致（最大误差 1–2 个 float ULP），一次完整电流环 step 最长 1395 cycle = 8.2 us（预算 3400），sinf+cosf 最长 761 cycle，double sin+cos 最长 6553 cycle，2026-10-15，记录见 `docs/evidence/lab_notes/stage5_foc_on_chip.md`）；Stage 6（FOC ISR 在零延迟 DMA 中断中运行 `foc_current_step`，RCR=1 顶点更新，无 DCBUS、SD1–SD4 低；手转电机轴时占空比角度 = 7 × 机械角，216 个位置最大误差 0.41 count；7.4 分钟内 ISR 最长 2344 cycle = 13.8 us、0 超期；强制超期可检测，连续 3 次锁存故障（MOE=0、占空比 50 %），2026-10-15，记录见 `docs/evidence/lab_notes/stage6_foc_isr.md`）。
- 各 Stage 的允许范围：
  - Stage 1–4：仅逻辑供电（USB 或调试器 3.3 V），不接 DCBUS，SD1–SD4 保持低。Stage 1 只用 J-Link 对板上现有固件做暂停、单步、观察点、读寄存器和复位，不烧录。允许在 BSP 中实现 TIM3 编码器、ADC2 读数、TIM1 PWM 时基与 ADC 触发；TIM1 只在内部运行，见“安全默认值”。
  - Stage 5：允许把 `lib/foc` 编入测试固件（不接 ISR、不使能 PWM），做数值对比和 DWT 计时。
  - Stage 6：允许 FOC ISR 在不接 DCBUS、SD1–SD4 保持低的条件下运行。
  - Stage 7（首次带电）：2026-10-15 用户确认进入，同日确认完成。结果（`docs/evidence/lab_notes/stage7_first_power.md`）：自动参数辨识（mode 5：R 0.198–0.201 Ω、逆变器损失 V0 约 234 mV、L 69–71 uH、方向 −1、每次上电以正反旋转平均修正偏置）；电流环（mode 4，CORDIC、死区补偿，id 阶跃 90 % 0.10 ms）；速度环（mode 7，均值保持 ±200 rpm 内，已知限制：2 倍电频率 ±15–24 rpm 波动，疑为磁钢齿槽，待齿槽前馈）；FOC ISR 热路径在 CCM，带电最长 2122 cycle；安全停止：解除使能与强制故障（0x1）均已演示，切断电源时电机停止但 UART 无法观察到 VBUS 跳闸（逻辑供电先掉电，需示波器）。以下限制仍有效：按 `docs/evidence/lab_notes/stage7_first_power.md` 的首次带电步骤执行，遇到第一个异常即停；带电期间不得暂停内核、烧录或断开/重连 J-Link（DWT 会停止，ISR 间隔与超期检测失效）。进入前须满足“安全默认值”，用逻辑分析仪或示波器记录 R-18 的真实引脚波形，并由用户对 R-02 作出决定（先增加独立于 CPU 的硬件关断，或仅针对低压限流学习台架正式接受风险），记录在本文件和 `docs/plan/risk_list.md`。
    - 2026-10-15 用户选择 (b)：**仅为 Stage 7 学习台架接受 R-02**。条件：台架电源限流 500 mA、OCP 开启；母线 **15 V**（用户 2026-10-15 确认；`+12V`（12.6 V）稳压最低约需 14.1 V，`HW-PWR-002`）；电机固定、轴上无负载；有人值守，**台架电源输出按钮作急停**（用户 2026-10-15 确认）。限流不限制相电流，固件须限制电压幅值并在 ISR 中按 IA/IC 做软件过流关断。不适用于 M4 或更高电压/电流测试；R-02 对 M4 仍为阻断项。
    - 供电（`HW-PWR-003`）：J-Link 供电关闭，只用 SWD 和 VTref 检测（用户 2026-10-15）；USB 与 DCBUS 不得同时连接（`USB_5V` 经 0 Ω 直连 `+5V`）。PC0–PC3 只在 `+12V` 有效（VBUS 在启用窗口内）后才可驱动（R-19）。
- host FOC 学习继续：`lib/foc` 的纯 C 数学先以 host 测试和 `tests/host/foc/sim/` 仿真验证，说明见 `docs/study/foc_study_guide.md`。
- 硬件事实和证据编号见 `docs/hardware/hardware_reference.md`，架构基线见 `docs/design/zephyr_architecture.md`，路线图见 `docs/plan/beginner_roadmap.md`（当前执行）和 `docs/plan/development_roadmap.md`（长期阶段门），M1 构建证据见 `docs/evidence/m1_build_evidence.md`，未关闭风险见 `docs/plan/risk_list.md`。

## Zephyr 与实时性

- 禁止在 FOC ISR 中调用任何 Zephyr kernel API、Zephyr 设备 API、日志、队列、信号量、互斥量、内存分配或 CAN/SPI API。
- FOC ISR 只允许访问预配置寄存器/固定缓冲区、读取周期计数器、调用无 RTOS 依赖的 FOC Kernel，并写入 PWM 影子寄存器。
- FOC 数学模块必须是纯 C，不得包含 Zephyr、CMSIS、STM32 HAL/LL 或板级头文件；host 与 ARM 构建必须复用同一份源码。
- 禁止动态内存：不得使用 `malloc/calloc/realloc/free`、`new/delete`、`k_malloc/k_calloc/k_heap`。线程栈、CANopen 对象字典、消息缓冲区和遥测缓冲区全部静态分配。
- 所有周期任务和 ISR 必须可测量：记录开始、结束、周期、WCET、最大抖动和超期计数。FOC ISR 使用 DWT CYCCNT；实验构建可在 SD4 保持低电平时复用 D 相信号做示波器测量。
- 中断优先级数字越小优先级越高。FOC 采样 ISR 是唯一计划使用的零延迟中断；普通线程和普通驱动不得提升到同级。
- FOC ISR 的目标周期为 20 kHz（可校准范围 10–25 kHz），50 us 截止期内 WCET 不得超过 20 us，连续运行不得出现超期。

## 分层与依赖

- 依赖只能向下：Application -> Control -> FOC Kernel -> BSP -> Hardware。
- BSP 负责引脚、时钟、TIM1、ADC2/DMA、TIM3 编码器、关断和周期计数；应用层不得直接写外设寄存器。
- FOC Kernel 只接收数值输入并返回数值输出，不拥有设备、线程、时钟、日志或通信。
- ISR 与线程之间只使用固定大小、单写者的快照；不得用锁阻塞 FOC ISR。
- 引脚、外设实例和时钟选择放在 devicetree/Kconfig 或 BSP，不在 Control/Application 中硬编码。

## 安全默认值

- 复位后立即保持 SD1/SD2/SD3/SD4 为低；D 相在本项目中始终禁用。
- PWM 只能在 ADC 校准、编码器检查、故障检查和控制状态机全部通过后使能。例外：Stage 4–6 在不接 DCBUS 且 SD1–SD4 保持低时，允许 TIM1 在内部运行（`CCER=0`、`MOE=0`），作为 ADC 触发和 ISR 时基；SD 拉高或接入 DCBUS 时不适用。无 DCBUS 时 PC0–PC3 必须保持模拟/不驱动（或输出低）：EG2104 VCC 为 0 V，IN 绝对最大值为 VCC+0.3 V（`HW-PWM-009`、R-19，2026-10-14 发现并经用户选择“仅寄存器验证”）。需要波形时只能把 TIM1 输出放到不连接栅极驱动的引脚，须用户另行确认；2026-10-14 用户已确认 CH1–3 → PA8/PA9/PA10（未连接）和 CH3N → PB9（U7-4 排针，浮空），CH1N/CH2N 禁止（CH2N 可映射到 PB0 = SD1）。
- 软件故障处理不是功率级保护。带电闭环测试前必须具备独立于 CPU 的硬件关断路径，并验证其关断时间。
- 看门狗只能由健康监督逻辑喂狗；不得在 FOC ISR 或无条件周期任务中喂狗。
- 不得在没有限流电源、急停、目标板身份确认和恢复方案时执行功率测试或烧录。

## 构建与测试

- Host 构建必须在没有 Zephyr 环境的普通 CMake/CTest 环境中编译 FOC 数学模块，并以 warnings-as-errors 和 ASan/UBSan 运行。
- ARM 构建使用本机 `/home/gtc/zephyrproject` 当前基线：Zephyr `v4.4.2`（tag `v4.4.2` / commit `dccb09599635bdff17633fa7e9dab014b91dce90`）、Zephyr SDK 1.0.1 和 `west build`。升级该 checkout 前必须先更新并重新生成 M1 证据；CI 只构建/测试，不自动烧录。
- Clarke、Park、PI、SVPWM 的每个行为变更必须先有 host 测试；PWM、ADC、DMA、编码器的行为必须由目标板/HIL 证据确认。
- Hardware 测试必须记录板卡序列号、固件提交、工具版本、接线、供电限制、原始日志和示波器/逻辑分析仪证据。
- Host 测试不能替代 ARM 构建，ARM 构建不能替代 HIL。没有硬件证据时不得声称硬件已验证。

## 实现原则

- 使用 C11 和 `float32` 作为初始数值基线；只有 WCET 或数值证据表明不足时才引入定点实现。
- 当前不开发 CAN/CANopen；重新启动该范围时优先使用 Zephyr 已有子系统和已集成的 CANopenNode，不自研 RTOS、协议栈、测试框架或内存管理器。
- 不为未验证需求增加抽象、配置项或依赖；最小可验证实现优先。
