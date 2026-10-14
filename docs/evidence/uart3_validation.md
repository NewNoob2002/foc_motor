# UART3 console 验证（2026-09-05）

状态：配置、Host/ARM 构建、J-Link 连接、烧录回读和寄存器检查通过；早期同步捕获为 0 B。已有 `boot4` 原始记录离线复核通过（1262 B），原 FAIL 为检查脚本正则错误；本轮尚未重新烧录或复位。

## 原理图与配置

- 已逐图核对 `Schematic_浩盛单路电机开发板V2.0.pdf` 的 U3、H2、U26。H2/6 = PB10/USART3_TX，H2/5 = PB11/USART3_RX，H2/1 = DGND。转接器 RXD 接 H2/6、TXD 接 H2/5；用户确认使用 3.3 V TTL，DCBUS 断开，仅逻辑供电。供电限流数值未提供，不宣称已测量。
  - 2026-10-14 更正：上述“H2/1 = DGND”与原理图不符。H2-1 接 +3.3V（本板实测对地 +3.34 V），DGND 在 H2-4，见 `HW-UART-003`。本条保留原记录；当时转接器 GND 的实际接点未再核实。
- console、shell-uart 均选 USART3，PB10/PB11 AF7，921600 8N1，无流控；USART1 从 SoC 继承 disabled。`CONFIG_SHELL` 仍未启用，本次验证目标为 console/日志输出，不是交互命令行。
- 保留 deferred logging、UART async TX 和 DMA1 Channel 3（selector 2），改为 USART3 IRQ 39、DMAMUX request 29，普通 IRQ 优先级 5。编译期断言覆盖 chosen、USART1 禁用、波特率、DMA、安全 GPIO 和 TIM1 禁用。

## 构建

- 固件源码：`0d230f371693328741ab0b6a5fc8196fe3becaa2` 加本次工作区修改；实际固件以以下哈希标识。
- 本机 Zephyr 已变为 `c199f92c7e4bba820573d6be9ba0c75385601b67`，且缺少旧提交对象；未修改该 checkout。独立下载项目锁定提交 `8dafb9a897da0f79a1a3724a108b21fc28719915` 到 `/tmp/foc-uart3-baseline/zephyr`，创建本地 west workspace 构建。
- CMSIS、CMSIS_6、STM32 HAL 仍分别匹配 M1 固定提交 `512cc7e…`、`b2dfbe1…`、`33576ef…`，通过 `ZEPHYR_MODULES` 显式引用本机目录。SDK 1.0.1 / ARM GCC 14.3.0 / west 1.5.0。
- Host CTest：1/1 通过（原有 warnings-as-errors、ASan/UBSan）。ARM 构建成功，Flash 36,916 B、RAM 6,656 B；原有 CI artifact、安全、栈和无动态内存门禁在本地全部通过。未触发远程 CI。Ccache 4.9.1 不满足构建要求，构建未使用它。

```sh
cd /tmp/foc-uart3-baseline
env ZEPHYR_BASE=/tmp/foc-uart3-baseline/zephyr \
  PATH=/home/gtc/zephyrproject/.venv/bin:/usr/local/bin:/usr/bin:/bin \
  CCACHE_DISABLE=1 /home/gtc/zephyrproject/.venv/bin/west build \
  -p always -b foc_motor \
  -d /home/gtc/Desktop/workspace/STM32_PROJ/foc_motor/build/uart3 \
  /home/gtc/Desktop/workspace/STM32_PROJ/foc_motor -- \
  -DBUILD_VERSION=v4.4.0-12700-g8dafb9a897da \
  '-DZEPHYR_MODULES=/home/gtc/zephyrproject/modules/hal/cmsis;/home/gtc/zephyrproject/modules/hal/cmsis_6;/home/gtc/zephyrproject/modules/hal/stm32' \
  -DZEPHYR_SDK_INSTALL_DIR=/home/gtc/zephyr-sdk-1.0.1 \
  -DUSER_CACHE_DIR=/tmp/foc-uart3-cache
```

| 产物 | SHA-256 |
|---|---|
| `build/uart3/zephyr/zephyr.hex` | `b3aafe8c303a18a52a2e05e3f7ab7a413b8142e810c6347b6e462e2fa70bedfb` |
| `build/uart3/zephyr/zephyr.bin` | `d1551241a7bdd8b7d8a010738ed7e2bda9405782d589e8a92c54d285cf143db3` |
| `build/hil/uart3/preflash.bin`（128 KiB 恢复点） | `28fdde69695716c7dcc660eefd75b6dd4f78a47b87bd33eb09849d6ab81bee2a` |

## J-Link 和目标证据

- J-Link Commander/DLL V9.70，探针 S/N `63728769`，SWD；独立校验与复位显式指定 1 MHz。MCP 初始设备误设为 `HC32F460xE`，在任何目标操作前改为 `STM32G431RB`。
- 连接返回 Cortex-M4 r0p1，DBGMCU ID `0x20036468`，Flash size `0x0080` KiB；UID 三个 32-bit 字 `001D0050 42535013 20363130`，用作本次板卡标识。VTref 3.375–3.378 V；未独立读取 PCB 修订号。
- 原 Flash 全部 131,072 B 备份并 `verifybin` 成功后，通过 J-Link MCP 烧录 HEX。首次 MCP 校验只返回启动信息，未计作成功；随后直接 Commander `verifybin` 读取 36,916 B，明确 `Verify successful.`，exit code 0。未 mass erase 或改 option bytes。
- 当前固件已通过 SYSRESETREQ + `g` 启动，保留无功率状态。恢复点未演练回滚；需要回滚时在同样无母线条件下经 SWD 写回备份、校验后再复位。

| 寄存器 | 实测 | 结论 |
|---|---|---|
| USART3 CR1 / BRR | `0x0000000d` / `0x000000b8` | USART3 收发已启用，921600 配置 |
| GPIOB AFRH | `0x00007700` | PB10/PB11 AF7 |
| DMAMUX C2CR | `0x0000001d` | USART3_TX request 29 |
| DMA1 Channel 3 CNDTR / CPAR | `0` / `0x40004828` | TX 搬运结束，目标为 USART3 TDR |
| USART3 TDR | `0x0a` | 最后写入换行；不是物理波形证据 |
| RCC APB1ENR1 / APB2ENR | `0x10040400` / `0x00000001` | USART3EN=1，USART1EN=0，TIM1EN=0 |
| GPIOA MODER / ODR | `0xabffffd5` / `0` | PA0/PA1/PA2 输出低 |
| GPIOB MODER / ODR | `0xffaffebd` / `0` | PB0 输出低 |

## 串口结果与待办

- 主机 `/dev/ttyUSB0` 枚举为 CH340 `1a86:7523`，by-id 为 `usb-1a86_USB_Serial-if00-port0`，无独立串号；仅凭这个名称不能证明其接的是排针 USART3。
- `boot1`、`boot2` 捕获为 0 B；`boot2` 结束于 14:01:36 UTC，复位日志完成于 14:01:48 UTC，错过采集窗口，不能用它判定链路。
- `boot3` 使用同一测试进程先打开串口并确认 READY，再执行 J-Link SYSRESETREQ/运行；35 秒内仍为 0 B，因此端到端日志验证失败。
- 向主机串口写一个 `0x55` 后，USART3 RDR 读为 `0x00`，GPIOB IDR=`0x410`（PB11/RX 低），尚未证明该字节到达 USART3。需要核对外接转接器身份、TX/RX 交叉和共地，调整后重跑。
- 可复跑：`python3 build/hil/uart3/run_boot_test.py boot5`（先确认该编号未使用，避免覆盖证据）。此脚本会复位目标，执行前仍需补齐供电限流、急停及恢复预检。通过条件为 banner、安全消息及 `seq=0..15` 各一次、无非 ASCII 字节和错误日志；保留 `boot4` 及其离线复核记录。
- 本次没有示波器/逻辑分析仪波形，不能关闭 SD 实际引脚时序风险 R-18。

原始证据和本地测试脚本保存在 `build/hil/uart3/`，固件在 `build/uart3/zephyr/`；受控日志副本见 `docs/evidence/uart3/`。

## 接线修改后的记录复核（2026-09-05）

- 本轮确认主机 `/dev/ttyUSB0` 为 CH340；沙箱内设备节点不可见，沙箱外只读枚举成功。J-Link MCP 检查显示探针可用、目标配置为 `STM32G431RB`，未执行目标复位、烧录或暂停。
- 固定 Zephyr 基线的 `west build -d build/uart3` 成功，Ninja 报告无待构建项；HEX/BIN 和 128 KiB 恢复镜像哈希仍与上表一致。
- 已有 `boot4` 在 14:15:34–14:16:09 UTC 捕获 1262 B，包含正确 banner、安全消息及 `seq=0..15` 各一次，无非 ASCII 字节或 `<err>`。原脚本使用 `(d+)` 匹配数字，导致序列为空、错误判 FAIL；已修正本地 `capture.py` 为 `([0-9]+)`。
- 直接执行修正后的脚本检查表达式复核原始字节通过；重复/缺失序列、非 ASCII、错误日志、缺失 banner 四类反例均拒绝。原 `boot4.json` 的 FAIL 保留，复核另存 `boot4_reanalysis.json`，没有将离线复核冒充新一次 HIL。
- 受控副本：`docs/evidence/uart3/boot4.raw`、`boot4.json`、`boot4_reset.log`、`boot4_reanalysis.json`。原始字节 SHA-256：`bc3ea11bcd7ea359021ca53e98794ec48b7c8da05c1c930084a8c53d7fdf0016`。
- 新一轮烧录/同步复位捕获尚待补齐供电电压、限流设定及急停/立即断电方式。原预检记录了无 DCBUS、仅逻辑供电及 3.3 V TTL，但未记录限流数值；不能仅凭旧 `approved: true` 认定项目安全条件完整。R-18 仍缺真实引脚波形。
