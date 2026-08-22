# M1 Build Evidence

状态：本地构建、目标识别、烧录校验、无功率寄存器检查和板载控制台通过；远程 CI 与真实引脚电平验证待完成。记录日期：2026-08-22。

## 1. 固定基线

| 组件 | 固定版本/提交 |
|---|---|
| Zephyr | `4.4.99/main` / `8dafb9a897da0f79a1a3724a108b21fc28719915` |
| CMSIS | `512cc7e895e8491696b61f7ba8066b4a182569b8` |
| CMSIS_6 | `b2dfbe1a20bbd49c2d2c605073799671074bbb30` |
| STM32 HAL | `33576ef05e529cad803f210cc95b52b607757c96` |
| west | 1.5.0 |
| Zephyr SDK | 1.0.1 |
| ARM GCC | 14.3.0 |
| Host GCC | 13.3.0 |

本地构建直接使用现有 west workspace。`west.yml` 保留给 CI，并在 workflow 中检查 Zephyr checkout 必须等于表中提交；manifest 只导入当前构建需要的 CMSIS、CMSIS_6 和 STM32 HAL。

## 2. 构建结果

| 门 | 结果 | 证据 |
|---|---|---|
| Host C11 configure/build | PASS | GCC 13.3，`-Wall -Wextra -Werror`，ASan/UBSan |
| Host CTest | PASS | `1/1` tests passed |
| ARM pristine build | PASS | board `foc_motor/stm32g431xx`，Zephyr build `v4.4.0-12700-g8dafb9a897da`，零编译 warning |
| 输出产物 | PASS | ELF、HEX、BIN、map、`.config`、generated DTS 均非空 |
| Flash 预算 | PASS | Zephyr 报告 36,912 B；门限 98,304 B；物理 131,072 B |
| RAM 预算 | PASS | Zephyr 报告 6,656 B；门限 24,576 B；物理 32,768 B |
| 动态内存入口 | PASS | ELF 不含 `malloc/calloc/realloc/free/k_malloc/k_calloc/k_heap_alloc`；`CONFIG_HEAP_MEM_POOL_SIZE=0` |
| 安全 devicetree | PASS | generated DTS 有且只有 4 个 `output-low` GPIO hog；TIM1/PWM disabled |
| UART DMA devicetree | PASS | USART1=921600；DMA selector 2（DMA1 Channel 3）；DMAMUX request 25；USART1/DMA IRQ priority 5 |

静态线程栈预算：

| 执行上下文 | 配置 | 预算 |
|---|---|---:|
| Main | `CONFIG_MAIN_STACK_SIZE` | 512 B |
| Idle | `CONFIG_IDLE_STACK_SIZE` | 320 B |
| ISR | `CONFIG_ISR_STACK_SIZE` | 1024 B |
| System workqueue | `CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE` | 512 B |
| Log processing | `CONFIG_LOG_PROCESS_THREAD_STACK_SIZE` | 768 B |
| 合计 | 不含对齐和内核对象元数据 | 3136 B |

CI 固定检查以上五项、`CONFIG_HW_STACK_PROTECTION=y` 和 `CONFIG_HEAP_MEM_POOL_SIZE=0`。M1 只建立静态预算；运行时 high-water mark 由 M2 在真实周期和中断负载下测量。

Host 命令：

```sh
cmake -S tests/host/foc -B build/host -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

ARM 命令契约：

```sh
cd /home/gtc/zephyrproject
west build -p always -b foc_motor \
  -d /home/gtc/Desktop/workspace/STM32_PROJ/foc_motor/build/arm \
  /home/gtc/Desktop/workspace/STM32_PROJ/foc_motor
```

## 3. 当前产物

| 产物 | SHA-256 |
|---|---|
| `build/arm/zephyr/zephyr.elf` | `78f6b742c31945f28f90fb578e56f1c72c4baacce7cc173dc40e7a54676c0702` |
| `build/arm/zephyr/zephyr.hex` | `bcb70b638dfb33e0c7b054fca26258b5fc0c9131804120c614819c1d004b9837` |
| `build/arm/zephyr/zephyr.bin` | `c13f32d185895d6546937f28f3e7fd1c86358fa2f443cde1a38d47a2fc7b7b75` |
| `build/arm/zephyr/zephyr.map` | `841bbe636a2ad8ffc02f94131c54a3c8eb27084e94246c41142f3cc5f4f9ac5b` |

构建目录被 `.gitignore` 排除；CI 上传相同六类产物用于审计。

## 4. 目标板证据

| 项目 | 结果 | 证据 |
|---|---|---|
| 探针 | PASS | Horco CMSIS-DAP FW 2.1.0，USB serial `3844331732`，SWD 1 MHz，无 NRST |
| 目标身份 | PASS | Cortex-M4 r0p1，DBGMCU ID `0x20036468`（STM32G43/G44），RDP0，128 KiB single-bank Flash |
| 烧录/校验 | PASS | `west flash --runner openocd --verify -i 3844331732` 写入并校验 36,912 B；未 mass erase，未修改 option bytes |
| 原固件备份 | PASS | `build/hil/preflash_3844331732.bin`，131,072 B，SHA-256 `570aeeef0235cbae9218afdc5f033793667edda10f3c6aff175477a8e5a12be6` |
| 本次恢复点 | PASS | `build/hil/preflash_uart_dma_921600.bin`，131,072 B，SHA-256 `056e105fc9eab905048fe5f798de926099e15421d9eff9114445f3aac322a21a` |
| SD 运行态 | PASS（寄存器） | GPIOA MODER 低 6 bit 均为 output、GPIOA ODR=`0`; PB0 为 output、GPIOB ODR=`0`，对应 SD1-SD4 逻辑低 |
| TIM1 运行态 | PASS（寄存器） | RCC APB2ENR=`0x00004001`，USART1EN=1，TIM1EN=0 |
| 固件运行 | PASS | SYSRESETREQ 后 500 ms，PC=`0x08002cb2`，Thread mode，位于 Zephyr idle 路径 |
| USART1 DMA 配置 | PASS（寄存器） | CR1=`0x0000000d`，BRR=`0x00b8`（921600）；DMAMUX C2CR=`0x19`（request 25）；DMA1 Channel 3 CPAR=`0x40013828`（USART1 TDR） |
| USB-UART 直连 | PASS | ACM0 与 ACM1 杜邦线直连，115200 8N1 两个方向各 13/13 B 一致 |
| ACM1 板端控制台 | FAIL/PENDING | ACM1 接开发板后，两次 SYSRESETREQ 启动仍捕获 0 B；同时 PC 正常、PB6/PB7=AF7、CR1=`0x0d`、BRR=`0x05c4`、TDR=`0x0a`，待检查 PB6->RXD/GND 物理路径 |
| 板载 CH340 控制台 | PASS | Type-C `/dev/ttyUSB0`，921600 8N1；两次完整启动各收到 banner、安全消息和 `seq=0..15`，共 2520 B，无乱码、丢日志或 async backend 错误 |

本次原始证据：`flash_uart_dma_921600.log` SHA-256 `c3fe2d6b10cd918ece217a470c10f52cc8c6cf8a76306a7c85c7fa0e499ec556`；`uart_dma_921600.log` SHA-256 `b3ebe1fb0bb62f44924775536bb0b50bb597a7ce4e0f435ef8ab352053063e83`；`openocd_uart_dma_registers.log` SHA-256 `089ab15f6e37b91e3b7fe22a8694b62b1203e0f36c3e6425dca184db18ce38ef`。受控副本与说明位于 [`docs/evidence/m1/`](./evidence/m1/README.md)。

原始日志位于 `build/hil/`。一次额外的独立 `verify_image` 诊断触发了 OpenOCD 目标端 CRC 算法 double fault；该次结果不计入通过证据。随后通过 SYSRESETREQ 恢复，PC 再次确认在正常 idle 路径。烧录命令自身的写入和 verify 无错误。

## 5. 安全边界与剩余项

- `src/main.c` 编译期核对 `SD1=PB0`、`SD2=PA1`、`SD3=PA2`、`SD4=PA0`，并要求四者均为 `output-low` GPIO hog。
- M1 不启用 TIM1、PWM、ADC、ADC DMA、TIM3、SPI 或 CAN；仅启用 USART1 TX 诊断 DMA，不包含 Clarke、Park、PI、SVPWM 或任何 FOC 实现。
- `R-18` 阻塞 M1 关闭：SWD 寄存器证据确认 SD1-SD4 配置为输出低且 TIM1 时钟关闭，但尚未证明真实引脚电平。后续需用示波器/逻辑分析仪确认上电复位、SYSRESETREQ 和 Zephyr 启动期间 SD1-SD4 均为低，PC0-PC3 无 PWM，并归档原始波形。
