# SPI —— 学习与配置记录

> 分工说明：**§1 配置记录**是本次 CubeMX 实际做了什么、对应到哪段代码（可直接作为作业的"配置过程"）；
> **§2 学习记录**按作业问题作答，由我自己写（照抄资料没有意义，老师看的是我自己的理解）。
全双工是指接收端和发射端可以同时接收和发送，半双工是接收端和发射端端不能同时接受和发送但是是可逆的
SPI是全双工，一共有四条信号线，MOSI和MISO两条线信息互传，SCK是时钟信号，CS是片选信号用来选特定从机的
一共有四个工作模式，由两个参数决定，CPOL时钟极性，CPHA时钟相位

---

## 1. 配置记录（SPI1 —— BMI088 六轴 IMU）

### 1.1 为什么是 SPI1、引脚为什么这么定

C 板上 BMI088 挂在 **SPI1** 上，且加速度计和陀螺仪是**两颗独立的片选**（可以看作同一根 SPI 总线上的两个从机）：

| MCU 引脚 | 片上信号 | 在本作业里的作用 |
| --- | --- | --- |
| PB3 | SPI1_SCK | 时钟，由主机（STM32）发出 |
| PB4 | SPI1_MISO | 从机 → 主机，读寄存器数据走这根 |
| PA7 | SPI1_MOSI | 主机 → 从机，写地址/写配置走这根 |
| **PA4** | GPIO 输出 | 加速度计片选 `CS1_Accel`，**低电平有效** |
| **PB0** | GPIO 输出 | 陀螺仪片选 `CS1_Gyro`，**低电平有效** |

两点关键：
- **MISO 是 PB4，不是 CubeMX 默认的 PA6** —— C 板原理图把 IMU 的 MISO 走到了 PB4，选错引脚就完全读不到数据。
- **片选不用硬件 NSS，改成两个普通 GPIO** —— 因为一根 SPI 上挂了两个器件，硬件 NSS 只能管一个；软件片选可以精确控制"这一帧我在跟谁说话"。
  上电时两个 CS 都置 **High（不选中）**，避免两颗芯片同时驱动 MISO 打架。

> ⚠️ SCK 在 PB3 是本次配置的结果（PB3/PB4 都是 SPI1 的 AF5 引脚）。上板若读不到芯片 ID，第一件事就是拿原理图核这一根。

### 1.2 CubeMX 参数与依据

`Connectivity → SPI1`，生成的初始化函数是 `Core/Src/spi.c` 里的 `MX_SPI1_Init()`，句柄 `hspi1`。

| 参数 | 配置值 | 为什么是它 |
| --- | --- | --- |
| Mode | **Full-Duplex Master** | 读寄存器要同时发地址、收数据，必须全双工 |
| Hardware NSS Signal | **Disable** | 片选交给 PA4 / PB0 两个 GPIO 软件控制 |
| Frame Format | Motorola | 标准 SPI 帧格式 |
| Data Size | 8 Bits | BMI088 寄存器是 8 位的，一帧一字节 |
| First Bit | MSB First | 芯片数据手册要求高位先出 |
| **Clock Polarity (CPOL)** | **High** | 空闲时 SCK 为高 |
| **Clock Phase (CPHA)** | **2 Edge** | 第二个边沿采样 |
| → 等效 SPI 模式 | **Mode 3** | BMI088 支持 Mode 0/3，官方例程用 Mode 3 |
| Baud Rate Prescaler | **16** | SPI1 挂在 APB2 = 84 MHz，84/16 = **5.25 MHz**；BMI088 上限 10 MHz，取一半留余量 |

### 1.3 生成的代码在哪里

```
board/Core/Src/spi.c      MX_SPI1_Init()、HAL_SPI_MspInit()（时钟 + GPIO 复用 AF5）
board/Core/Inc/spi.h      extern SPI_HandleTypeDef hspi1;
```

### 1.4 我的驱动怎么用它（`modules/src/BMI088.cpp`）

```
写寄存器：拉低对应 CS → HAL_SPI_Transmit(地址 & 0x7F) → 写数据 → 拉高 CS
读寄存器：拉低对应 CS → HAL_SPI_TransmitReceive([地址 | 0x80, 0x00...]) → 拉高 CS
```

一个**必须记住的坑**：BMI088 加速度计的 SPI 读时序里，**第一个字节是无效的**（芯片在时钟前沿就把数据推出来了），所以：

```
发送 [0x80 | reg, 0x00, 0x00]  →  收到的 data[0] 是空字节，真正的数据在 data[1], data[2]
```

陀螺仪的读时序没有这个问题，`data[0]` 就是第一个有效字节。这也是我代码里 `ReadReg` 对加速度计特殊处理长度的原因。

### 1.5 验证状态

- [x] 编译链接通过（`board/` 全量编译 `exit=0`）
- [x] `hspi1` 的 CLKPolarity / CLKPhase / BaudRatePrescaler / NSS 与上表一致（从生成的 `spi.c` 读出核对）
- [ ] 上板：读到加速度计/陀螺仪的芯片 ID（CHIP_ID 校验通过）
- [ ] 上板：三轴数据与静止时的重力（≈9.8 m/s²）对得上

---

## 2. 学习记录

### 2.1 全双工、半双工通信分别指的是什么？
全双工是指接收端和发射端可以同时接收和发送，半双工是接收端和发射端端不能同时接受和发送但是是可逆的


### 2.2 什么是 SPI？SPI 通信具有什么特点？

<!-- 提示：四根线各是什么、谁做主谁做从、没有地址靠什么选从机、有没有应答机制、速率大致多少、一主多从时引脚怎么变多 -->
SPI是全双工，一共有四条信号线，MOSI和MISO两条线信息互传，SCK是时钟信号，CS是片选信号用来选特定从机的
### 2.3 和 I²C 比，SPI 快在哪、贵在哪？

<!-- 提示：线数 vs 速率；无地址 vs 有地址；无应答 vs 有 ACK；点对点 vs 总线挂多设备 -->
iic是两根线，spi是4+n根线，需要很多从机的话就需要多接线，iic是半双工并且有应答机制，而且协议复杂，spi是全双工