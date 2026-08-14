# RA4M2 SimpleFOC 无刷电机控制工程

基于 Renesas RA4M2、e2studio 和 FSP 的 SimpleFOC 移植工程。当前工程用于单个三相无刷电机的 AS5600 绝对角度反馈、三相 PWM 驱动、两相电流采样、位置/速度闭环、串口网页调参、OLED 显示和故障锁存。

> 这是面向当前硬件接线的开发工程，不是通用开发板示例。使用前必须确认电机、驱动器、编码器、电源和引脚连接与本文一致。

## 当前状态

- 目标 MCU：`R7FA4M2AD3CFL`，Cortex-M33，CPU 主频 90 MHz。
- IDE：Renesas e2studio，FSP 6.3.0，GCC Arm Embedded 工具链。
- PWM：25 kHz 对称三相 PWM，支持 SimpleFOC SVPWM。
- 编码器：AS5600，SCI4 I2C，地址 `0x36`。
- 电流采样：两相在线电流采样，U 相和 V 相。
- 控制模式：位置闭环、速度闭环；力矩层可选电压控制或 FOC 电流控制。
- 通信：SCI9 串口，115200 8N1；发送使用 DTC 分块传输和软件环形队列。
- 页面：[FOC_Dashboard_RA4M2.html](FOC_Dashboard_RA4M2.html)，通过浏览器 Web Serial API 与串口通信。

## 安全警告

1. 第一次上电请使用限流电源，并让电机处于可安全转动的状态。
2. `P302` 是驱动器使能，高电平使能、低电平失能。P000 按键下降沿会立即拉低 P302。
3. 当前启动默认使用已保存的传感器方向和零电角度，**不会进行原生 SimpleFOC 自动对齐扫描**。
4. 原生自动对齐曾在当前硬件上出现持续驻停和发热风险，网页完整校准入口已锁定。未确认 PWM、相序、编码器安装和功率级安全前，不要开启该路径。
5. 发生故障后必须复位单片机再恢复输出；不要通过软件强行解除故障锁存。

## 硬件与引脚

| 功能 | MCU 引脚 | 外设/通道 | 说明 |
|---|---|---|---|
| PWM U 相 | P108 | GPT0 GTIOC0B | 三相 PWM U |
| PWM V 相 | P104 | GPT1 GTIOC1B | 三相 PWM V |
| PWM W 相 | P102 | GPT2 GTIOC2B | 三相 PWM W |
| 驱动使能 | P302 | GPIO 输出 | 高电平使能，低电平失能 |
| 用户急停按键 | P000 | ICU IRQ6 | 外部上拉、下降沿触发 |
| U 相电流 | P014 | ADC0 AN012 / ADC 通道 12 | INA240 输出，当前命名为 U 相 |
| V 相电流 | P013 | ADC0 AN011 / ADC 通道 11 | INA240 输出，当前命名为 V 相 |
| AS5600 SCL | P206 | SCI4 SCL4 | 开漏 I2C |
| AS5600 SDA | P207 | SCI4 SDA4 | 开漏 I2C |
| OLED SCL | P408 | IIC0 SCL0 | OLED 地址 `0x3C` |
| OLED SDA | P407 | IIC0 SDA0 | OLED 地址 `0x3C` |
| 上位机 TX | P109 | SCI9 TXD9 | 经板载 CH340/USB-C 连接电脑 |
| 上位机 RX | P110 | SCI9 RXD9 | 经板载 CH340/USB-C 连接电脑 |
| CAN TX | P103 | CAN0 CTX0 | 当前仅完成 XML 配置，尚未启用应用协议 |
| CAN RX | P402 | CAN0 CRX0 | 需要外置 CAN 收发器，不能直接接 CANH/CANL |

`P001`、`P002` 在 XML 中保留为 ADC 模拟输入，但当前电机控制程序未使用。`P112/P301/P100` 是遗留的 PWMB 配置，不是当前电机的三相输出。

## 外设配置

| 外设实例 | 配置 | 当前用途 |
|---|---|---|
| `g_three_phase_pwm0` | GPT0/1/2，25 kHz，对称 PWM，单缓冲，死区 0 | 三相 PWM 输出 |
| `g_adc0` | ADC0，12 位、右对齐、软件单次扫描、3.3 V 参考 | U/V 两相电流读取 |
| `g_i2c_as5600` | SCI4 I2C，约 100 kHz，7 位地址 `0x36` | AS5600 |
| `g_i2c_master0` | IIC0，约 400 kHz，7 位地址 `0x3C` | OLED |
| `g_uart9` | SCI9，115200、8N1 | 网页、日志、调参 |
| `g_transfer_uart9_tx` | DTC Normal，SCI9 TXI 触发，1 字节 | 串口发送搬运 |
| `g_external_irq6` | IRQ6，下降沿，优先级 2 | P000 紧急失能 |
| `g_can0` | CAN0，500 kbit/s，32 邮箱 | 已配置，暂未在应用中打开 |

当前 ADC 是软件扫描，尚未实现 PWM 中点触发、ADC-DTC 同步采样。后续进入高性能电流环时，建议采用 GPT 触发 ADC，再由 DTC/DMAC 将采样值搬入 RAM。

## 工程结构

```text
src/simplefoc_ra4m2_app.cpp   电机对象、PID/校准、主初始化和主循环入口
src/app/app_ui.inc            OLED 初始化、数值格式化和显示工具
src/app/app_safety.inc        P302 使能、P000 中断和故障锁存
src/app/app_sensor.inc        AS5600 与 ADC 启动校验
src/app/app_telemetry.inc     网页遥测帧、电流诊断帧和 OLED 运行页面
src/renesas_simplefoc_port.*  Arduino/SimpleFOC 到 RA FSP 的适配层
src/SimpleFOC/                SimpleFOC 源码
src/oled.*                    OLED 驱动
ra_cfg/                       FSP 配置头文件
ra_gen/                       e2studio/FSP 自动生成代码
configuration.xml             e2studio 图形化配置源文件
FOC_Dashboard_RA4M2.html      浏览器串口上位机
script/                       Python 位置环整定工具
```

`src/app/*.inc` 由 `simplefoc_ra4m2_app.cpp` 在同一个匿名命名空间中包含，不会生成额外目标文件，也不改变电机对象的全局状态、链接方式或初始化顺序。这样适合当前硬件排障阶段按职责阅读代码；等 FOC 校准和电流环稳定后，再将 PID、校准、串口命令拆成独立 `.cpp/.h` 模块会更合适。

不要手工长期修改 `ra_gen/` 中的外设配置。应在 e2studio 打开 `configuration.xml`，修改 Pins/Stacks 后重新生成代码；应用逻辑放在 `src/`。

## 构建与烧录

1. 用 e2studio 导入现有工程目录。
2. 确认目标为 `R7FA4M2AD3CFL`，FSP 版本为 6.3.0。
3. 打开 `configuration.xml`，确认引脚和外设实例没有被错误重置。
4. 选择 `Debug` 配置并执行 Build Project。
5. 使用当前板卡可用的烧录方式下载 `Debug/simplefoc1.elf`。
6. 打开串口 `115200`、8N1，复位后检查启动日志。

正确的安全启动日志应包含：

```text
FOC_BOOT_ALIGN:STORED_SENSOR
AS5600_VERIFY_RESULT,ok=1,good=32,error=0,...
RA4M2 SimpleFOC motor mode ready.
```

如果仍显示 `FOC_BOOT_ALIGN:SIMPLEFOC_SENSOR`，说明烧录的不是当前工程构建得到的固件。

## 上位机

使用 Chrome 或 Edge 打开 `FOC_Dashboard_RA4M2.html`，通过 Web Serial 选择 CH340 对应 COM 口，波特率设为 `115200`。

普通遥测帧格式固定为：

```text
target,actual
```

- 位置模式：目标角度与实际轴角度，单位 rad。
- 速度模式：目标速度与实际轴速度，单位 rad/s。

请保持该两列格式，网页波形依赖它。日志、应答与故障信息为单独文本行。

## 串口命令

所有命令以换行结束，成功后返回 `MCU_ACK: <命令>`。

| 命令 | 说明 |
|---|---|
| `NMD0` | 切换位置闭环，并把目标设为当前位置 |
| `NMD1` | 切换速度闭环，初始目标速度为 0 |
| `NTC0` | 使用电压力矩控制 |
| `NTC1` | 使用 FOC 电流力矩控制 |
| `NTG<value>` | 设置位置目标 rad 或速度目标 rad/s |
| `NAP<value>` | 设置位置 P |
| `NVL<value>` | 设置位置模式速度限制；速度模式下当前代码不使用此限制 |
| `NVP<value>` / `NVI<value>` / `NVF<value>` | 设置速度环 P / I / 低通滤波 Tf |
| `NCQP<value>` / `NCQI<value>` | 设置 q 轴电流环 P / I |
| `NCDP<value>` / `NCDI<value>` | 设置 d 轴电流环 P / I |
| `NCCF<value>` | 设置电流环低通滤波 Tf |
| `NCL<value>` | 设置最大电流限制，允许范围 0 到 1 A |
| `NDS0` / `NDS1` | 打开 / 静默普通遥测 |
| `NID0` / `NID1` | 关闭 / 打开低频相电流诊断帧 |
| `NAV0` / `NAV1` | 关闭 / 打开上电 AS5600 连续通信验证 |
| `NAV2` | 立即验证 AS5600，执行前关闭电机且验证后保持失能 |
| `NCE` | 打印当前 FOC 配置和故障码 |
| `NCP1` / `NCP0` | 进入 / 退出独立电流 PID 调试模式 |
| `NCR<value>` | 电流 PID 调试的 Iq 目标 |
| `NDBQ<value>` / `NDBD<value>` | 电流 PID 调试 Iq / Id 目标 |
| `NDBL<value>` / `NDBV<value>` | 电流 PID 调试电流限值 / 电压限值 |
| `NDBS<ms>` | 电流 PID 调试阶跃测试时长 |

`NCA1` 的完整 FOC 校准命令目前被安全锁定，会返回失败，不应作为正常使用路径。

## 故障码

故障为锁存状态，触发后会关闭 PWM 和 P302。串口格式：

```text
FAULT,code=0x00000002,latched=1
```

| 位 | 掩码 | 含义 |
|---:|---:|---|
| 0 | `0x00000001` | P000 用户急停 |
| 1 | `0x00000002` | AS5600 连续 3 次通信失败 |
| 2 | `0x00000004` | 电流采样初始化失败 |
| 3 | `0x00000008` | 三相 PWM 驱动初始化失败 |
| 4 | `0x00000010` | FOC 初始化失败 |
| 5 | `0x00000020` | 校准失败或校准中止 |

多个故障可按位叠加。后续 CAN 状态帧将直接复用这个 32 位故障字。

## Python 位置环整定

脚本位于 `script/ra4m2_position_tuner.py`，使用与网页相同的串口文本协议，默认只保守地调整位置 P。

```powershell
cd script
python -m pip install pyserial
.\run_position_tuner.bat COM3
```

脚本会在本地生成 `position_tuner_last_result.txt`，该日志已被 Git 忽略。调参后的值仅在 RAM 中有效，复位后会恢复 `src/simplefoc_ra4m2_app.cpp` 内的默认参数；确认结果后需手动写回源代码并重新烧录。

## CAN 多节点计划

项目已配置 CAN0：`P103=CTX0`、`P402=CRX0`、`500 kbit/s`，但尚未调用 `R_CAN_Open()`。

未来架构为电脑上位机 + USB-CAN + 多个独立 RA4M2 电机节点。每个节点本地运行 FOC，仅通过 CAN 接收位置/速度/使能命令并回传角度、速度、电流和故障字。

CANH/CANL 必须经过外置 CAN 收发器；总线物理两端各接一个 `120 Ohm` 终端电阻。建议先实现节点 ID、心跳超时失能、广播急停和状态帧，再进行多电机协同控制。

## 版本管理

构建产物 `Debug/`、`Release/`、Python 缓存和本地整定日志已在 `.gitignore` 中排除。提交外设配置变更时请同时提交：

- `configuration.xml`
- `ra_cfg/`
- `ra_gen/`
- 对应 `src/` 应用逻辑
