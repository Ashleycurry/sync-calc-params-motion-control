# 双轴步进电机同步运动控制系统

[中文](README.md) | [English](README-English.md)

基于 STM32F103VET6 的双轴步进电机同步运动控制完整工程。固件通过 ESP8266 无线透传接收上位机运动指令，内置 **Sync_CalcParams 双轴同步算法** 与 **AVR446 梯形加减速状态机**，两轴可同时启停、精确到达目标位置；配套 Python 上位机提供指令下发、移动记录与实时轨迹可视化。

## 核心亮点

### 嵌入式软件开发

- 裸机前后台架构：中断服务程序负责实时脉冲输出与串口不定长帧接收，主循环负责指令解析与运动调度，全程无阻塞延时
- 双定时器独立中断通道（TIM3 / TIM4）驱动两轴脉冲输出，时基精度 1 µs，提供软件步进计数与位置查询接口
- USART3 + IDLE 空闲中断实现不定长指令帧接收；USART1 重定向 `fputc` / `printf` 输出调试信息
- ESP8266 AT 指令集驱动：DHCP 组网、TCP 透传、链路状态检测、检测到 `CLOSED` 上报后自动重连
- 模块化分层设计：bsp 外设层（esp8266 / usart）→ 通用工具层（common / core_delay）→ 应用层（main / sync_calc_params / microstep_driver）
- DWT 周期计数器实现微秒级精确延时（core_delay）；Keil MDK 工程开箱即用
- 统一 Doxygen 风格注释、UTF-8 编码，死代码清理与敏感信息脱敏

### 算法优化

- **Sync_CalcParams 双轴同步算法**：从轴速度 / 加速度 / 减速度按步数比例统一缩放，两轴三段运动时间完全对齐，同时启停（全整数运算，无浮点）
- **AVR446 整数递推梯形加减速**：首步延时、临界步数、加减速临界点全部整数化，自动规划"加速 → 匀速 → 减速"曲线
- 递推公式携带余数校正（`rest`），长时间运动无累计误差；无浮点、无查表，适合低端 MCU
- 32 位中间量防溢出设计 + 缩放结果下限保护（钳位为 1），覆盖悬殊步数与极端参数场景

## 系统架构

```text
┌───────────────────────────┐   TCP :5000    ┌───────────────────────────┐
│  Python 上位机            │ <============> │  ESP8266（TCP 透传）      │
│  指令下发 / 轨迹可视化    │                └─────────────┬─────────────┘
└───────────────────────────┘                              │ USART3 115200
                                                           v
┌──────────────────────────────────────────────────────────────────────┐
│                          STM32F103VET6                              │
│                                                                      │
│  USART3 中断（IDLE 帧判定）→ 指令解析（sscanf 8 参数）              │
│      → Sync_CalcParams 双轴同步算法（参数缩放）                      │
│      → MSD_Move1 / MSD_Move2 同时启动                                │
│                                                                      │
│  TIM3 中断 → 电机 1 脉冲（加速 / 匀速 / 减速状态机）                │
│  TIM4 中断 → 电机 2 脉冲（加速 / 匀速 / 减速状态机）                │
│                                                                      │
│  USART1 115200 → 调试信息打印                                        │
└──────────────────────────────────────────────────────────────────────┘
```

## 核心算法

### 1. Sync_CalcParams 双轴同步算法（参数比例缩放）

**问题**：两轴各自独立执行梯形加减速时，若步数不同，就会在不同时刻停止，无法同步到达。

**原理**：梯形速度曲线的关键时间参数为

```text
t_accel = speed / accel      （加速段时间）
t_decel = speed / decel      （减速段时间）
```

只要两轴的 `speed/accel`、`speed/decel` 相同，三段运动时间就完全一致。因此选出步数较多的轴为**主轴**（参数不变），**从轴**的速度与加/减速度统一乘以步数比例：

```text
ratio       = steps_slave / steps_master
speed_slave = speed_master × ratio
accel_slave = accel_master × ratio
decel_slave = decel_master × ratio
```

`ratio` 在时间公式中被约去：`t = (speed×ratio) / (accel×ratio) = speed / accel`，从轴在时间轴上与主轴完全对齐。步数相差多少，缩放比例就是多少，与方向（正负步数）无关。

**示例**：`2000 200 200 75 | 1000 200 200 75`

| 参数 | 电机 1（主轴） | 电机 2（从轴，原始） | 电机 2（同步缩放后） |
| --- | ---: | ---: | ---: |
| 步数 | 2000 | 1000 | 1000（不变） |
| 速度 | 75 | 75 | 75 × 0.5 = **37** |
| 加速度 | 200 | 200 | 200 × 0.5 = **100** |
| 减速度 | 200 | 200 | 200 × 0.5 = **100** |

电机 2 以一半的速度、一半的加速度走一半的路程，与电机 1 **同时出发、同时到达**。

**工程保护**（`sync_calc_params.c`）：

- 缩放乘法在 `unsigned long`（32 位）中间量中进行，防止 16 位乘法溢出
- 缩放结果下限钳位为 1，杜绝加速度 / 速度为 0 导致的除零或运动异常
- 步数为 0 的轴跳过同步计算；步数取绝对值比较，方向不影响同步

### 2. 梯形加减速（AVR446 整数状态机）

速度规划基于 Atmel AVR446 应用笔记的整数算法，全部采用整数乘除与平方根查算，无浮点、无查表：

```text
min_delay  = A_T_x100 / speed                            # 最高速度对应的最小脉冲间隔
step_delay = T1_FREQ_148 × sqrt(A_SQ / accel) / 100      # 加速度对应的首步脉冲间隔
max_s_lim  = speed² / (A_x20000 × accel / 100)           # 达到最高速度所需步数
accel_lim  = step × decel / (accel + decel)              # 开始减速的临界步数
```

定时器中断内以递推方式逼近理论速度曲线：

```text
new_delay = delay - (2 × delay + rest) / (4 × accel_count + 1)
rest      = (2 × delay + rest) % (4 × accel_count + 1)
```

余数 `rest` 携带到下一次迭代，使整数运算不产生累计误差。中断服务程序内维护五状态机：

```text
STOP ──> ACCEL ──> RUN ──> DECEL ──> STOP
  启动     加速段      匀速段     减速段     到达目标步数
```

每个脉冲下降沿完成一次状态评估与延时更新（软件步进计数），保证双轴脉冲互不干扰。

## 通信协议

上位机与固件之间采用**文本指令协议**：8 个整数、空格分隔、`\r\n` 结尾。

| 字段 | 含义 |
| --- | --- |
| `step1` / `step2` | 电机 1 / 2 运动步数（正负表示方向） |
| `accel1` / `accel2` | 电机 1 / 2 加速度 |
| `decel1` / `decel2` | 电机 1 / 2 减速度 |
| `speed1` / `speed2` | 电机 1 / 2 最高速度 |

示例：

```text
2000 200 200 75 1000 200 200 75
```

用户只需给出两轴各自步数与主轴速度参数，从轴参数由固件自动按比例缩放，实现同步运动。报文以 IDLE 空闲中断判定帧结束，无帧头帧尾开销。

## 硬件资源

| 功能 | 引脚 | 说明 |
| --- | --- | --- |
| 电机 1 脉冲 PUL | PA6 | TIM3_CH1 |
| 电机 1 方向 DIR | PA4 | GPIO 推挽输出 |
| 电机 1 使能 ENA | PA5 | GPIO 推挽输出 |
| 电机 2 脉冲 PUL | PB6 | TIM4_CH1 |
| 电机 2 方向 DIR | PB7 | GPIO 推挽输出 |
| 电机 2 使能 ENA | PB5 | GPIO 推挽输出 |
| ESP8266 串口 | PB10 / PB11 | USART3 透传，115200 |
| ESP8266 复位 / 使能 | PB9 / PB8 | RST / CH_PD |
| 调试串口 | PA9 / PA10 | USART1，115200 |

| 器件 | 说明 |
| --- | --- |
| MCU | STM32F103VET6（Cortex-M3，72 MHz，512 KB Flash） |
| 执行机构 | 微步细分步进电机驱动器 ×2（PUL / DIR / ENA 接口） |
| 无线模块 | ESP8266（AT 固件，STA 模式 TCP 客户端） |

## 目录结构

```text
sync-calc-params-motion-control/
├── README.md
├── README-English.md
├── .gitignore
└── src/
    ├── stm32-firmware/              # Keil MDK 固件工程
    │   ├── Project/                 # Keil 工程与调试配置（uvprojx / uvoptx / dbgconf）
    │   ├── User/                    # 用户源代码
    │   │   ├── main.c               # 主流程：网络初始化 / 帧处理 / 断线重连
    │   │   ├── sync_calc_params/    # Sync_CalcParams 双轴同步算法
    │   │   ├── microstep_driver/    # 梯形加减速状态机 + TIM 脉冲驱动
    │   │   ├── bsp_esp8266/         # ESP8266 AT 驱动（TCP 透传 / 断线处理）
    │   │   ├── bsp_usart/           # 调试串口 + 指令解析
    │   │   ├── common/              # USART_printf 等通用工具
    │   │   ├── core_delay/          # DWT 微秒级延时
    │   │   └── stm32f10x_it.c       # 中断服务函数
    │   ├── Libraries/               # STM32F10x 标准外设库 + CMSIS
    │   ├── Doc/                     # 硬件接线与工作流程说明
    │   └── keilkill.bat             # Keil 构建产物清理脚本
    └── python-server/               # 上位机（TCP 服务器 + 可视化）
        └── server.py
```

## 上位机（python-server）

`server.py` 是配套的运动控制上位机，基于 TCP Socket 与 Matplotlib：

- 监听端口 5000，等待 ESP8266 客户端接入，自动替换旧连接并线程安全地维护会话
- 控制台输入 8 参数指令下发；校验失败自动拒绝并提示格式
- 按脉冲当量（默认 2000 步 / cm）换算位移，实时绘制双轴位置（X=电机 2，Y=电机 1）与运动轨迹
- 移动记录追加写入 `memory.txt`（重启自动恢复计数；退出时清理）
- 后台接收线程打印 STM32 上报的调试信息

运行：

```bash
cd src/python-server
python -m pip install numpy matplotlib
python server.py
```

## 快速开始

固件：

1. 使用 Keil MDK 打开 `src/stm32-firmware/Project/D_Motor_sync_server.uvprojx`
2. 在 `User/main.c` 中，将 `WIFI_SSID` / `WIFI_PWD` / `SERVER_IP` 的占位符替换为实际网络环境值
3. 编译并烧录；上电后 ESP8266 自动组网并连接上位机（端口 5000）
4. USART1（PA9 / PA10，115200）可查看运行日志

上位机：按上文安装依赖并启动，随后即可下发指令，例如：

```text
2000 200 200 75 1000 200 200 75
```

## 注意事项

- `main.c` 中 WiFi 名称 / 密码 / 服务器 IP 均为占位符（`YOUR_WIFI_SSID`、`YOUR_WIFI_PASSWORD`、`192.168.1.100`），烧录前必须替换为实际值
- 上位机端口与指令格式须和固件一致（默认 5000 / 8 参数）
- 两轴步数相差越悬殊，从轴速度越低；低速时梯形曲线可能退化为纯加减速（无匀速段），但时间同步仍然成立
- 构建产物（`Output/`、`*.hex` 等）不入库，Keil 重新编译即可生成；`keilkill.bat` 可一键清理临时文件
- 上位机 `memory.txt` 为运行期日志，已在 `.gitignore` 中排除

## 致谢

梯形加减速算法参考 Atmel 应用笔记 AVR446《Linear Speed Control of Stepper Motor》，并在其基础上扩展了 Sync_CalcParams 双轴同步算法与无线指令通道。

## 关键词

`STM32F103` `Keil MDK` `Stepper Motor` `Motion Control` `Trapezoidal Profile` `AVR446` `Sync_CalcParams` `Dual-Axis Synchronization` `Interrupt-Driven` `ESP8266` `TCP` `Embedded C` `Python`
