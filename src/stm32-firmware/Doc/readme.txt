项目概览

  一个基于 STM32F103VET6 的双轴步进电机同步运动控制系统。
  固件通过 ESP8266（USART3 透传）接收上位机 TCP 指令，内置
  Sync_CalcParams 双轴同步算法与 AVR446 梯形加减速状态机，实现两轴同时
  启停；运行状态通过 USART1（115200）打印。

硬件资源分配

步进电机1（下）：
  PUL+   ---   PA6（TIM3_CH1）
  PUL-   ---   GND
  DIR+   ---   PA4
  DIR-   ---   GND
  ENA+   ---   PA5
  ENA-   ---   GND

步进电机2（上）：
  PUL+   ---   PB6（TIM4_CH1）
  PUL-   ---   GND
  DIR+   ---   PB7
  DIR-   ---   GND
  ENA+   ---   PB5
  ENA-   ---   GND

ESP8266 无线模块（USART3，115200）：
  PB10（USART3_TX）  ---   ESP8266 RX
  PB11（USART3_RX）  ---   ESP8266 TX
  PB9                ---   ESP8266 RST
  PB8                ---   ESP8266 CH_PD（使能）

调试串口（USART1，115200）：
  PA9 （USART1_TX）  ---   串口调试助手 RX
  PA10（USART1_RX）  ---   串口调试助手 TX

工作流程

  1. main.c 初始化系统延时、USART1（调试打印）与 ESP8266（连接服务器）
  2. USART3 中断接收字节，IDLE 空闲中断判定一帧结束并置帧标志
  3. 主循环检测帧标志，Parse_Cmd_From_Buf() 用 sscanf 解析 8 个参数
  4. Sync_CalcParams 双轴同步算法：比较两轴步数，选出主轴并对从轴做比例缩放
  5. 同时调用 MSD_Move1 / MSD_Move2 启动两轴
  6. TIM3 / TIM4 各自中断输出脉冲，执行"加速 -> 匀速 -> 减速"梯形曲线
  7. 检测到 "CLOSED" 上报判定 TCP 断开，自动重连服务器

指令格式

  步数1 加速度1 减速度1 速度1 步数2 加速度2 减速度2 速度2

  示例：2000 200 200 75 1000 200 200 75

  说明：程序自动判断主轴（步数较多者），主轴参数不变，从轴的
  speed / accel / decel 按步数比例缩放，因此两轴同时启停；
  用户只需给出各自步数与主轴的速度参数即可完成同步运动。
