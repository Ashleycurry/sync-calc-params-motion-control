# Dual-Axis Stepper Motor Synchronized Motion Control System

[中文](README.md) | [English](README-English.md)

A complete dual-axis stepper motor synchronized motion control project based on the STM32F103VET6. The firmware receives motion commands from a host PC through an ESP8266 Wi-Fi transparent bridge, and implements the **Sync_CalcParams dual-axis synchronization algorithm** together with an **AVR446 integer trapezoidal acceleration state machine**, so that both axes start and stop simultaneously and reach their targets precisely. A companion Python host application provides command dispatching, movement logging and real-time trajectory visualization.

## Highlights

### Embedded Software Development

- Bare-metal foreground/background architecture: interrupt handlers perform real-time pulse generation and UART frame reception, while the main loop handles command parsing and motion scheduling - no blocking delays anywhere
- Two independent timer interrupt channels (TIM3 / TIM4) drive the pulse outputs of both axes with 1 us time-base resolution, providing software step counting and position query interfaces
- USART3 with the IDLE line interrupt receives variable-length command frames; USART1 retargets `fputc` / `printf` for debug output
- ESP8266 AT-command driver: DHCP networking, TCP transparent transmission, link status polling, and automatic reconnection upon detecting the `CLOSED` report
- Layered modular design: BSP layer (esp8266 / usart) -> utility layer (common / core_delay) -> application layer (main / sync_calc_params / microstep_driver)
- DWT cycle counter provides microsecond-accurate delays (core_delay); the Keil MDK project works out of the box
- Consistent Doxygen-style comments, UTF-8 encoding, dead-code removal and sensitive-information sanitization

### Algorithm Optimization

- **Sync_CalcParams dual-axis synchronization algorithm**: the slave axis speed / acceleration / deceleration are uniformly scaled by the step-count ratio, so the three motion phases of both axes align exactly in time (pure integer arithmetic, no floating point)
- **AVR446 integer recursive trapezoidal profile**: first-step delay, speed-limit steps and the acceleration/deceleration boundary are all computed with integer math, automatically planning the "accel -> cruise -> decel" profile
- The recursion carries a remainder correction (`rest`), eliminating accumulated error over long movements; no floating point and no lookup tables - ideal for low-end MCUs
- 32-bit intermediate values prevent overflow, and scaled results are clamped to a lower bound of 1, covering extreme parameter combinations

## System Architecture

```text
┌───────────────────────────┐   TCP :5000    ┌───────────────────────────┐
│  Python Host Application  │ <============> │  ESP8266 (TCP passthrough)│
│  Commands / Visualization │                └─────────────┬─────────────┘
└───────────────────────────┘                              │ USART3 115200
                                                           v
┌──────────────────────────────────────────────────────────────────────┐
│                          STM32F103VET6                              │
│                                                                      │
│  USART3 IRQ (IDLE frame detection) -> command parsing (sscanf)      │
│      -> Sync_CalcParams dual-axis sync algorithm                    │
│      -> MSD_Move1 / MSD_Move2 launched simultaneously               │
│                                                                      │
│  TIM3 IRQ -> Motor 1 pulses (accel / cruise / decel state machine)  │
│  TIM4 IRQ -> Motor 2 pulses (accel / cruise / decel state machine)  │
│                                                                      │
│  USART1 115200 -> debug output                                       │
└──────────────────────────────────────────────────────────────────────┘
```

## Core Algorithms

### 1. Sync_CalcParams Dual-Axis Synchronization Algorithm (Parameter Scaling)

**Problem**: when both axes run their trapezoidal profiles independently, different step counts make them stop at different times - they cannot arrive simultaneously.

**Principle**: the key timing parameters of a trapezoidal speed profile are

```text
t_accel = speed / accel      (acceleration phase duration)
t_decel = speed / decel      (deceleration phase duration)
```

As long as both axes share the same `speed/accel` and `speed/decel` ratios, their phase durations are identical. Therefore the axis with more steps is chosen as the **master** (parameters unchanged), and the **slave** axis scales its speed and acceleration/deceleration by the step ratio:

```text
ratio       = steps_slave / steps_master
speed_slave = speed_master × ratio
accel_slave = accel_master × ratio
decel_slave = decel_master × ratio
```

The `ratio` cancels out in the timing formula: `t = (speed×ratio) / (accel×ratio) = speed / accel`, so the slave axis aligns perfectly with the master on the time axis, regardless of the step difference or motion direction.

**Example**: `2000 200 200 75 | 1000 200 200 75`

| Parameter | Motor 1 (master) | Motor 2 (slave, raw) | Motor 2 (scaled) |
| --- | ---: | ---: | ---: |
| Steps | 2000 | 1000 | 1000 (unchanged) |
| Speed | 75 | 75 | 75 × 0.5 = **37** |
| Acceleration | 200 | 200 | 200 × 0.5 = **100** |
| Deceleration | 200 | 200 | 200 × 0.5 = **100** |

Motor 2 travels half the distance at half the speed and half the acceleration, **starting and finishing together** with Motor 1.

**Engineering safeguards** (`sync_calc_params.c`):

- Scaling multiplications use `unsigned long` (32-bit) intermediates to prevent 16-bit overflow
- Scaled results are clamped to a minimum of 1, avoiding division by zero or invalid motion parameters
- Axes with zero steps are skipped; the comparison uses absolute values, so direction does not affect synchronization

### 2. Trapezoidal Profile (AVR446 Integer State Machine)

Speed planning is based on the integer algorithms from the Atmel AVR446 application note - only integer multiply/divide and a square-root pre-computation, with no floating point or lookup tables:

```text
min_delay  = A_T_x100 / speed                            # min pulse delay at top speed
step_delay = T1_FREQ_148 × sqrt(A_SQ / accel) / 100      # first-step delay for accel
max_s_lim  = speed² / (A_x20000 × accel / 100)           # steps needed to reach top speed
accel_lim  = step × decel / (accel + decel)              # steps before deceleration starts
```

Inside the timer interrupt, the theoretical speed curve is approached recursively:

```text
new_delay = delay - (2 × delay + rest) / (4 × accel_count + 1)
rest      = (2 × delay + rest) % (4 × accel_count + 1)
```

The remainder `rest` is carried into the next iteration, so the integer arithmetic introduces no accumulated error. The interrupt service routine maintains a five-state machine:

```text
STOP ──> ACCEL ──> RUN ──> DECEL ──> STOP
  start   accel     cruise    decel     target reached
```

Each pulse falling edge triggers one state evaluation and delay update (software step counting), keeping the two axes completely independent.

## Communication Protocol

The host application and firmware communicate through a **text-based command protocol**: 8 integers, space-separated, terminated by `\r\n`.

| Field | Meaning |
| --- | --- |
| `step1` / `step2` | Step count of motor 1 / 2 (sign selects direction) |
| `accel1` / `accel2` | Acceleration of motor 1 / 2 |
| `decel1` / `decel2` | Deceleration of motor 1 / 2 |
| `speed1` / `speed2` | Top speed of motor 1 / 2 |

Example:

```text
2000 200 200 75 1000 200 200 75
```

Users only provide each axis's step count and the master speed parameters; the slave parameters are scaled automatically by the firmware. Frame boundaries are detected by the IDLE line interrupt - no header/trailer overhead.

## Hardware Resources

| Function | Pin | Note |
| --- | --- | --- |
| Motor 1 pulse PUL | PA6 | TIM3_CH1 |
| Motor 1 direction DIR | PA4 | GPIO push-pull output |
| Motor 1 enable ENA | PA5 | GPIO push-pull output |
| Motor 2 pulse PUL | PB6 | TIM4_CH1 |
| Motor 2 direction DIR | PB7 | GPIO push-pull output |
| Motor 2 enable ENA | PB5 | GPIO push-pull output |
| ESP8266 UART | PB10 / PB11 | USART3 passthrough, 115200 |
| ESP8266 reset / enable | PB9 / PB8 | RST / CH_PD |
| Debug UART | PA9 / PA10 | USART1, 115200 |

| Component | Description |
| --- | --- |
| MCU | STM32F103VET6 (Cortex-M3, 72 MHz, 512 KB Flash) |
| Actuators | Micro-stepping stepper drivers x2 (PUL / DIR / ENA interface) |
| Wireless module | ESP8266 (AT firmware, STA-mode TCP client) |

## Repository Layout

```text
sync-calc-params-motion-control/
├── README.md
├── README-English.md
├── .gitignore
└── src/
    ├── stm32-firmware/              # Keil MDK firmware project
    │   ├── Project/                 # Keil project & debug config (uvprojx / uvoptx / dbgconf)
    │   ├── User/                    # Application source code
    │   │   ├── main.c               # Main flow: network init / frame handling / reconnect
    │   │   ├── sync_calc_params/    # Sync_CalcParams dual-axis sync algorithm
    │   │   ├── microstep_driver/    # Trapezoidal state machine + TIM pulse driver
    │   │   ├── bsp_esp8266/         # ESP8266 AT driver (TCP passthrough / disconnect handling)
    │   │   ├── bsp_usart/           # Debug UART + command parser
    │   │   ├── common/              # USART_printf and other utilities
    │   │   ├── core_delay/          # DWT microsecond delay
    │   │   └── stm32f10x_it.c       # Interrupt service routines
    │   ├── Libraries/               # STM32F10x standard peripheral library + CMSIS
    │   ├── Doc/                     # Wiring and workflow documentation
    │   └── keilkill.bat             # Keil build-artifact cleanup script
    └── python-server/               # Host application (TCP server + visualization)
        └── server.py
```

## Host Application (python-server)

`server.py` is the companion host application built on TCP sockets and Matplotlib:

- Listens on port 5000, waits for the ESP8266 client, and thread-safely replaces stale connections
- Reads 8-parameter commands from the console and dispatches them; invalid commands are rejected with a format hint
- Converts steps to displacement (default 2000 steps/cm) and plots both axes' positions (X = motor 2, Y = motor 1) and trajectory in real time
- Appends movement records to `memory.txt` (count restored on restart, file removed on exit)
- A background receive thread prints debug messages reported by the STM32

Run it with:

```bash
cd src/python-server
python -m pip install numpy matplotlib
python server.py
```

## Quick Start

Firmware:

1. Open `src/stm32-firmware/Project/D_Motor_sync_server.uvprojx` with Keil MDK
2. In `User/main.c`, replace the placeholders of `WIFI_SSID` / `WIFI_PWD` / `SERVER_IP` with your actual network settings
3. Build and flash; after power-up the ESP8266 joins the network and connects to the host application (port 5000)
4. Open USART1 (PA9 / PA10, 115200) to view runtime logs

Host application: install the dependencies and start it as described above, then send commands such as:

```text
2000 200 200 75 1000 200 200 75
```

## Notes

- The Wi-Fi SSID / password / server IP in `main.c` are placeholders (`YOUR_WIFI_SSID`, `YOUR_WIFI_PASSWORD`, `192.168.1.100`) and must be replaced before flashing
- The host port and command format must match the firmware (default: port 5000 / 8 parameters)
- The larger the step-count difference, the lower the slave-axis speed; at low speed the trapezoid may degenerate into pure acceleration/deceleration (no cruise phase), but time synchronization still holds
- Build artifacts (`Output/`, `*.hex`, etc.) are not tracked in this repository; rebuild them with Keil, or run `keilkill.bat` to clean temporary files
- `memory.txt` produced by the host application is a runtime log and is excluded via `.gitignore`

## Acknowledgements

The trapezoidal speed-control implementation is based on Atmel application note AVR446 "Linear Speed Control of Stepper Motor", extended with the Sync_CalcParams dual-axis synchronization algorithm and a wireless command channel.

## Keywords

`STM32F103` `Keil MDK` `Stepper Motor` `Motion Control` `Trapezoidal Profile` `AVR446` `Sync_CalcParams` `Dual-Axis Synchronization` `Interrupt-Driven` `ESP8266` `TCP` `Embedded C` `Python`
