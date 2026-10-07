"""
双轴步进电机运动控制上位机（TCP 服务器 + 实时轨迹可视化）
============================================================

配合 STM32F103 双轴步进电机同步运动控制系统使用：
    - 监听 TCP 端口，等待 STM32（ESP8266 透传）客户端接入；
    - 从控制台读取 8 参数运动指令并下发至 STM32；
    - 根据运动指令实时更新并绘制双轴位置轨迹（Matplotlib 动画）；
    - 将移动记录追加写入本地日志文件 memory.txt。

指令格式（8 个整数，空格分隔，以 \\r\\n 结尾）：
    步数1 加速度1 减速度1 速度1 步数2 加速度2 减速度2 速度2
    示例：2000 200 200 75 1000 200 200 75

运行环境：
    Python 3.x，依赖 numpy 与 matplotlib。
    用法：python server.py
"""

import os
import socket
import threading
from datetime import datetime

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


# ============================ 配置常量 ============================

PORT = 5000                     # 监听端口，需与固件 SERVER_PORT 一致
MAX_RANGE_CM = 48               # 运动平台最大行程，单位 cm
STEPS_PER_CM = 2000             # 脉冲当量：每厘米对应步数（按实际机械校准）
RECV_BUFFER_SIZE = 1024         # socket 单次接收缓冲区大小，单位字节
PLOT_INTERVAL_MS = 100          # Matplotlib 动画刷新周期，单位毫秒
PLOT_SAVE_COUNT = 100           # 动画保存帧数上限（仅用于避免警告）
MEMORY_FILE = "./memory.txt"    # 移动记录日志文件路径


# ============================ 全局状态 ============================

client_conn = None                          # 当前连接的 STM32 客户端 socket
client_lock = threading.Lock()              # client_conn 访问锁
connected_event = threading.Event()         # 客户端连接就绪事件

motor1_pos = 0.0    # 电机 1（Y 轴）当前位置，单位 cm
motor2_pos = 0.0    # 电机 2（X 轴）当前位置，单位 cm

trajectory_x = []   # 轨迹点 X 坐标序列（电机 2）
trajectory_y = []   # 轨迹点 Y 坐标序列（电机 1）

move_count = 0      # 累计移动次数（从 memory.txt 恢复）


# ============================ 辅助函数 ============================

def get_local_ip() -> str:
    """获取本机局域网 IP 地址（用于提示 STM32 连接目标）。

    Returns:
        本机 IP 字符串；获取失败时返回 "unknown"。
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except OSError:
        return "unknown"


def validate_cmd(cmd: str) -> bool:
    """校验运动指令是否为 8 个空格分隔的整数。

    Args:
        cmd: 待校验的指令字符串。

    Returns:
        校验通过返回 True，否则返回 False。
    """
    parts = cmd.strip().split()
    if len(parts) != 8:
        return False
    try:
        [int(p) for p in parts]
        return True
    except ValueError:
        return False


def log_movement(y_direction, y_distance, x_direction, x_distance):
    """将本次移动信息追加写入 memory.txt。

    Args:
        y_direction: Y 轴（电机 1）移动方向文本。
        y_distance:  Y 轴移动距离，单位 cm。
        x_direction: X 轴（电机 2）移动方向文本。
        x_distance:  X 轴移动距离，单位 cm。
    """
    global move_count
    move_count += 1

    with open(MEMORY_FILE, "a", encoding="utf-8") as f:
        f.write(f"第 {move_count} 次移动\n")
        f.write(f"Y轴(电机1)：向 {y_direction} 移动了 {y_distance:.2f} cm\n")
        f.write(f"X轴(电机2)：向 {x_direction} 移动了 {x_distance:.2f} cm\n")
        f.write("\n")


def update_position(cmd: str):
    """根据运动指令更新双轴位置、轨迹点与移动日志。

    指令第 1 个参数为电机 1（Y 轴）步数，第 5 个参数为电机 2（X 轴）步数；
    约定正步数代表负方向移动，位置被限制在 [0, MAX_RANGE_CM] 范围内。

    Args:
        cmd: 已通过 validate_cmd 校验的指令字符串。
    """
    global motor1_pos, motor2_pos

    parts = cmd.strip().split()
    if len(parts) == 8:
        try:
            # 解析指令 —— 每个步进值代表移动距离
            # 第 1 个步进值对应电机 1（Y 轴），第 5 个步进值对应电机 2（X 轴）
            steps_motor1 = int(parts[0])
            steps_motor2 = int(parts[4])

            # 步数换算为距离（脉冲当量按实际机械校准调整）
            y_distance = steps_motor1 / STEPS_PER_CM
            x_distance = steps_motor2 / STEPS_PER_CM

            # 正步数表示负方向移动
            y_direction = "负" if y_distance > 0 else "正"
            x_direction = "负" if x_distance > 0 else "正"

            log_movement(y_direction, abs(y_distance), x_direction, abs(x_distance))

            # 更新位置（正步数 = 负方向移动）
            motor1_pos -= y_distance
            motor2_pos -= x_distance

            # 位置限幅到有效行程内
            motor1_pos = max(0, min(MAX_RANGE_CM, motor1_pos))
            motor2_pos = max(0, min(MAX_RANGE_CM, motor2_pos))

            # 追加轨迹点
            trajectory_x.append(motor2_pos)
            trajectory_y.append(motor1_pos)

            print(f"[Position Update] Motor1(Y): {motor1_pos:.2f}cm, Motor2(X): {motor2_pos:.2f}cm")

        except ValueError:
            pass


# ============================ 网络线程 ============================

def recv_thread(conn: socket.socket, addr):
    """后台接收线程：持续读取 STM32 上报的调试信息并打印。

    Args:
        conn: 已建立的客户端连接。
        addr: 客户端地址元组 (ip, port)。
    """
    global client_conn
    try:
        while True:
            data = conn.recv(RECV_BUFFER_SIZE)
            if not data:
                break
            print(f"\n[STM32] {data.decode('utf-8', errors='replace').strip()}\n>>> ", end="", flush=True)
    except (ConnectionResetError, OSError):
        pass
    finally:
        with client_lock:
            if client_conn is conn:
                client_conn = None
                connected_event.clear()
        conn.close()
        print(f"\n[disconnected] {addr}")


def accept_thread(server: socket.socket):
    """后台接入线程：接受客户端连接，替换旧连接并启动接收线程。

    Args:
        server: 已绑定并进入监听状态的服务器 socket。
    """
    global client_conn
    while True:
        try:
            conn, addr = server.accept()
        except OSError:
            break
        with client_lock:
            if client_conn:
                client_conn.close()
            client_conn = conn
        connected_event.set()
        print(f"\n[connected] {addr}  -- ready to send commands\n>>> ", end="", flush=True)
        threading.Thread(target=recv_thread, args=(conn, addr), daemon=True).start()


# ============================ 可视化 ============================

# 初始化坐标系：X 轴为电机 2，Y 轴为电机 1，原点位于左下角
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(0, MAX_RANGE_CM)
ax.set_ylim(0, MAX_RANGE_CM)
ax.set_xlabel('Motor 2 Position (X axis) - cm')
ax.set_ylabel('Motor 1 Position (Y axis) - cm')
ax.set_title('Real-time Motor Positions (Origin at Bottom Left)')
ax.grid(True, linestyle='--', alpha=0.6)

current_point, = ax.plot([], [], 'ro', markersize=10, label='Current Position')      # 当前位置点
trajectory_line, = ax.plot([], [], 'b-', linewidth=1, alpha=0.6, label='Trajectory')  # 轨迹线
ax.plot(0, 0, 'go', markersize=8, label='Origin (0,0)')                               # 原点标记

ax.legend()


def animate(frame):
    """Matplotlib 动画更新函数：刷新当前位置点与轨迹线。

    Args:
        frame: 动画帧序号（由 FuncAnimation 传入）。

    Returns:
        需要重绘的图元元组。
    """
    global motor1_pos, motor2_pos

    if len(trajectory_x) > 0 and len(trajectory_y) > 0:
        current_point.set_data([trajectory_x[-1]], [trajectory_y[-1]])

    if len(trajectory_x) > 1 and len(trajectory_y) > 1:
        trajectory_line.set_data(trajectory_x, trajectory_y)

    return current_point, trajectory_line


def update_plot_immediately():
    """发送指令后立即强制重绘图形，保证轨迹实时刷新。"""
    if len(trajectory_x) > 0 and len(trajectory_y) > 0:
        current_point.set_data([trajectory_x[-1]], [trajectory_y[-1]])

    if len(trajectory_x) > 1 and len(trajectory_y) > 1:
        trajectory_line.set_data(trajectory_x, trajectory_y)

    fig.canvas.draw()
    fig.canvas.flush_events()


def cleanup_memory_file():
    """服务器退出时清理 memory.txt 日志文件。"""
    try:
        if os.path.exists(MEMORY_FILE):
            os.remove(MEMORY_FILE)
            print(f"[info] Memory file '{MEMORY_FILE}' has been cleared.")
    except Exception as e:
        print(f"[error] Failed to clear memory file: {e}")


# ============================ 主流程 ============================

def main():
    """程序入口：初始化日志与图形，启动服务器并进入指令输入循环。"""
    global motor1_pos, motor2_pos, move_count

    # 初始化 memory.txt：存在则恢复移动计数，不存在则创建并写入文件头
    if os.path.exists(MEMORY_FILE):
        with open(MEMORY_FILE, "r", encoding="utf-8") as f:
            content = f.read()
            move_count = content.count("第 ")
    else:
        with open(MEMORY_FILE, "w", encoding="utf-8") as f:
            f.write("# 电机移动记录\n")
            f.write(f"# 开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("# 格式: 正方向表示向右/向上，负方向表示向左/向下\n")
            f.write("\n")

    # 启动非阻塞动画
    ani = animation.FuncAnimation(fig, animate, interval=PLOT_INTERVAL_MS,
                                  blit=True, save_count=PLOT_SAVE_COUNT)
    plt.show(block=False)

    # 启动 TCP 服务器
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", PORT))
    server.listen(1)

    ip = get_local_ip()
    print(f"Server started on port {PORT}")
    print(f"Local IP: {ip}:{PORT}")
    print(f"Command format: step1 accel1 decel1 speed1 step2 accel2 decel2 speed2")
    print(f"   Example    : 2000 200 200 75 2000 200 200 75")
    print(f"Type 'quit' to exit\n")
    print("Waiting for STM32 to connect...")

    threading.Thread(target=accept_thread, args=(server,), daemon=True).start()

    try:
        while True:
            connected_event.wait()

            try:
                cmd = input(">>> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nExit.")
                break

            if cmd.lower() == "quit":
                break
            if not cmd:
                continue
            if not validate_cmd(cmd):
                print("[error] Need 8 integers separated by spaces.")
                continue

            with client_lock:
                conn = client_conn

            if conn is None:
                print("[warning] No client connected.")
                continue

            try:
                conn.sendall((cmd + "\r\n").encode("utf-8"))
                print(f"[sent] {cmd}")

                # 更新位置并立即刷新图形显示
                update_position(cmd)
                update_plot_immediately()

            except OSError as e:
                print(f"[error] Send failed: {e}")

    finally:
        # 退出时清理日志文件并关闭服务器
        cleanup_memory_file()
        server.close()


if __name__ == "__main__":
    main()
