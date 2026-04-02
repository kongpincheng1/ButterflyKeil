#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CRSF摇杆数据实时监测工具
通过串口接收摇杆数据并实时绘制曲线

数据格式预期：
    CSV格式: Left_X,Left_Y,Right_X,Right_Y,S1,S2,A,B,C,D,E,F\r\n
    示例: -45.5,23.0,12.5,-67.8,50.0,75.0,0,1,2,0,1,2\r\n

依赖安装：
    pip install pyserial matplotlib

使用方法：
    python joystick_monitor.py -p COM4 -b 115200
"""

import serial
import serial.tools.list_ports
import argparse
import sys
import threading
import time
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, FancyBboxPatch
import numpy as np


class JoystickMonitor:
    def __init__(self, port, baudrate=115200, max_points=200):
        """
        初始化摇杆监测器

        Args:
            port: 串口号，如 'COM4' 或 '/dev/ttyUSB0'
            baudrate: 波特率，默认115200
            max_points: 历史数据最大保留点数
        """
        self.port = port
        self.baudrate = baudrate
        self.max_points = max_points
        self.ser = None
        self.running = False
        self.lock = threading.Lock()

        # 摇杆数据（浮点值 -100~100）
        self.left_x = 0.0
        self.left_y = 0.0
        self.right_x = 0.0
        self.right_y = 0.0
        self.s1 = 0.0
        self.s2 = 0.0

        # 开关状态（0, 1, 2）
        self.a = 0
        self.b = 0
        self.c = 0
        self.d = 0
        self.e = 0
        self.f = 0

        # 历史数据用于绘制曲线
        self.time_history = deque(maxlen=max_points)
        self.left_x_history = deque(maxlen=max_points)
        self.left_y_history = deque(maxlen=max_points)
        self.right_x_history = deque(maxlen=max_points)
        self.right_y_history = deque(maxlen=max_points)
        self.s1_history = deque(maxlen=max_points)
        self.s2_history = deque(maxlen=max_points)

        self.start_time = time.time()
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0

    def connect(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            print(f"✓ 成功连接到 {self.port} @ {self.baudrate}bps")
            return True
        except serial.SerialException as e:
            print(f"✗ 无法连接到串口 {self.port}: {e}")
            return False

    def disconnect(self):
        """断开串口连接"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"✓ 已断开与 {self.port} 的连接")

    def parse_data(self, line):
        """
        解析CSV格式的摇杆数据

        格式: Left_X,Left_Y,Right_X,Right_Y,S1,S2,A,B,C,D,E,F
        """
        try:
            parts = line.strip().split(',')
            if len(parts) >= 12:
                with self.lock:
                    self.left_x = float(parts[0])
                    self.left_y = float(parts[1])
                    self.right_x = float(parts[2])
                    self.right_y = float(parts[3])
                    self.s1 = float(parts[4])
                    self.s2 = float(parts[5])
                    self.a = int(parts[6])
                    self.b = int(parts[7])
                    self.c = int(parts[8])
                    self.d = int(parts[9])
                    self.e = int(parts[10])
                    self.f = int(parts[11])

                    # 记录历史数据
                    current_time = time.time() - self.start_time
                    self.time_history.append(current_time)
                    self.left_x_history.append(self.left_x)
                    self.left_y_history.append(self.left_y)
                    self.right_x_history.append(self.right_x)
                    self.right_y_history.append(self.right_y)
                    self.s1_history.append(self.s1)
                    self.s2_history.append(self.s2)

                self.frame_count += 1
                return True
        except (ValueError, IndexError) as e:
            pass
        return False

    def read_thread(self):
        """串口读取线程"""
        buffer = ""
        while self.running:
            try:
                if self.ser and self.ser.is_open:
                    data = self.ser.read(self.ser.in_waiting or 1)
                    if data:
                        buffer += data.decode('utf-8', errors='ignore')
                        # 处理完整行
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            self.parse_data(line)
            except serial.SerialException as e:
                print(f"串口错误: {e}")
                break
            except Exception as e:
                pass

            time.sleep(0.001)

    def start(self):
        """启动监测"""
        if not self.connect():
            return False

        self.running = True
        self.start_time = time.time()

        # 启动读取线程
        self.read_thread_obj = threading.Thread(target=self.read_thread)
        self.read_thread_obj.daemon = True
        self.read_thread_obj.start()

        return True

    def get_data(self):
        """获取当前摇杆数据（线程安全）"""
        with self.lock:
            return {
                'left_x': self.left_x,
                'left_y': self.left_y,
                'right_x': self.right_x,
                'right_y': self.right_y,
                's1': self.s1,
                's2': self.s2,
                'a': self.a,
                'b': self.b,
                'c': self.c,
                'd': self.d,
                'e': self.e,
                'f': self.f,
                'time_history': list(self.time_history),
                'left_x_history': list(self.left_x_history),
                'left_y_history': list(self.left_y_history),
                'right_x_history': list(self.right_x_history),
                'right_y_history': list(self.right_y_history),
                's1_history': list(self.s1_history),
                's2_history': list(self.s2_history),
            }


def draw_joystick(ax, x, y, title, color):
    """绘制单个摇杆的可视化"""
    ax.clear()
    ax.set_xlim(-120, 120)
    ax.set_ylim(-120, 120)
    ax.set_aspect('equal')
    ax.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
    ax.axvline(x=0, color='gray', linestyle='-', alpha=0.3)

    # 绘制边界圆
    circle = Circle((0, 0), 100, fill=False, color='gray', linestyle='--', alpha=0.5)
    ax.add_patch(circle)

    # 绘制摇杆位置
    ax.plot(x, y, 'o', markersize=20, color=color, markeredgecolor='black', markeredgewidth=2)

    # 绘制连接线
    ax.plot([0, x], [0, y], '-', color=color, alpha=0.5, linewidth=2)

    # 显示数值
    ax.text(0, 110, f'{title}', ha='center', fontsize=12, fontweight='bold')
    ax.text(0, -115, f'X:{x:6.1f} Y:{y:6.1f}', ha='center', fontsize=10, family='monospace')

    ax.grid(True, alpha=0.3)


def draw_slider(ax, value, title, color):
    """绘制滑块可视化"""
    ax.clear()
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 1)
    ax.set_aspect('equal')

    # 绘制滑块轨道
    rect = FancyBboxPatch((0, 0.3), 100, 0.4, boxstyle="round,pad=0.02",
                          facecolor='lightgray', edgecolor='gray')
    ax.add_patch(rect)

    # 绘制滑块位置
    ax.plot(value, 0.5, 'o', markersize=15, color=color, markeredgecolor='black', markeredgewidth=2)

    # 显示数值
    ax.text(50, 1.2, f'{title}: {value:.1f}', ha='center', fontsize=11, fontweight='bold')

    ax.axis('off')


def draw_switch(ax, value, title):
    """绘制开关状态"""
    colors = {0: 'lightgray', 1: 'green', 2: 'red'}
    color = colors.get(value, 'gray')

    ax.clear()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_aspect('equal')

    # 绘制开关背景
    rect = FancyBboxPatch((0.1, 0.2), 0.8, 0.6, boxstyle="round,pad=0.05",
                          facecolor=color, edgecolor='black', linewidth=2)
    ax.add_patch(rect)

    # 显示数值
    ax.text(0.5, 0.5, f'{value}', ha='center', va='center', fontsize=20, fontweight='bold')
    ax.text(0.5, -0.1, title, ha='center', fontsize=10)

    ax.axis('off')


def create_plot(monitor):
    """创建实时绘图窗口"""
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('CRSF摇杆数据实时监测', fontsize=16, fontweight='bold')

    # 创建子图布局
    # 左摇杆
    ax_left = fig.add_subplot(3, 4, 1)
    # 右摇杆
    ax_right = fig.add_subplot(3, 4, 2)
    # 滑块S1
    ax_s1 = fig.add_subplot(3, 4, 3)
    # 滑块S2
    ax_s2 = fig.add_subplot(3, 4, 4)

    # 开关A-F
    ax_a = fig.add_subplot(3, 4, 5)
    ax_b = fig.add_subplot(3, 4, 6)
    ax_c = fig.add_subplot(3, 4, 7)
    ax_d = fig.add_subplot(3, 4, 8)
    ax_e = fig.add_subplot(3, 4, 9)
    ax_f = fig.add_subplot(3, 4, 10)

    # 历史曲线图
    ax_history = fig.add_subplot(3, 4, (11, 12))

    def update(frame):
        data = monitor.get_data()

        # 计算FPS
        current_time = time.time()
        if current_time - monitor.last_fps_time >= 1.0:
            monitor.fps = monitor.frame_count
            monitor.frame_count = 0
            monitor.last_fps_time = current_time

        # 绘制摇杆
        draw_joystick(ax_left, data['left_x'], data['left_y'], '左摇杆', 'blue')
        draw_joystick(ax_right, data['right_x'], data['right_y'], '右摇杆', 'red')

        # 绘制滑块
        draw_slider(ax_s1, data['s1'], 'S1', 'orange')
        draw_slider(ax_s2, data['s2'], 'S2', 'purple')

        # 绘制开关
        draw_switch(ax_a, data['a'], 'A')
        draw_switch(ax_b, data['b'], 'B')
        draw_switch(ax_c, data['c'], 'C')
        draw_switch(ax_d, data['d'], 'D')
        draw_switch(ax_e, data['e'], 'E')
        draw_switch(ax_f, data['f'], 'F')

        # 绘制历史曲线
        ax_history.clear()
        if len(data['time_history']) > 1:
            ax_history.plot(data['time_history'], data['left_x_history'], 'b-', label='Left X', alpha=0.7)
            ax_history.plot(data['time_history'], data['left_y_history'], 'b--', label='Left Y', alpha=0.7)
            ax_history.plot(data['time_history'], data['right_x_history'], 'r-', label='Right X', alpha=0.7)
            ax_history.plot(data['time_history'], data['right_y_history'], 'r--', label='Right Y', alpha=0.7)
            ax_history.set_xlabel('时间 (秒)')
            ax_history.set_ylabel('数值')
            ax_history.set_title(f'历史数据曲线 (FPS: {monitor.fps})')
            ax_history.legend(loc='upper right')
            ax_history.grid(True, alpha=0.3)
            ax_history.set_ylim(-120, 120)

        plt.tight_layout()

    # 创建动画
    ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)

    # 添加关闭事件处理
    def on_close(event):
        monitor.disconnect()
        print("监测已停止")

    fig.canvas.mpl_connect('close_event', on_close)

    plt.show()


def list_serial_ports():
    """列出可用串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未找到可用串口")
        return

    print("\n可用串口列表:")
    print("-" * 50)
    for port in ports:
        print(f"  {port.device}: {port.description}")
    print("-" * 50)


def main():
    parser = argparse.ArgumentParser(
        description='CRSF摇杆数据实时监测工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s -p COM4              # 使用COM4，默认115200波特率
  %(prog)s -p /dev/ttyUSB0 -b 921600  # Linux系统，指定波特率
  %(prog)s -l                   # 列出可用串口
        """
    )
    parser.add_argument('-p', '--port', help='串口号 (如 COM4 或 /dev/ttyUSB0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                        help='波特率 (默认: 115200)')
    parser.add_argument('-l', '--list', action='store_true',
                        help='列出可用串口')

    args = parser.parse_args()

    if args.list:
        list_serial_ports()
        return

    if not args.port:
        print("错误: 请指定串口号 (-p PORT)")
        list_serial_ports()
        print("\n使用 -h 查看帮助")
        sys.exit(1)

    print("=" * 60)
    print("CRSF摇杆数据实时监测工具")
    print("=" * 60)
    print(f"串口: {args.port}")
    print(f"波特率: {args.baudrate}")
    print(f"数据格式: CSV (Left_X,Left_Y,Right_X,Right_Y,S1,S2,A,B,C,D,E,F)")
    print("=" * 60)
    print()

    # 创建监测器
    monitor = JoystickMonitor(args.port, args.baudrate)

    # 启动监测
    if not monitor.start():
        print("启动失败")
        sys.exit(1)

    print("正在启动图形界面...")
    print("提示: 关闭图形窗口即可停止监测")
    print()

    try:
        # 创建绘图
        create_plot(monitor)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        monitor.disconnect()


if __name__ == '__main__':
    main()
