#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CRSF Joystick Data Real-time Monitor
Receive joystick data via serial port and plot in real-time

Expected data format:
    CSV format: Left_X,Left_Y,Right_X,Right_Y,S1,S2,A,B,C,D,E,F\r\n
    Example: -45.5,23.0,12.5,-67.8,50.0,75.0,0,1,2,0,1,2\r\n

Dependency installation:
    pip install pyserial matplotlib

Usage:
    python joystick_monitor.py -p COM4 -b 420000
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
    def __init__(self, port='COM7', baudrate=420000, max_points=200):
        """
        Initialize joystick monitor

        Args:
            port: Serial port, e.g., 'COM7' or '/dev/ttyUSB0'
            baudrate: Baud rate, default 420000bps
            max_points: Maximum number of historical data points to retain
        """
        self.port = port
        self.baudrate = baudrate
        self.max_points = max_points
        self.ser = None
        self.running = False
        self.lock = threading.Lock()

        # Joystick data (float values -100~100)
        self.left_x = 0.0
        self.left_y = 0.0
        self.right_x = 0.0
        self.right_y = 0.0
        self.s1 = 0.0
        self.s2 = 0.0

        # Switch states (0, 1, 2)
        self.a = 0
        self.b = 0
        self.c = 0
        self.d = 0
        self.e = 0
        self.f = 0
        
        # Battery voltage
        self.battery_voltage = 0.0
        # Debug: raw ADC value
        self.adc_value = 0
        # Hall sensor data
        self.hall_value = 0.0
        self.hall_adc_value = 0
        self.hall_min = 0.0
        self.hall_max = 0.0
        self.hall_avg = 0.0
        # Second hall sensor data
        self.hall_value2 = 0.0
        self.hall_adc_value2 = 0
        # Motor direction states: 0=UP, 1=DOWN
        self.motor1_state = 0
        self.motor4_state = 0

        # Historical data for plotting curves
        self.time_history = deque(maxlen=max_points)
        self.left_x_history = deque(maxlen=max_points)
        self.left_y_history = deque(maxlen=max_points)
        self.right_x_history = deque(maxlen=max_points)
        self.right_y_history = deque(maxlen=max_points)
        self.s1_history = deque(maxlen=max_points)
        self.s2_history = deque(maxlen=max_points)
        self.battery_history = deque(maxlen=max_points)
        self.hall_history = deque(maxlen=max_points)
        self.hall_history2 = deque(maxlen=max_points)

        self.start_time = time.time()
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0

    def connect(self):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            print(f"Successfully connected to {self.port} @ {self.baudrate}bps")
            return True
        except serial.SerialException as e:
            print(f"Cannot connect to serial port {self.port}: {e}")
            return False

    def disconnect(self):
        """Disconnect from serial port"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"Disconnected from {self.port}")

    def parse_data(self, line):
        """
        Parse CSV format joystick data

        Format: Left_X,Left_Y,Right_X,Right_Y,S1,S2,A,B,C,D,E,F,BatteryVoltage,ADCValue,HallValue,HallADCValue,HallMin,HallMax,HallAvg
        """
        try:
            parts = line.strip().split(',')
            if len(parts) >= 23:
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
                    self.battery_voltage = float(parts[12])
                    self.adc_value = int(parts[13])
                    # Hall sensor data
                    self.hall_value = float(parts[14])
                    self.hall_adc_value = int(parts[15])
                    # Second hall sensor data
                    self.hall_value2 = float(parts[16])
                    self.hall_adc_value2 = int(parts[17])
                    # Hall sensor statistics (using first hall sensor data)
                    self.hall_min = float(parts[18])
                    self.hall_max = float(parts[19])
                    self.hall_avg = float(parts[20])
                    # Motor direction states: 0=FLAP_UP, 1=FLAP_DOWN
                    self.motor1_state = int(parts[21])
                    self.motor4_state = int(parts[22])

                    # Record historical data
                    current_time = time.time() - self.start_time
                    self.time_history.append(current_time)
                    self.left_x_history.append(self.left_x)
                    self.left_y_history.append(self.left_y)
                    self.right_x_history.append(self.right_x)
                    self.right_y_history.append(self.right_y)
                    self.s1_history.append(self.s1)
                    self.s2_history.append(self.s2)
                    self.battery_history.append(self.battery_voltage)
                    self.hall_history.append(self.hall_value)
                    self.hall_history2.append(self.hall_value2)

                self.frame_count += 1
                return True
        except (ValueError, IndexError) as e:
            pass
        return False

    def read_thread(self):
        """Serial port read thread"""
        buffer = ""
        while self.running:
            try:
                if self.ser and self.ser.is_open:
                    data = self.ser.read(self.ser.in_waiting or 1)
                    if data:
                        buffer += data.decode('utf-8', errors='ignore')
                        # Process complete lines
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            self.parse_data(line)
            except serial.SerialException as e:
                print(f"Serial port error: {e}")
                break
            except Exception as e:
                pass

            time.sleep(0.001)

    def start(self):
        """Start monitoring"""
        if not self.connect():
            return False

        self.running = True
        self.start_time = time.time()

        # Start read thread
        self.read_thread_obj = threading.Thread(target=self.read_thread)
        self.read_thread_obj.daemon = True
        self.read_thread_obj.start()

        return True

    def get_data(self):
        """Get current joystick data (thread-safe)"""
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
                'battery_voltage': self.battery_voltage,
                'adc_value': self.adc_value,
                'hall_value': self.hall_value,
                'hall_adc_value': self.hall_adc_value,
                'hall_value2': self.hall_value2,
                'hall_adc_value2': self.hall_adc_value2,
                'hall_min': self.hall_min,
                'hall_max': self.hall_max,
                'hall_avg': self.hall_avg,
                'motor1_state': self.motor1_state,
                'motor4_state': self.motor4_state,
                'time_history': list(self.time_history),
                'left_x_history': list(self.left_x_history),
                'left_y_history': list(self.left_y_history),
                'right_x_history': list(self.right_x_history),
                'right_y_history': list(self.right_y_history),
                's1_history': list(self.s1_history),
                's2_history': list(self.s2_history),
                'battery_history': list(self.battery_history),
                'hall_history': list(self.hall_history),
                'hall_history2': list(self.hall_history2),
            }


def draw_joystick(ax, x, y, title, color):
    """Draw single joystick visualization"""
    ax.clear()
    ax.set_xlim(-120, 120)
    ax.set_ylim(-120, 120)
    ax.set_aspect('equal')
    ax.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
    ax.axvline(x=0, color='gray', linestyle='-', alpha=0.3)

    # Draw boundary circle
    circle = Circle((0, 0), 100, fill=False, color='gray', linestyle='--', alpha=0.5)
    ax.add_patch(circle)

    # Draw joystick position
    ax.plot(x, y, 'o', markersize=20, color=color, markeredgecolor='black', markeredgewidth=2)

    # Draw connection line
    ax.plot([0, x], [0, y], '-', color=color, alpha=0.5, linewidth=2)

    # Display value
    ax.text(0, 110, f'{title}', ha='center', fontsize=12, fontweight='bold')
    ax.text(0, -115, f'X:{x:6.1f} Y:{y:6.1f}', ha='center', fontsize=10, family='monospace')

    ax.grid(True, alpha=0.3)


def draw_slider(ax, value, title, color):
    """Draw slider visualization"""
    ax.clear()
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 1)
    ax.set_aspect('equal')

    # Draw slider track
    rect = FancyBboxPatch((0, 0.3), 100, 0.4, boxstyle="round,pad=0.02",
                          facecolor='lightgray', edgecolor='gray')
    ax.add_patch(rect)

    # Draw slider position
    ax.plot(value, 0.5, 'o', markersize=15, color=color, markeredgecolor='black', markeredgewidth=2)

    # Display value
    ax.text(50, 1.2, f'{title}: {value:.1f}', ha='center', fontsize=11, fontweight='bold')

    ax.axis('off')


def draw_switch(ax, value, title, is_three_position=False):
    """Draw switch status
    
    Args:
        ax: matplotlib axis object
        value: switch value (0, 1, 2)
        title: switch name
        is_three_position: whether it's a three-position switch
    """
    if is_three_position:
        # Three-position switch: 0=Down, 1=Mid, 2=Up
        colors = {0: 'lightgray', 1: 'yellow', 2: 'green'}
        labels = {0: 'Down', 1: 'Mid', 2: 'Up'}
    else:
        # Two-position switch: 0=Down, 1=Up
        colors = {0: 'lightgray', 1: 'green'}
        labels = {0: 'Down', 1: 'Up'}
    
    color = colors.get(value, 'gray')
    label = labels.get(value, str(value))

    ax.clear()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_aspect('equal')

    # Draw switch background
    rect = FancyBboxPatch((0.1, 0.2), 0.8, 0.6, boxstyle="round,pad=0.05",
                          facecolor=color, edgecolor='black', linewidth=2)
    ax.add_patch(rect)

    # Display value and gear label
    ax.text(0.5, 0.5, f'{value}', ha='center', va='center', fontsize=20, fontweight='bold')
    ax.text(0.5, 0.1, label, ha='center', fontsize=10, color='blue')
    ax.text(0.5, -0.1, title, ha='center', fontsize=10)

    ax.axis('off')


def draw_battery(ax, voltage, adc_value):
    """Draw battery voltage display
    
    Args:
        ax: matplotlib axis object
        voltage: battery voltage value
        adc_value: raw ADC value for debugging
    """
    ax.clear()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_aspect('equal')

    # Determine battery status color
    if voltage > 7.0:
        color = 'green'
    elif voltage > 6.5:
        color = 'yellow'
    else:
        color = 'red'

    # Draw battery outline
    rect = FancyBboxPatch((0.1, 0.2), 0.7, 0.6, boxstyle="round,pad=0.05",
                          facecolor='white', edgecolor='black', linewidth=2)
    ax.add_patch(rect)

    # Draw battery positive terminal
    rect = FancyBboxPatch((0.8, 0.35), 0.1, 0.3, boxstyle="round,pad=0.02",
                          facecolor='white', edgecolor='black', linewidth=2)
    ax.add_patch(rect)

    # Draw battery level
    level = min((voltage - 6.0) / 2.4, 1.0)  # Assume 6.0V to 8.4V range
    if level > 0:
        rect = FancyBboxPatch((0.15, 0.25), 0.6 * level, 0.5, boxstyle="round,pad=0.02",
                              facecolor=color, edgecolor='black', linewidth=1)
        ax.add_patch(rect)

    # Display voltage and ADC value
    ax.text(0.5, 0.6, f'{voltage:.2f}V', ha='center', va='center', fontsize=12, fontweight='bold')
    ax.text(0.5, 0.35, f'ADC:{adc_value}', ha='center', va='center', fontsize=9, color='blue')
    ax.text(0.5, -0.1, 'Battery', ha='center', fontsize=10)

    ax.axis('off')


def draw_hall_sensor(ax, hall_value, hall_adc_value, hall_min, hall_max, hall_avg):
    """Draw hall sensor data display
    
    Args:
        ax: matplotlib axis object
        hall_value: current hall sensor value
        hall_adc_value: raw hall ADC value for debugging
        hall_min: minimum hall sensor value
        hall_max: maximum hall sensor value
        hall_avg: average hall sensor value
    """
    ax.clear()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_aspect('equal')

    # Draw sensor outline
    rect = FancyBboxPatch((0.05, 0.1), 0.9, 0.8, boxstyle="round,pad=0.05",
                          facecolor='white', edgecolor='black', linewidth=2)
    ax.add_patch(rect)

    # Display current value
    ax.text(0.5, 0.75, f'Current: {hall_value:.3f}V', ha='center', va='center', fontsize=11, fontweight='bold')
    
    # Display min, max, avg values
    ax.text(0.33, 0.5, f'Min: {hall_min:.3f}V', ha='center', va='center', fontsize=10)
    ax.text(0.5, 0.5, f'Max: {hall_max:.3f}V', ha='center', va='center', fontsize=10)
    ax.text(0.67, 0.5, f'Avg: {hall_avg:.3f}V', ha='center', va='center', fontsize=10)
    
    # Display raw ADC value
    ax.text(0.5, 0.25, f'ADC: {hall_adc_value}', ha='center', va='center', fontsize=9, color='blue')
    
    # Display sensor name
    ax.text(0.5, -0.1, 'Hall Sensor 1', ha='center', fontsize=10)

    ax.axis('off')


def draw_hall_sensor2(ax, hall_value, hall_adc_value):
    """Draw second hall sensor data display

    Args:
        ax: matplotlib axis object
        hall_value: current hall sensor value
        hall_adc_value: raw hall ADC value for debugging
    """
    ax.clear()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_aspect('equal')

    # Draw sensor outline
    rect = FancyBboxPatch((0.05, 0.1), 0.9, 0.8, boxstyle="round,pad=0.05",
                          facecolor='white', edgecolor='black', linewidth=2)
    ax.add_patch(rect)

    # Display current value
    ax.text(0.5, 0.75, f'Current: {hall_value:.3f}V', ha='center', va='center', fontsize=11, fontweight='bold')

    # Display raw ADC value
    ax.text(0.5, 0.5, f'ADC: {hall_adc_value}', ha='center', va='center', fontsize=9, color='blue')

    # Display sensor name
    ax.text(0.5, -0.1, 'Hall Sensor 2', ha='center', fontsize=10)

    ax.axis('off')


def draw_motor_state(ax, state, title, color):
    """Draw motor direction state display

    Args:
        ax: matplotlib axis object
        state: motor state (0=UP, 1=DOWN)
        title: motor name
        color: display color
    """
    ax.clear()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_aspect('equal')

    state_colors = {0: 'green', 1: 'red'}
    state_labels = {0: 'UP', 1: 'DOWN'}

    state_color = state_colors.get(state, 'gray')
    state_label = state_labels.get(state, 'UNKNOWN')

    # Draw state outline
    rect = FancyBboxPatch((0.05, 0.15), 0.9, 0.7, boxstyle="round,pad=0.05",
                          facecolor=state_color, edgecolor='black', linewidth=3)
    ax.add_patch(rect)

    # Display state
    ax.text(0.5, 0.5, state_label, ha='center', va='center', fontsize=16, fontweight='bold', color='white')

    # Display title
    ax.text(0.5, -0.1, title, ha='center', fontsize=10)

    ax.axis('off')


def create_plot(monitor):
    """Create real-time plotting window"""
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('CRSF Joystick Data Real-time Monitor', fontsize=16, fontweight='bold')

    # Create subplot layout
    # Left joystick
    ax_left = fig.add_subplot(4, 4, 1)
    # Right joystick
    ax_right = fig.add_subplot(4, 4, 2)
    # Slider S1
    ax_s1 = fig.add_subplot(4, 4, 3)
    # Battery voltage
    ax_battery = fig.add_subplot(4, 4, 4)

    # Switches: SA and SD on top row, SB and SC on bottom row
    ax_b = fig.add_subplot(4, 4, 5)  # B(SA)
    ax_f = fig.add_subplot(4, 4, 6)  # F(SD)
    ax_e = fig.add_subplot(4, 4, 9)  # E(SB)
    ax_c = fig.add_subplot(4, 4, 10) # C(SC)
    # Hall sensor 1
    ax_hall = fig.add_subplot(4, 4, 7)  # Hall sensor 1 display
    # Hall sensor 2
    ax_hall2 = fig.add_subplot(4, 4, 8)  # Hall sensor 2 display
    # Motor 1 state
    ax_motor1_state = fig.add_subplot(4, 4, 11)  # Motor 1 state display
    # Motor 4 state
    ax_motor4_state = fig.add_subplot(4, 4, 12)  # Motor 4 state display

    # Historical curve
    ax_history = fig.add_subplot(4, 4, (13, 16))

    def update(frame):
        data = monitor.get_data()

        # Calculate FPS
        current_time = time.time()
        if current_time - monitor.last_fps_time >= 1.0:
            monitor.fps = monitor.frame_count
            monitor.frame_count = 0
            monitor.last_fps_time = current_time

        # Draw joysticks
        draw_joystick(ax_left, data['left_x'], data['left_y'], 'Left Joystick', 'blue')
        draw_joystick(ax_right, data['right_x'], data['right_y'], 'Right Joystick', 'red')

        # Draw sliders, battery, and hall sensors
        draw_slider(ax_s1, data['s1'], 'S1', 'orange')
        draw_battery(ax_battery, data['battery_voltage'], data['adc_value'])
        draw_hall_sensor(ax_hall, data['hall_value'], data['hall_adc_value'], data['hall_min'], data['hall_max'], data['hall_avg'])
        draw_hall_sensor2(ax_hall2, data['hall_value2'], data['hall_adc_value2'])
        draw_motor_state(ax_motor1_state, data['motor1_state'], 'Motor 1', 'blue')
        draw_motor_state(ax_motor4_state, data['motor4_state'], 'Motor 4', 'red')

        # Draw switches
        # Mapping: SA->B, SC->C, SB->E, SD->F
        # B(SA) is a two-position switch: 0=Down, 1=Up
        draw_switch(ax_b, data['b'], 'B(SA)', is_three_position=False)
        # F(SD) is a two-position switch: 0=Down, 1=Up
        draw_switch(ax_f, data['f'], 'F(SD)', is_three_position=False)
        # E(SB) is a three-position switch: 0=Down, 1=Mid, 2=Up
        draw_switch(ax_e, data['e'], 'E(SB)', is_three_position=True)
        # C(SC) is a three-position switch: 0=Down, 1=Mid, 2=Up
        draw_switch(ax_c, data['c'], 'C(SC)', is_three_position=True)

        # Draw historical curve
        ax_history.clear()
        if len(data['time_history']) > 1:
            ax_history.plot(data['time_history'], data['left_x_history'], 'b-', label='Left X', alpha=0.7)
            ax_history.plot(data['time_history'], data['left_y_history'], 'b--', label='Left Y', alpha=0.7)
            ax_history.plot(data['time_history'], data['right_x_history'], 'r-', label='Right X', alpha=0.7)
            ax_history.plot(data['time_history'], data['right_y_history'], 'r--', label='Right Y', alpha=0.7)
            ax_history.plot(data['time_history'], data['battery_history'], 'g-', label='Battery (V)', alpha=0.7)
            ax_history.plot(data['time_history'], data['hall_history'], 'purple', label='Hall 1 (V)', alpha=0.7)
            ax_history.plot(data['time_history'], data['hall_history2'], 'green', label='Hall 2 (V)', alpha=0.7)
            ax_history.set_xlabel('Time (s)')
            ax_history.set_ylabel('Value')
            ax_history.set_title(f'Historical Data Curve (FPS: {monitor.fps})')
            ax_history.legend(loc='upper right')
            ax_history.grid(True, alpha=0.3)
            ax_history.set_ylim(-120, 120)

        plt.tight_layout()

    # Create animation
    ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)

    # Add close event handler
    def on_close(event):
        monitor.disconnect()
        print("Monitoring stopped")

    fig.canvas.mpl_connect('close_event', on_close)

    plt.show()


def list_serial_ports():
    """List available serial ports"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No available serial ports found")
        return

    print("\nAvailable serial ports:")
    print("-" * 50)
    for port in ports:
        print(f"  {port.device}: {port.description}")
    print("-" * 50)


def main():
    parser = argparse.ArgumentParser(
        description='CRSF Joystick Data Real-time Monitor',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s -p COM4              # Use COM4, default 420000 baud rate
  %(prog)s -p /dev/ttyUSB0 -b 921600  # Linux system, specify baud rate
  %(prog)s -l                   # List available serial ports
        """
    )
    parser.add_argument('-p', '--port', help='Serial port (e.g., COM4 or /dev/ttyUSB0)')
    parser.add_argument('-b', '--baudrate', type=int, default=420000,
                        help='Baud rate (default: 420000)')
    parser.add_argument('-l', '--list', action='store_true',
                        help='List available serial ports')

    args = parser.parse_args()

    if args.list:
        list_serial_ports()
        return

    if not args.port:
        print("Error: Please specify serial port (-p PORT)")
        list_serial_ports()
        print("\nUse -h for help")
        sys.exit(1)

    print("=" * 60)
    print("CRSF Joystick Data Real-time Monitor")
    print("=" * 60)
    print(f"Serial Port: {args.port}")
    print(f"Baud Rate: {args.baudrate}")
    print(f"Data Format: CSV (Left_X,Left_Y,Right_X,Right_Y,S1,S2,A,B,C,D,E,F)")
    print("=" * 60)
    print()

    # Create monitor
    monitor = JoystickMonitor(args.port, args.baudrate)

    # Start monitoring
    if not monitor.start():
        print("Startup failed")
        sys.exit(1)

    print("Starting graphical interface...")
    print("Tip: Close the graphical window to stop monitoring")
    print()

    try:
        # Create plot
        create_plot(monitor)
    except KeyboardInterrupt:
        print("\nUser interrupted")
    finally:
        monitor.disconnect()


if __name__ == '__main__':
    main()
