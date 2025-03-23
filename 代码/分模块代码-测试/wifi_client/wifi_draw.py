import matplotlib.pyplot as plt
import math
import time
from matplotlib.font_manager import FontProperties
import matplotlib
import threading
import re
from queue import Queue, Empty
import sys
from matplotlib.widgets import Button
import traceback
from matplotlib.patches import FancyArrowPatch
import socket
import os
from datetime import datetime
import argparse

# 创建logs目录（如果不存在）
if not os.path.exists('logs'):
    os.makedirs('logs')

# 创建新的日志文件（使用当前时间作为文件名）
log_filename = os.path.join('logs', f'wifi_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')
print(f"日志文件将保存在: {log_filename}")

# 设置最大探测范围（单位：厘米）
MAX_DETECTION_RANGE = 70

# WiFi服务器配置
HOST = '192.168.4.1'  # ESP32的IP地址
PORT = 8080           # 端口号

# 尝试加载支持中文的字体
try:
    font_path = r"C:\Windows\Fonts\simhei.ttf"  # 黑体路径
    font = FontProperties(fname=font_path, size=10)
    matplotlib.rcParams['font.family'] = ['SimHei']
    matplotlib.rcParams['font.sans-serif'] = ['SimHei']
    matplotlib.rcParams['axes.unicode_minus'] = False
except FileNotFoundError:
    font = None
    print("无法加载中文字体，将使用默认字体")
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS']
    plt.rcParams['axes.unicode_minus'] = False

# 初始化socket变量
client = None

# 初始化图形
plt.ion()
fig, ax = plt.subplots(figsize=(10, 10))
ax.grid(True)

# 设置初始显示范围和纵横比
initial_range = 150
ax.set_xlim(-initial_range, initial_range)
ax.set_ylim(initial_range, -initial_range)  # 反转y轴方向
# 使用'equal'而不是'equal, datalim'，并设置adjustable参数
ax.set_aspect('equal', adjustable='box')

# 所有扫描点
all_scan_points = []
accumulated_scan_points = []
current_scan_points = []
car_trajectory = [(0, 0, 0)]  # 初始化轨迹，起始点为原点

# 创建绘图对象
scan_points_plot, = ax.plot([], [], 'b.', markersize=2, label='障碍物')
trajectory_plot, = ax.plot([], [], 'g-', linewidth=2, label='运动轨迹')
car_position_plot, = ax.plot([], [], 'ro', markersize=5)
scan_line, = ax.plot([], [], 'b-', linewidth=1)

# 创建红色箭头的代理对象
arrow_proxy = FancyArrowPatch((0, 0), (1, 0), mutation_scale=15, color='r', label='当前朝向')

# 添加图例
ax.legend(handles=[scan_points_plot, trajectory_plot, arrow_proxy], loc='upper right')

# 用于线程间通信的锁
data_lock = threading.Lock()

# 创建一个队列用于线程间通信
update_queue = Queue()

# 在全局变量部分添加：
scan_lines = []
distance_circles = []
distance_texts = []

# 减少绘图更新频率
UPDATE_INTERVAL = 0.1

# 添加退出事件
exit_event = threading.Event()
start_replay_event = threading.Event()  # 添加开始回放事件

# 添加变量用于缩放和平移
zoom_level = 1.0
pan_start = None
current_xlim = ax.get_xlim()
current_ylim = ax.get_ylim()

# 添加状态切换按钮相关变量
manual_mode = False

# 初始化全局小车位置变量
car_x = 0.0
car_y = 0.0
car_yaw = 0.0

# 滤波器参数
FILTER_WINDOW_SIZE = 8  # 滑动窗口大小
filtered_points = []  # 存储滤波后的点
raw_points_buffer = []  # 原始点缓存

def read_log_file(log_path):
    """
    从本地日志文件读取数据并进行处理
    
    Args:
        log_path: 日志文件路径
    """
    global current_scan_points, all_scan_points, accumulated_scan_points, car_trajectory
    
    print(f"正在准备回放日志文件: {log_path}")
    print("请点击'开始回放'按钮开始...")
    
    try:
        with open(log_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        
        print(f"成功读取日志文件，共 {len(lines)} 行")
        
        # 等待开始回放按钮被点击
        start_replay_event.wait()
        
        # 解析日志文件,收集所有带时间戳的事件
        events = []
        timestamp_pattern = re.compile(r'\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+)\]')
        pos_pattern = re.compile(r'\[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+\] 位置数据: x=([-+]?\d*\.?\d+), y=([-+]?\d*\.?\d+), yaw=([-+]?\d*\.?\d+)')
        scan_pattern = re.compile(r'\[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+\] 扫描数据: angle=([-+]?\d*\.?\d+), distance=([-+]?\d*\.?\d+)')
        segment_end_pattern = re.compile(r'\[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+\] 数据段结束')
        
        current_segment = []
        for line in lines:
            # 解析时间戳
            timestamp_match = timestamp_pattern.search(line)
            if not timestamp_match:
                continue
                
            timestamp_str = timestamp_match.group(1)
            timestamp = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S.%f')
            
            # 处理位置数据
            pos_match = pos_pattern.search(line)
            if pos_match:
                x, y, yaw = map(float, pos_match.groups())
                events.append({
                    'timestamp': timestamp,
                    'type': 'position',
                    'data': (x, y, yaw)
                })
                continue
            
            # 处理扫描数据
            scan_match = scan_pattern.search(line)
            if scan_match:
                angle, distance = map(float, scan_match.groups())
                events.append({
                    'timestamp': timestamp,
                    'type': 'scan',
                    'data': (angle, distance)
                })
                continue
                
            # 处理数据段结束标记
            segment_end_match = segment_end_pattern.search(line)
            if segment_end_match:
                events.append({
                    'timestamp': timestamp,
                    'type': 'segment_end',
                    'data': None
                })
        
        # 按时间戳排序事件
        events.sort(key=lambda x: x['timestamp'])
        
        if not events:
            print("未在日志中找到有效数据")
            return
            
        # 获取第一个和最后一个事件的时间戳
        start_time = events[0]['timestamp']
        end_time = events[-1]['timestamp']
        total_duration = (end_time - start_time).total_seconds()
        
        print(f"数据时间范围: {start_time} 到 {end_time}")
        print(f"总时长: {total_duration:.2f} 秒")
        
        # 初始化显示
        current_position = None
        
        # 处理每个事件
        for event in events:
            # 计算事件的相对时间
            relative_time = (event.get('timestamp') - start_time).total_seconds()
            
            # 根据事件类型更新数据
            if event['type'] == 'position':
                x, y, yaw = event['data']
                with data_lock:
                    car_trajectory.append((x, y, yaw))
                    current_position = (x, y, yaw)
                    update_queue.put(True)
                
            elif event['type'] == 'scan' and current_position:
                angle, distance = event['data']
                x, y, yaw = current_position
                
                # 计算扫描点坐标
                adjusted_angle = angle - 90
                scan_x = distance * math.cos(math.radians(adjusted_angle))
                scan_y = -distance * math.sin(math.radians(adjusted_angle))
                
                rotated_x = scan_x * math.cos(math.radians(yaw)) - scan_y * math.sin(math.radians(yaw))
                rotated_y = scan_x * math.sin(math.radians(yaw)) + scan_y * math.cos(math.radians(yaw))
                final_x = rotated_x + x
                final_y = rotated_y + y
                
                with data_lock:
                    if distance < MAX_DETECTION_RANGE:
                        all_scan_points.append((final_x, final_y))
                        accumulated_scan_points.append((final_x, final_y))
                    else:
                        accumulated_scan_points.append((float('nan'), float('nan')))
                    update_queue.put(True)
                    
            elif event['type'] == 'segment_end':
                with data_lock:
                    accumulated_scan_points.append((float('nan'), float('nan')))
                    update_queue.put(True)
            
            # 根据实际时间间隔添加延时,将总时长缩短为原始的1/2
            if events.index(event) < len(events) - 1:
                next_event = events[events.index(event) + 1]
                delay = (next_event['timestamp'] - event['timestamp']).total_seconds() / 2
                time.sleep(delay)
        
        print("日志文件复现完成")
        
        # 持续更新图形，直到用户关闭窗口
        while not exit_event.is_set():
            time.sleep(0.1)
            
    except Exception as e:
        error_msg = f"日志文件处理错误: {e}\n{traceback.format_exc()}"
        print(error_msg)

def apply_moving_average_filter(points, window_size):
    """
    对点集应用滑动窗口平均滤波
    
    Args:
        points: 原始点集列表，每个元素为(x, y)元组
        window_size: 滑动窗口大小
        
    Returns:
        filtered_points: 滤波后的点集列表
    """
    if len(points) < window_size:
        return points
        
    filtered = []
    valid_points = [(x, y) for x, y in points if not (math.isnan(x) or math.isnan(y))]
    
    for i in range(len(valid_points)):
        # 获取当前窗口内的点
        start_idx = max(0, i - window_size // 2)
        end_idx = min(len(valid_points), i + window_size // 2 + 1)
        window = valid_points[start_idx:end_idx]
        
        if not window:
            continue
            
        # 计算窗口内点的平均位置
        x_sum = sum(p[0] for p in window)
        y_sum = sum(p[1] for p in window)
        x_avg = x_sum / len(window)
        y_avg = y_sum / len(window)
        
        filtered.append((x_avg, y_avg))
    
    return filtered

def read_wifi_data():
    global current_scan_points, all_scan_points, accumulated_scan_points, car_trajectory
    global raw_points_buffer, filtered_points
    buffer = ""
    current_segment = []
    is_receiving_segment = False
    
    while not exit_event.is_set():
        try:
            # 从WiFi接收数据
            data = client.recv(1024).decode('utf-8')
            if not data:
                print("WiFi连接已断开")
                break
            
            # 记录原始数据
            with open(log_filename, 'a', encoding='utf-8') as f:
                f.write(f"\n[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}] 接收数据:\n{data}\n")
                
            buffer += data
            
            # 处理数据
            while True:
                if not is_receiving_segment:
                    start_index = buffer.find('<START>')
                    if start_index == -1:
                        break
                    
                    buffer = buffer[start_index + len('<START>'):]
                    is_receiving_segment = True
                    current_segment = []
                    # 记录数据段开始
                    with open(log_filename, 'a', encoding='utf-8') as f:
                        f.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}] 开始新数据段\n")
                    continue
                
                end_index = buffer.find('<END>')
                
                while True:
                    line_end = buffer.find('\n')
                    if line_end == -1:
                        break
                        
                    line = buffer[:line_end].strip()
                    buffer = buffer[line_end + 1:]
                    
                    # 跳过空行
                    if not line:
                        continue
                    
                    # 处理位置数据
                    pos_match = re.match(r'POS:([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+)', line)
                    if pos_match:
                        x, y, yaw = map(float, pos_match.groups())
                        # 反转y坐标
                        y = -y
                        with data_lock:
                            car_trajectory.append((x, y, yaw))
                        # 记录解析后的位置数据
                        with open(log_filename, 'a', encoding='utf-8') as f:
                            f.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}] 位置数据: x={x}, y={y}, yaw={yaw}\n")
                    
                    # 处理扫描数据
                    scan_match = re.match(r'SCAN:([-+]?\d*\.?\d+),([-+]?\d*\.?\d+)', line)
                    if scan_match:
                        angle, distance = map(float, scan_match.groups())
                        # 记录原始扫描数据
                        with open(log_filename, 'a', encoding='utf-8') as f:
                            f.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}] 扫描数据: angle={angle}, distance={distance}\n")
                            
                        adjusted_angle = angle - 90
                        scan_x = distance * math.cos(math.radians(adjusted_angle))
                        scan_y = -distance * math.sin(math.radians(adjusted_angle))  # 反转y坐标
                        
                        if car_trajectory:
                            x, y, yaw = car_trajectory[-1]  # y已经是反转后的值
                            rotated_x = scan_x * math.cos(math.radians(yaw)) - scan_y * math.sin(math.radians(yaw))
                            rotated_y = scan_x * math.sin(math.radians(yaw)) + scan_y * math.cos(math.radians(yaw))
                            final_x = rotated_x + x
                            final_y = rotated_y + y
                            
                            # 记录计算后的扫描点坐标
                            with open(log_filename, 'a', encoding='utf-8') as f:
                                f.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}] 计算后的扫描点: x={final_x}, y={final_y}\n")
                            
                            with data_lock:
                                if distance < MAX_DETECTION_RANGE:
                                    # 添加原始点到缓存
                                    raw_points_buffer.append((final_x, final_y))
                                    # 当缓存达到窗口大小时进行滤波
                                    if len(raw_points_buffer) >= FILTER_WINDOW_SIZE:
                                        filtered = apply_moving_average_filter(raw_points_buffer, FILTER_WINDOW_SIZE)
                                        filtered_points.extend(filtered)
                                        raw_points_buffer = raw_points_buffer[len(filtered):]
                                        
                                    all_scan_points.append((final_x, final_y))
                                    accumulated_scan_points.append((final_x, final_y))
                                else:
                                    accumulated_scan_points.append((float('nan'), float('nan')))
                                update_queue.put(True)
                
                if end_index != -1:
                    with data_lock:
                        accumulated_scan_points.append((float('nan'), float('nan')))
                    
                    # 记录数据段结束
                    with open(log_filename, 'a', encoding='utf-8') as f:
                        f.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}] 数据段结束\n")
                    
                    buffer = buffer[end_index + len('<END>'):]
                    is_receiving_segment = False
                    break
                
                if len(buffer) > 1000:
                    # 记录缓冲区溢出警告
                    with open(log_filename, 'a', encoding='utf-8') as f:
                        f.write(f"[{datetime.now().strftime('%Y-%m-d %H:%M:%S.%f')}] 警告：缓冲区溢出，清空缓冲区\n")
                    buffer = ""
                    is_receiving_segment = False
                    break
                
                break

        except Exception as e:
            error_msg = f"WiFi数据处理错误: {e}\n{traceback.format_exc()}"
            print(error_msg)
            # 记录错误信息
            with open(log_filename, 'a', encoding='utf-8') as f:
                f.write(f"\n[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}] {error_msg}\n")
            break

    print("WiFi读取线程结束")
    # 记录线程结束信息
    with open(log_filename, 'a', encoding='utf-8') as f:
        f.write(f"\n[{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}] WiFi读取线程结束\n")
    client.close()

def update_plot():
    global car_indicator, scan_lines, distance_circles, distance_texts
    global zoom_level, current_xlim, current_ylim, manual_mode
    global car_x, car_y, car_yaw
    car_indicator = None

    while not exit_event.is_set():
        try:
            # 等待更新信号
            try:
                update_queue.get(timeout=UPDATE_INTERVAL)
            except Empty:
                pass  # Timeout, proceed to update

            with data_lock:
                # 更新所有扫描点,使用滤波后的点
                if filtered_points:
                    x_points, y_points = zip(*filtered_points)
                    scan_points_plot.set_data(x_points, y_points)
                else:
                    scan_points_plot.set_data([], [])

                # 更新累积的扫描线
                for line in scan_lines:
                    line.remove()
                scan_lines.clear()

                current_segment = []
                points_to_draw = filtered_points if filtered_points else accumulated_scan_points
                
                for point in points_to_draw:
                    if not (math.isnan(point[0]) or math.isnan(point[1])):
                        current_segment.append(point)
                    else:
                        if current_segment:
                            x_points, y_points = zip(*current_segment)
                            line, = ax.plot(x_points, y_points, 'b-', linewidth=1)
                            scan_lines.append(line)
                            current_segment = []
                
                if current_segment:
                    x_points, y_points = zip(*current_segment)
                    line, = ax.plot(x_points, y_points, 'b-', linewidth=1)
                    scan_lines.append(line)

                # 更新小车轨迹
                if car_trajectory:
                    traj_x, traj_y = zip(*[(p[0], p[1]) for p in car_trajectory])
                    trajectory_plot.set_data(traj_x, traj_y)
                    car_position_plot.set_data([traj_x[-1]], [traj_y[-1]])  # 更新小车当前位置

                # 更新小车指示器
                if 'car_indicator' in globals() and car_indicator:
                    car_indicator.remove()
                if len(car_trajectory) > 0:
                    car_x, car_y, car_yaw = car_trajectory[-1]
                    arrow_length = 10
                    dx = arrow_length * math.cos(math.radians(car_yaw))
                    dy = arrow_length * math.sin(math.radians(car_yaw))
                    car_indicator = ax.arrow(car_x, car_y, dx, dy,
                                             head_width=5, head_length=5, fc='r', ec='r')

                # 绘制距离信息圆环
                for circle in distance_circles:
                    circle.remove()
                distance_circles.clear()

                # 移除旧的圆环标注文字
                for text in distance_texts:
                    text.remove()
                distance_texts.clear()

                # 定义圆环的半径和颜色
                radii = [MAX_DETECTION_RANGE - 20, MAX_DETECTION_RANGE - 10, MAX_DETECTION_RANGE]
                colors = ['#ADD8E6', '#87CEEB', '#4682B4']  # 低饱和度颜色
                for radius, color in zip(radii, colors):
                    circle = plt.Circle((car_x, car_y), radius, color=color, fill=False, linestyle='--')
                    ax.add_patch(circle)
                    distance_circles.append(circle)
                    text = ax.text(car_x + radius, car_y, f'{radius:.1f} cm',
                                   color=color, fontsize=8, ha='left', va='bottom')
                    distance_texts.append(text)

                # 根据模式调整坐标轴范围
                if not manual_mode:
                    if accumulated_scan_points or car_trajectory:
                        all_points = accumulated_scan_points + [(p[0], p[1]) for p in car_trajectory]
                        x_points = [p[0] for p in all_points if not math.isnan(p[0])]
                        y_points = [p[1] for p in all_points if not math.isnan(p[1])]
                        if x_points and y_points:
                            x_min, x_max = min(x_points), max(x_points)
                            y_min, y_max = min(y_points), max(y_points)

                            # 确保坐标轴范围包含原点
                            x_min, x_max = min(x_min, 0), max(x_max, 0)
                            y_min, y_max = min(y_min, 0), max(y_max, 0)

                            # 如果所有点都在同一位置，添加一个最小范围
                            if abs(x_max - x_min) < 1e-10:
                                x_min -= 50
                                x_max += 50
                            if abs(y_max - y_min) < 1e-10:
                                y_min -= 50
                                y_max += 50

                            # 计算合适的边距
                            margin = max(abs(x_max - x_min), abs(y_max - y_min)) * 0.1
                            margin = max(margin, 10)  # 确保最小边距
                            
                            # 确保x和y的范围相等，保持正方形显示区域
                            x_range = max(abs(x_max - x_min), abs(y_max - y_min)) + 2 * margin
                            x_range = max(x_range, 100)  # 确保最小显示范围
                            
                            x_center = (x_max + x_min) / 2
                            y_center = (y_max + y_min) / 2
                            
                            # 注意y轴范围的设置顺序是相反的
                            ax.set_xlim(x_center - x_range/2, x_center + x_range/2 + 1e-6)
                            ax.set_ylim(y_center + x_range/2 + 1e-6, y_center - x_range/2)  # 反转y轴范围
                else:
                    # 确保手动模式下也不会有相同的上下限
                    if abs(current_xlim[1] - current_xlim[0]) < 1e-10:
                        center = current_xlim[0]
                        current_xlim = [center - 50, center + 50]
                    if abs(current_ylim[1] - current_ylim[0]) < 1e-10:
                        center = current_ylim[0]
                        current_ylim = [center + 50, center - 50]  # 注意顺序是相反的
                        
                    ax.set_xlim(current_xlim[0], current_xlim[1] + 1e-6)
                    ax.set_ylim(current_ylim[0], current_ylim[1])  # y轴范围已经是反转的

                # 更新标题
                if font:
                    ax.set_title(
                        f"小车位置: ({car_x:.2f}, {car_y:.2f}), 朝向: {car_yaw:.2f}°\n"
                        f"最大探测范围: {MAX_DETECTION_RANGE} 厘米    模式: {'手动' if manual_mode else '自动'}",
                        fontproperties=font)
                else:
                    ax.set_title(
                        f"Car Position: ({car_x:.2f}, {car_y:.2f}), Orientation: {car_yaw:.2f}°\n"
                        f"Max Detection Range: {MAX_DETECTION_RANGE} cm    Mode: {'Manual' if manual_mode else 'Auto'}")

            # 暂停以更新图形
            plt.pause(UPDATE_INTERVAL)

            # 检查窗口是否已关闭
            if not plt.fignum_exists(fig.number):
                exit_event.set()
                break

        except Exception as e:
            print(f"绘图更新错误: {e}")
            exit_event.set()
            break

def on_scroll(event):
    global zoom_level, current_xlim, current_ylim
    global manual_mode

    if not manual_mode:
        return  # 如果不是手动模式，忽略缩放事件

    # 获取鼠标位置对应的数据坐标
    xdata = event.xdata
    ydata = event.ydata
    if xdata is None or ydata is None:
        return

    # 缩放因子（反转方向，符合Windows习惯：向上滚动放大，向下滚动缩小）
    if event.button == 'up':
        scale_factor = 1.1  # 向上滚动放大
    elif event.button == 'down':
        scale_factor = 0.9  # 向下滚动缩小
    else:
        scale_factor = 1.0

    # 更新缩放级别
    zoom_level *= scale_factor

    # 获取当前坐标范围
    cur_xlim = ax.get_xlim()
    cur_ylim = ax.get_ylim()

    # 计算新的坐标范围
    new_width = (cur_xlim[1] - cur_xlim[0]) * scale_factor
    new_height = (cur_ylim[1] - cur_ylim[0]) * scale_factor

    # 计算新的坐标中心
    relx = (xdata - cur_xlim[0]) / (cur_xlim[1] - cur_xlim[0])
    rely = (ydata - cur_ylim[0]) / (cur_ylim[1] - cur_ylim[0])

    new_xlim = [xdata - new_width * relx, xdata + new_width * (1 - relx)]
    new_ylim = [ydata - new_height * rely, ydata + new_height * (1 - rely)]

    # 应用新的坐标范围
    current_xlim = new_xlim
    current_ylim = new_ylim

def on_press(event):
    global pan_start
    if not manual_mode:
        return  # 如果不是手动模式，忽略拖动事件
    if event.button == 1:
        pan_start = (event.x, event.y)

def on_release(event):
    global pan_start
    if not manual_mode:
        return  # 如果不是手动模式，忽略拖动事件
    if event.button == 1:
        pan_start = None

def on_motion(event):
    global pan_start, current_xlim, current_ylim
    global manual_mode

    if not manual_mode:
        return  # 如果不是手动模式，忽略拖动事件

    if pan_start is None or event.xdata is None or event.ydata is None:
        return
    if event.button != 1:
        return

    # 计算拖动距离（像素）
    dx = event.x - pan_start[0]
    dy = event.y - pan_start[1]

    # 获取当前坐标范围
    cur_xlim = ax.get_xlim()
    cur_ylim = ax.get_ylim()

    # 转换像素移动为数据移动
    # 获取比例：数据单位 / 点（pixel）
    inv = ax.transData.inverted()
    dx_data, dy_data = inv.transform((dx, dy)) - inv.transform((0, 0))

    # 调整移动幅度，增加移灵敏度
    pan_speed = 3.0  # 增加此值可以使移动更加灵敏
    new_xlim = [cur_xlim[0] - dx_data * pan_speed, cur_xlim[1] - dx_data * pan_speed]
    new_ylim = [cur_ylim[0] - dy_data * pan_speed, cur_ylim[1] - dy_data * pan_speed]

    current_xlim = new_xlim
    current_ylim = new_ylim

    # 更新起始点
    pan_start = (event.x, event.y)

def toggle_mode(event):
    global manual_mode, current_xlim, current_ylim
    global car_x, car_y, car_yaw

    manual_mode = not manual_mode
    if manual_mode:
        print("切换到手动模式")
    else:
        print("切换到自动模式")
    
    # 如果在手动模式下没有数据显示，可能需要点击开始回放
    if manual_mode and not (accumulated_scan_points or car_trajectory):
        print("请点击'开始回放'按钮开始数据回放")

    # 如果切换到自动模式，重置坐标轴以适应所有数据
    if not manual_mode:
        with data_lock:
            if accumulated_scan_points or car_trajectory:
                all_points = accumulated_scan_points + [(p[0], p[1]) for p in car_trajectory]
                x_points = [p[0] for p in all_points if not math.isnan(p[0])]
                y_points = [p[1] for p in all_points if not math.isnan(p[1])]
                if x_points and y_points:
                    x_min, x_max = min(x_points), max(x_points)
                    y_min, y_max = min(y_points)

                    # 确保坐标轴范围包含原点
                    x_min, x_max = min(x_min, 0), max(x_max, 0)
                    y_min, y_max = min(y_min, 0), max(y_max, 0)

                    # 如果所有点都在同一位置，添加一个最小范围
                    if abs(x_max - x_min) < 1e-10:
                        x_min -= 50
                        x_max += 50
                    if abs(y_max - y_min) < 1e-10:
                        y_min -= 50
                        y_max += 50

                    # 计算合适的边距
                    margin = max(abs(x_max - x_min), abs(y_max - y_min)) * 0.1
                    margin = max(margin, 10)  # 确保最小边距
                    
                    # 确保x和y的范围相等，保持正方形显示区域
                    x_range = max(abs(x_max - x_min), abs(y_max - y_min)) + 2 * margin
                    x_range = max(x_range, 100)  # 确保最小显示范围
                    
                    x_center = (x_max + x_min) / 2
                    y_center = (y_max + y_min) / 2
                    
                    # 添加一个小的偏移量，确保范围永远不会完全相同
                    current_xlim = [x_center - x_range/2, x_center + x_range/2 + 1e-6]
                    current_ylim = [y_center - x_range/2, y_center + x_range/2 + 1e-6]
    else:
        # 如果切换到手动模式，保持当前坐标轴不变
        pass

    # 更新标题反映当前模式
    if font:
        ax.set_title(
            f"小车位置: ({car_x:.2f}, {car_y:.2f}), 朝向: {car_yaw:.2f}°\n"
            f"最大探测范围: {MAX_DETECTION_RANGE} 厘米    模式: {'手动' if manual_mode else '自动'}",
            fontproperties=font)
    else:
        ax.set_title(
            f"Car Position: ({car_x:.2f}, {car_y:.2f}), Orientation: {car_yaw:.2f}°\n"
            f"Max Detection Range: {MAX_DETECTION_RANGE} cm    Mode: {'Manual' if manual_mode else 'Auto'}")

def start_replay(event):
    """开始回放按钮的回调函数"""
    if not start_replay_event.is_set():
        print("开始回放数据...")
        start_replay_event.set()

def main():
    global current_xlim, current_ylim, client
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='WiFi绘图工具')
    parser.add_argument('--log', type=str, help='指定要读取的日志文件路径')
    args = parser.parse_args()
    
    # 根据命令行参数决定数据来源
    if args.log:
        # 从日志文件读取数据
        log_thread = threading.Thread(target=read_log_file, args=(args.log,), daemon=True)
        log_thread.start()
    else:
        # 创建socket连接
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print(f"正在连接到 {HOST}:{PORT}...")
            client.connect((HOST, PORT))
            print("WiFi连接成功！")
            
            # 创建并启动WiFi数据读取线程
            wifi_thread = threading.Thread(target=read_wifi_data, daemon=True)
            wifi_thread.start()
        except socket.error as e:
            print(f"无法连接到WiFi服务器。错误信息: {e}")
            print("请检查:")
            print("1. ESP32是否正确配置为AP模式")
            print("2. IP地址和端口是否正确")
            print("3. ESP32是否正常工作")
            sys.exit(1)

    # 连接窗口关闭事件
    fig.canvas.mpl_connect('close_event', lambda event: exit_event.set())

    # 连接鼠标事件
    fig.canvas.mpl_connect('scroll_event', on_scroll)
    fig.canvas.mpl_connect('button_press_event', on_press)
    fig.canvas.mpl_connect('button_release_event', on_release)
    fig.canvas.mpl_connect('motion_notify_event', on_motion)

    # 添加状态切换按钮
    ax_mode_button = plt.axes([0.81, 0.01, 0.1, 0.05])
    mode_button = Button(ax_mode_button, '切换模式', hovercolor='0.975')
    mode_button.on_clicked(toggle_mode)

    # 添加开始回放按钮
    ax_start_button = plt.axes([0.69, 0.01, 0.1, 0.05])
    start_button = Button(ax_start_button, '开始回放', hovercolor='0.975')
    start_button.on_clicked(start_replay)

    try:
        update_plot()
    except KeyboardInterrupt:
        print("程序被用户中断")
    except Exception as e:
        print(f"发生未知错误: {e}")
        traceback.print_exc()
    finally:
        exit_event.set()
        if client:
            try:
                client.close()
                print("WiFi连接已关闭")
            except:
                print("关闭WiFi连接时出错")
        plt.close('all')
        sys.exit(0)

if __name__ == "__main__":
    main()