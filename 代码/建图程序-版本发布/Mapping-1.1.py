import matplotlib.pyplot as plt
import numpy as np
import serial
import math
import time
from matplotlib.font_manager import FontProperties
import threading
import re
from queue import Queue
import sys

# 设置最大探测范围（单位：厘米）
MAX_DETECTION_RANGE = 50

# 尝试加载支持中文的字体
try:
    font = FontProperties(fname=r"C:\Windows\Fonts\simhei.ttf", size=10)
except:
    font = None
    print("无法加载中文字体，将使用默认字体")

# 设置串口通信
try:
    ser = serial.Serial('COM3', 115200, timeout=1)
    print(f"成功打开串口 COM3，最大探测范围设置为 {MAX_DETECTION_RANGE} 厘米")
except serial.SerialException:
    print("无法打开串口 COM3。请检查连接和端口设置。")
    exit()

# 初始化图形
plt.ion()
fig, ax = plt.subplots(figsize=(10, 10))  # 设置一个较大的初始图形大小
ax.set_aspect('equal', 'datalim')
ax.grid(True)

# 设置初始显示范围
ax.set_xlim(-100, 100)  # 设置初始 x 轴范围
ax.set_ylim(-100, 100)  # 设置初始 y 轴范围

# 所有扫描点
all_scan_points = []
# 小车轨迹点
car_trajectory = [(0, 0, 0)]  # 初始化轨迹，起始点为原点

# 创建绘图对象
scan_points_plot, = ax.plot([], [], 'b.', markersize=2)
trajectory_plot, = ax.plot([], [], 'g-', linewidth=2)
car_positions_plot, = ax.plot([], [], 'ro', markersize=5)  # 新增：用于显示小车位置的点
car_indicator = None  # 小车指示器

# 用于线程间通信的锁
data_lock = threading.Lock()

# 创建一个队列用于线程间通信
update_queue = Queue()

# 在全局变量部分添加：
current_scan_points = []
scan_lines = None
object_lines = None

# 修改 update_plot 函数
def update_plot():
    global car_indicator, scan_lines, object_lines
    car_indicator = None
    car_x, car_y, car_yaw = 0, 0, 0

    while True:
        try:
            update_queue.get(timeout=0.1)
        except Queue.Empty:
            continue
        
        with data_lock:
            if all_scan_points:
                # 过滤掉无效点
                valid_points = [(x, y) for x, y in all_scan_points if not (math.isnan(x) or math.isnan(y))]
                if valid_points:
                    x_points, y_points = zip(*valid_points)
                    scan_points_plot.set_data(x_points, y_points)
                else:
                    scan_points_plot.set_data([], [])

            if car_trajectory:
                traj_x, traj_y = zip(*[(p[0], p[1]) for p in car_trajectory])
                trajectory_plot.set_data(traj_x, traj_y)
                
                # 更新小车位置点
                car_positions_plot.set_data(traj_x, traj_y)

            # 更新小车指示器
            if car_indicator:
                car_indicator.remove()

            # 绘制小车位置和朝向
            if len(car_trajectory) > 0:
                car_x, car_y, car_yaw = car_trajectory[-1]
                
                # 创建箭头
                arrow_length = 10
                dx = arrow_length * math.cos(math.radians(car_yaw))
                dy = arrow_length * math.sin(math.radians(car_yaw))
                car_indicator = ax.arrow(car_x, car_y, dx, dy, 
                                         head_width=5, head_length=5, fc='r', ec='r')
                ax.add_patch(car_indicator)

            # 更新扫描线
            if current_scan_points:
                # 过滤掉无效点，但保持断开
                valid_segments = []
                current_segment = []
                for point in current_scan_points:
                    if math.isnan(point[0]) or math.isnan(point[1]):
                        if current_segment:
                            valid_segments.append(current_segment)
                            current_segment = []
                    else:
                        current_segment.append(point)
                if current_segment:
                    valid_segments.append(current_segment)

                if scan_lines:
                    for line in scan_lines:
                        line.remove()
                scan_lines = []
                for segment in valid_segments:
                    x_points, y_points = zip(*segment)
                    line, = ax.plot(x_points, y_points, 'b-', linewidth=1)
                    scan_lines.append(line)

                # 绘制物体分割线
                if object_lines:
                    for line in object_lines:
                        line.remove()
                object_lines = []

                for i in range(len(current_scan_points) - 1):
                    p1 = current_scan_points[i]
                    p2 = current_scan_points[i + 1]
                    distance = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                    if distance > 20:  # 可以根据需要调整这个阈值
                        line = ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'r-', linewidth=2)[0]
                        object_lines.append(line)

            # 调整坐标轴范围
            if all_scan_points or car_trajectory:
                all_points = all_scan_points + [(p[0], p[1]) for p in car_trajectory]
                x_points, y_points = zip(*all_points)
                x_min, x_max = min(x_points), max(x_points)
                y_min, y_max = min(y_points), max(y_points)
                
                # 确保原点始终可见
                x_min = min(x_min, 0)
                x_max = max(x_max, 0)
                y_min = min(y_min, 0)
                y_max = max(y_max, 0)
                
                # 添加一些边距
                margin = 50  # 可以根据需要调整
                ax.set_xlim(x_min - margin, x_max + margin)
                ax.set_ylim(y_min - margin, y_max + margin)
            
            # 更新标题
            if font:
                ax.set_title(f"小车位置: ({car_x:.2f}, {car_y:.2f}), 朝向: {car_yaw:.2f}°\n最大探测范围: {MAX_DETECTION_RANGE} 厘米", fontproperties=font)
            else:
                ax.set_title(f"Car Position: ({car_x:.2f}, {car_y:.2f}), Orientation: {car_yaw:.2f}°\nMax Detection Range: {MAX_DETECTION_RANGE} cm")

        plt.pause(0.01)

# 修改 read_serial 函数
def read_serial():
    global current_scan_points, all_scan_points
    buffer = ""
    in_data_section = False
    x, y, yaw = 0, 0, 0
    pattern_start = re.compile(r'START')
    pattern_end = re.compile(r'END')
    pattern_position = re.compile(r'([-+]?\d*\.\d+|\d+),([-+]?\d*\.\d+|\d+),([-+]?\d*\.\d+|\d+)')
    pattern_scan = re.compile(r'([-+]?\d*\.\d+|\d+),([-+]?\d*\.\d+|\d+)')
    
    while True:
        try:
            if ser.in_waiting:
                new_data = ser.read(ser.in_waiting).decode(errors='replace')
                buffer += new_data

                # 检查开始标志
                if not in_data_section and 'START' in buffer:
                    buffer = buffer[buffer.index('START') + len('START'):]
                    in_data_section = True
                    print("检测到开始标记")

                # 处理数据
                while in_data_section:
                    # 检查结束标志
                    if 'END' in buffer:
                        buffer = buffer[buffer.index('END') + len('END'):]
                        in_data_section = False
                        print("检测到结束标记")
                        current_scan_points = []  # 清空当前扫描点
                        continue

                    # 提取位置数据
                    match_position = pattern_position.search(buffer)
                    if match_position:
                        x, y, yaw = map(float, match_position.groups())
                        with data_lock:
                            car_trajectory.append((x, y, yaw))
                        buffer = buffer[match_position.end():]
                        continue

                    # 提取扫描数据
                    match_scan = pattern_scan.search(buffer)
                    if match_scan:
                        angle, distance = map(float, match_scan.groups())
                        # 只处理在最大探测范围内的点
                        if distance <= MAX_DETECTION_RANGE:
                            # 更新扫描点
                            # 将角度从0-180度映射到-90到90度
                            adjusted_angle = angle - 90
                            scan_x = distance * math.cos(math.radians(adjusted_angle))
                            scan_y = distance * math.sin(math.radians(adjusted_angle))

                            # 应用小车的位置和朝向
                            rotated_x = scan_x * math.cos(math.radians(yaw)) - scan_y * math.sin(math.radians(yaw))
                            rotated_y = scan_x * math.sin(math.radians(yaw)) + scan_y * math.cos(math.radians(yaw))
                            final_x = rotated_x + x
                            final_y = rotated_y + y

                            with data_lock:
                                all_scan_points.append((final_x, final_y))
                                current_scan_points.append((final_x, final_y))
                        else:
                            # 如果点超出范围，添加一个无效点来表示断开
                            with data_lock:
                                current_scan_points.append((float('nan'), float('nan')))

                        buffer = buffer[match_scan.end():]
                        continue

                    # 如果没有匹配到任何数据，等待更多数据
                    break

            # 在每次更新数据后，向队列发送更新信号
            update_queue.put(True)
            time.sleep(0.01)  # 短暂休眠以避免CPU过度使用

        except Exception as e:
            print(f"串口读取错误: {e}")
            break

# 主循环
def main():
    # 创建并启动线程
    serial_thread = threading.Thread(target=read_serial, daemon=True)
    serial_thread.start()

    try:
        while True:
            update_plot()
    except KeyboardInterrupt:
        print("程序被用户中断")
    except Exception as e:
        print(f"发生未知错误: {e}")
    finally:
        # 等待串口线程结束
        serial_thread.join(timeout=1.0)
        ser.close()
        print("串口已关闭")
        plt.close()  # 关闭所有图形窗口
        sys.exit(0)  # 强制退出程序

if __name__ == "__main__":
    main()