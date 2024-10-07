import matplotlib.pyplot as plt
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
    font = FontProperties(fname=r"C:\\Windows\\Fonts\\simhei.ttf", size=10)
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
ax.set_xlim(-150, 150)  # 设置初始 x 轴范围
ax.set_ylim(-150, 150)  # 设置初始 y 轴范围

# 所有扫描点
all_scan_points = []    #
accumulated_scan_points = []
current_scan_points = []
car_trajectory = [(0, 0, 0)]  # 初始化轨迹，起始点为原点

# 创建绘图对象
scan_points_plot, = ax.plot([], [], 'b.', markersize=2)
trajectory_plot, = ax.plot([], [], 'g-', linewidth=2)
car_position_plot, = ax.plot([], [], 'ro', markersize=5)
scan_line, = ax.plot([], [], 'b-', linewidth=1)

# 用于线程间通信的锁
data_lock = threading.Lock()

# 创建一个队列用于线程间通信
update_queue = Queue()

# 在全局变量部分添加：
scan_lines = []
distance_circles = []

def update_plot():
    global car_indicator, scan_lines, distance_circles
    car_indicator = None
    car_x, car_y, car_yaw = 0, 0, 0

    while True:
        try:
            # 等待更新信号
            update_queue.get(timeout=0.1)
        except Queue.Empty:
            continue
        
        with data_lock:
            # 更新所有扫描点
            if all_scan_points:
                # 过滤掉无效点（NaN）
                valid_points = [(x, y) for x, y in all_scan_points if not (math.isnan(x) or math.isnan(y))]
                if valid_points:
                    x_points, y_points = zip(*valid_points)
                    scan_points_plot.set_data(x_points, y_points)
                else:
                    scan_points_plot.set_data([], [])

            # 更新累积的扫描线
            if accumulated_scan_points:
                # 移除旧的扫描线
                for line in scan_lines:
                    line.remove()
                scan_lines.clear()

                # 绘制新的扫描线
                current_segment = []
                for point in accumulated_scan_points:
                    if math.isnan(point[0]) or math.isnan(point[1]):
                        if current_segment:
                            x_points, y_points = zip(*current_segment)
                            line, = ax.plot(x_points, y_points, 'b-', linewidth=1)
                            scan_lines.append(line)
                            current_segment = []
                    else:
                        current_segment.append(point)
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
            if car_indicator:
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
            
            # 定义圆环的半径和颜色
            radii = [MAX_DETECTION_RANGE - 20, MAX_DETECTION_RANGE - 10, MAX_DETECTION_RANGE]
            colors = ['#ADD8E6', '#87CEEB', '#4682B4']  # 低饱和度颜色
            for radius, color in zip(radii, colors):
                circle = plt.Circle((car_x, car_y), radius, color=color, fill=False, linestyle='--')
                ax.add_patch(circle)
                distance_circles.append(circle)
                ax.text(car_x + radius, car_y, f'{radius:.1f} cm', color=color, fontsize=8, ha='left', va='bottom')

            # 调整坐标轴范围
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
                    
                    margin = 50
                    ax.set_xlim(x_min - margin, x_max + margin)
                    ax.set_ylim(y_min - margin, y_max + margin)
            
            # 更新标题
            if font:
                ax.set_title(f"小车位置: ({car_x:.2f}, {car_y:.2f}), 朝向: {car_yaw:.2f}°\n最大探测范围: {MAX_DETECTION_RANGE} 厘米", fontproperties=font)
            else:
                ax.set_title(f"Car Position: ({car_x:.2f}, {car_y:.2f}), Orientation: {car_yaw:.2f}°\nMax Detection Range: {MAX_DETECTION_RANGE} cm")

        # 暂停以更新图形
        plt.pause(0.01)

def read_serial():
    global current_scan_points, all_scan_points, accumulated_scan_points, car_trajectory
    current_scan_points = []
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
                    # 在每轮扫描开始时添加一个断点
                    with data_lock:
                        accumulated_scan_points.append((float('nan'), float('nan')))

                # 处理数据
                while in_data_section:
                    # 检查结束标志
                    if 'END' in buffer:
                        buffer = buffer[buffer.index('END') + len('END'):]
                        in_data_section = False
                        print("检测到结束标记")
                        current_scan_points = []  # 清空当前扫描点
                        # 在每轮扫描结束时添加一个断点
                        with data_lock:
                            accumulated_scan_points.append((float('nan'), float('nan')))
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
                        angle, distance = map(float, match_scan.groups())   # 提取角度和距离
                        if distance < MAX_DETECTION_RANGE:  # 仅绘制有效数据
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

                            with data_lock: # 使用锁以确保线程安全
                                all_scan_points.append((final_x, final_y))  # 添加到所有扫描点
                                current_scan_points.append((final_x, final_y))  # 添加到当前扫描点
                                accumulated_scan_points.append((final_x, final_y))  # 添加到累积扫描点
                        else:
                            with data_lock: 
                                current_scan_points.append((float('nan'), float('nan')))
                                accumulated_scan_points.append((float('nan'), float('nan')))

                        buffer = buffer[match_scan.end():]
                        continue

                    # 如果没有匹配到任何数据，等待更多数据
                    break

            # 在每次更新数据后，向队列发送更新信号
            update_queue.put(True)  # 向队列发送更新信号
            time.sleep(0.01)  # 短暂休眠以避免CPU过度使用

        except serial.SerialException:
            print("串口连接已断开")
            break
        except Exception as e:
            print(f"串口读取错误: {e}")
            break

    print("串口读取线程结束")

def main():
    # 创建并启动线程
    serial_thread = threading.Thread(target=read_serial, daemon=True)
    serial_thread.start()

    try:
        update_plot()   # 更新图形
    except KeyboardInterrupt:
        print("程序被用户中断")
    except Exception as e:
        print(f"发生未知错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 等待串口线程结束
        serial_thread.join(timeout=1.0)
        try:
            ser.close()
            print("串口已关闭")
        except:
            print("关闭串口时出错")
        plt.close()  # 关闭所有图形窗口
        sys.exit(0)  # 强制退出程序

if __name__ == "__main__":
    main()