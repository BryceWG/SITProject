import matplotlib.pyplot as plt
import serial
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

# 设置最大探测范围（单位：厘米）
MAX_DETECTION_RANGE = 50

# 尝试加载支持中文的字体
try:
    font_path = r"C:\Windows\Fonts\simhei.ttf"  # 黑体路径
    font = FontProperties(fname=font_path, size=10)
    matplotlib.rcParams['font.family'] = ['SimHei']  # 修改这里
    matplotlib.rcParams['font.sans-serif'] = ['SimHei']  # 添加这行
    matplotlib.rcParams['axes.unicode_minus'] = False  # 解决减号显示问题
except FileNotFoundError:
    font = None
    print("无法加载中文字体，将使用默认字体")
    # 设置 Matplotlib 使用支持中文的默认字体
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS']  # 添加多个备选字体
    plt.rcParams['axes.unicode_minus'] = False

# 设置串口通信
try:
    ser = serial.Serial('COM6', 115200, timeout=1)
    print(f"成功打开串口 COM6，最大探测范围设置为 {MAX_DETECTION_RANGE} 厘米")
    # 添加串口测试
    ser.write(b'test\n')  # 发送测试数据
    time.sleep(0.1)  # 等待响应
    if ser.in_waiting:
        print(f"串口测试响应: {ser.read(ser.in_waiting).decode(errors='replace')}")
except serial.SerialException as e:
    print(f"无法打开串口 COM6。错误信息: {e}")
    print("请检查:")
    print("1. 串口号是否正确")
    print("2. 串口是否被其他程序占用")
    print("3. 设备是否正确连接")
    sys.exit(1)

# 始化图形
plt.ion()
fig, ax = plt.subplots(figsize=(10, 10))  # 设置一个较大的初始图形大小
ax.set_aspect('equal', 'datalim')
ax.grid(True)

# 设置初始显示范围
ax.set_xlim(-150, 150)  # 设置初始 x 轴范围
ax.set_ylim(-150, 150)  # 设置初始 y 轴范围

# 所有扫描点
all_scan_points = []
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
distance_texts = []  # 用于存储圆环标注文字对象

# 减少绘图更新频率
UPDATE_INTERVAL = 0.1  # 每0.1秒更新一次图形

# 添加退出事件
exit_event = threading.Event()

# 添加变量用于缩放和平移
zoom_level = 1.0
pan_start = None
current_xlim = ax.get_xlim()
current_ylim = ax.get_ylim()

# 添加状态切换按钮相关变量
manual_mode = False  # 初始为自模式

# 初始化全局小车位置变量
car_x = 0.0
car_y = 0.0
car_yaw = 0.0

def update_plot():
    global car_indicator, scan_lines, distance_circles, distance_texts
    global zoom_level, current_xlim, current_ylim, manual_mode
    global car_x, car_y, car_yaw  # 声明为全局变量
    car_indicator = None

    while not exit_event.is_set():
        try:
            # 等待更新信号
            try:
                update_queue.get(timeout=UPDATE_INTERVAL)
            except Empty:
                pass  # Timeout, proceed to update

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

                            margin = 50
                            ax.set_xlim(x_min - margin, x_max + margin)
                            ax.set_ylim(y_min - margin, y_max + margin)
                else:
                    ax.set_xlim(current_xlim)
                    ax.set_ylim(current_ylim)

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

def read_serial():
    global current_scan_points, all_scan_points, accumulated_scan_points, car_trajectory
    current_scan_points = []
    buffer = ""
    
    # 使用独特的开始和结束标记，以便精确定位数据块
    pattern_start = re.compile(r'<START>')
    pattern_end = re.compile(r'<END>')
    # 为位置和扫描数据添加明确的前缀标识
    pattern_position = re.compile(r'POS:([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+)')
    pattern_scan = re.compile(r'SCAN:([-+]?\d*\.?\d+),([-+]?\d*\.?\d+)')
    
    while not exit_event.is_set():
        try:
            if ser.in_waiting:
                new_data = ser.read(ser.in_waiting).decode(errors='replace')
                print(new_data)
                buffer += new_data
    
                # 处理完整的数据块
                while True:
                    start_index = buffer.find('<START>')
                    if start_index == -1:
                        break
                    
                    end_index = buffer.find('<END>', start_index)
                    if end_index == -1:
                        break  # 等待更多数据
                        
                    data_block = buffer[start_index:end_index + len('<END>')]
                    print(f"处理数据块: {data_block}")
                    
                    # 处理位置数据
                    pos_match = pattern_position.search(data_block)
                    if pos_match:
                        x, y, yaw = map(float, pos_match.groups())
                        print(f"更新位置: x={x}, y={y}, yaw={yaw}")
                        with data_lock:
                            car_trajectory.append((x, y, yaw))
                    
                    # 处理扫描数据
                    scan_matches = pattern_scan.finditer(data_block)
                    for match in scan_matches:
                        angle, distance = map(float, match.groups())
                        print(f"处理扫描点: angle={angle}, distance={distance}")
                        if distance < MAX_DETECTION_RANGE:
                            adjusted_angle = angle - 90
                            scan_x = distance * math.cos(math.radians(adjusted_angle))
                            scan_y = distance * math.sin(math.radians(adjusted_angle))
                            
                            # 使用最新的车辆位置进行坐标转换
                            if car_trajectory:
                                x, y, yaw = car_trajectory[-1]
                                rotated_x = scan_x * math.cos(math.radians(yaw)) - scan_y * math.sin(math.radians(yaw))
                                rotated_y = scan_x * math.sin(math.radians(yaw)) + scan_y * math.cos(math.radians(yaw))
                                final_x = rotated_x + x
                                final_y = rotated_y + y
                                
                                with data_lock:
                                    all_scan_points.append((final_x, final_y))
                                    current_scan_points.append((final_x, final_y))
                                    accumulated_scan_points.append((final_x, final_y))
                    
                    # 移除已处理的数据块
                    buffer = buffer[end_index + len('<END>'):]
                    
                    # 触发图形更新
                    update_queue.put(True)
                    
        except Exception as e:
            print(f"数据处理错误: {e}")
            traceback.print_exc()  # 打印详细错误信息
            continue

    print("串口读取线程结束")

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
    global car_x, car_y, car_yaw  # 声明为全局变量

    manual_mode = not manual_mode
    if manual_mode:
        print("切换到手动模式")
    else:
        print("切换到自动模式")
    # 如果切换到自动模式，重置坐标轴以适应所有数据
    if not manual_mode:
        with data_lock:
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
                    current_xlim = [x_min - margin, x_max + margin]
                    current_ylim = [y_min - margin, y_max + margin]
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

def main():
    global current_xlim, current_ylim

    # 创建并启动线程
    serial_thread = threading.Thread(target=read_serial, daemon=True)
    serial_thread.start()

    # 连接窗口关闭事件
    fig.canvas.mpl_connect('close_event', lambda event: exit_event.set())

    # 连接鼠标事件
    fig.canvas.mpl_connect('scroll_event', on_scroll)
    fig.canvas.mpl_connect('button_press_event', on_press)
    fig.canvas.mpl_connect('button_release_event', on_release)
    fig.canvas.mpl_connect('motion_notify_event', on_motion)

    # 添加状态切换按钮
    ax_button = plt.axes([0.81, 0.01, 0.1, 0.05])  # [left, bottom, width, height]
    button = Button(ax_button, '切换模式', hovercolor='0.975')

    button.on_clicked(toggle_mode)

    try:
        update_plot()   # 更新图形
    except KeyboardInterrupt:
        print("程序被用户中断")
    except Exception as e:
        print(f"发生未知错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 设置退出事件
        exit_event.set()
        # 等待串口线程结束
        serial_thread.join(timeout=1.0)
        try:
            ser.close()
            print("串口已关闭")
        except:
            print("关闭串口时出错")
        plt.close('all')  # 关闭所有图形窗口
        sys.exit(0)  # 强制退出程序

if __name__ == "__main__":
    main()