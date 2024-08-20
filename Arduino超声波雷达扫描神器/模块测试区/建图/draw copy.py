import matplotlib.pyplot as plt
import numpy as np
import serial
import math
import time
from matplotlib.font_manager import FontProperties

try:
    font = FontProperties(fname=r"C:\Windows\Fonts\simhei.ttf", size=10)
except:
    font = None
    print("无法加载中文字体，将使用默认字体")

# 设置串口通信
try:
    ser = serial.Serial('COM9', 115200, timeout=1)
    print("成功打开串口 COM9")
except serial.SerialException:
    print("无法打开串口 COM9。请检查连接和端口设置。")
    exit()

# 初始化图形
plt.ion()
fig, ax = plt.subplots()
ax.set_xlim(-500, 500)
ax.set_ylim(-500, 500)
ax.grid(True)

# 扫描点
scan_points, = ax.plot([], [], 'b.', markersize=2)

def update_plot(angles, distances, x, y, yaw):
    if len(angles) == 0 or len(distances) == 0:
        print("没有数据可以绘制")
        return
    # 转换极坐标到笛卡尔坐标
    scan_x = distances * np.cos(np.radians(angles))
    scan_y = distances * np.sin(np.radians(angles))
    
    # 应用小车的位置和朝向
    rotated_x = scan_x * math.cos(math.radians(yaw)) - scan_y * math.sin(math.radians(yaw))
    rotated_y = scan_x * math.sin(math.radians(yaw)) + scan_y * math.cos(math.radians(yaw))
    final_x = rotated_x + x
    final_y = rotated_y + y
    
    scan_points.set_data(final_x, final_y)
    if font:
        ax.set_title(f"小车位置: ({x:.2f}, {y:.2f}), 朝向: {yaw:.2f}°", fontproperties=font)
    else:
        ax.set_title(f"Car Position: ({x:.2f}, {y:.2f}), Orientation: {yaw:.2f}°")
    fig.canvas.draw()
    fig.canvas.flush_events()
    print(f"更新图形: 小车位置 ({x:.2f}, {y:.2f}), 朝向 {yaw:.2f}°")

def clean_buffer(buffer):
    return ''.join(char for char in buffer if char.isprintable() or char in '\n\r')

try:
    buffer = ""
    in_data_section = False
    angles = []
    distances = []
    x, y, yaw = 0, 0, 0
    print("开始监听串口数据...")
    while True:
        if ser.in_waiting:
            new_data = ser.read(ser.in_waiting).decode(errors='replace')
            buffer += new_data
            print(f"接收到新数据: {new_data}")
            
            buffer = clean_buffer(buffer)
            
            if "START" in buffer:
                buffer = buffer[buffer.index("START") + 5:]
                in_data_section = True
                angles = []
                distances = []
                print("检测到开始标记")
            
            if in_data_section:
                lines = buffer.split('\n')
                for line in lines[:-1]:  # 处理除最后一行外的所有完整行
                    line = line.strip()
                    print(f"处理行: {line}")
                    if "END" in line:
                        in_data_section = False
                        if angles and distances:
                            update_plot(np.array(angles), np.array(distances), x, y, yaw)
                        print("检测到结束标记")
                        break
                    parts = line.split(',')
                    if len(parts) == 3:  # x, y, yaw 数据
                        try:
                            x, y, yaw = map(float, parts)
                            print(f"小车位置更新: x={x}, y={y}, yaw={yaw}")
                        except ValueError:
                            print(f"无法解析位置数据: {line}")
                    elif len(parts) == 2:  # 角度和距离数据
                        try:
                            angle, distance = map(float, parts)
                            angles.append(angle)
                            distances.append(distance)
                            print(f"添加扫描点: 角度={angle}, 距离={distance}")
                        except ValueError:
                            print(f"无法解析扫描数据: {line}")
                buffer = lines[-1]  # 保留最后一个不完整的行
        
        # 给系统一些时间处理其他任务
        time.sleep(0.001)

except KeyboardInterrupt:
    print("程序被用户中断")
except serial.SerialException as e:
    print(f"串口通信错误: {e}")
except Exception as e:
    print(f"发生未知错误: {e}")
finally:
    ser.close()
    print("串口已关闭")
    plt.ioff()
    plt.show()
