import matplotlib.pyplot as plt
import numpy as np
import serial
import math
import time

# 设置串口通信
try:
    ser = serial.Serial('COM9', 115200, timeout=1)  # 注意：可能需要调整波特率
except serial.SerialException:
    print("无法打开串口 COM9。请检查连接和端口设置。")
    exit()

# 初始化图形
plt.ion()
fig, ax = plt.subplots()
ax.set_xlim(-500, 500)  # 根据实际情况调整范围
ax.set_ylim(-500, 500)
ax.grid(True)

# 小车位置和朝向
car_position, = ax.plot([], [], 'ro', markersize=10)
car_direction, = ax.plot([], [], 'r-', linewidth=2)
# 扫描点
scan_points, = ax.plot([], [], 'b.', markersize=2)

def rotate_point(x, y, angle):
    """围绕原点旋转点"""
    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)
    return x * cos_theta - y * sin_theta, x * sin_theta + y * cos_theta

def is_valid_scan_data(data):
    """检查是否为有效的扫描数据"""
    values = data.split(',')
    return len(values) > 363 and all(v.replace('.', '', 1).replace('-', '', 1).isdigit() for v in values[:3])

try:
    while True:
        # 读取数据直到找到有效的扫描数据
        while True:
            data = ser.readline().decode().strip()
            if is_valid_scan_data(data):
                break
            else:
                print("接收到非扫描数据:", data)

        try:
            # 解析数据
            values = data.split(',')
            car_x, car_y, car_orientation = map(float, values[:3])
            scan_data = list(map(float, values[3:-1]))  # 去掉最后一个空值

            # 更新小车位置和朝向
            car_position.set_data(car_x, car_y)
            direction_x = car_x + 20 * math.cos(math.radians(car_orientation))
            direction_y = car_y + 20 * math.sin(math.radians(car_orientation))
            car_direction.set_data([car_x, direction_x], [car_y, direction_y])

            # 计算并更新扫描点
            x_points = []
            y_points = []
            for i in range(0, len(scan_data), 2):
                angle = i // 2  # 角度从0开始
                distance = scan_data[i+1]
                if distance < 50:  # 假设50是最大有效距离
                    # 计算相对于小车的点位置
                    relative_angle = math.radians(angle)
                    relative_x = distance * math.cos(relative_angle)
                    relative_y = distance * math.sin(relative_angle)
                    
                    # 旋转点以匹配小车朝向
                    rotated_x, rotated_y = rotate_point(relative_x, relative_y, math.radians(car_orientation))
                    
                    # 转换为全局坐标
                    global_x = car_x + rotated_x
                    global_y = car_y + rotated_y
                    
                    x_points.append(global_x)
                    y_points.append(global_y)
            
            scan_points.set_data(x_points, y_points)

            # 刷新图形
            fig.canvas.draw()
            fig.canvas.flush_events()

            # 短暂暂停，给系统时间处理其他任务
            time.sleep(0.1)

        except ValueError as e:
            print(f"数据格式错误: {e}")
            print(f"问题数据: {data}")

        except serial.SerialException:
            print("串口通信错误")
            break

except KeyboardInterrupt:
    print("程序被用户中断")

finally:
    # 关闭串口连接
    ser.close()
    plt.ioff()
    plt.show()
