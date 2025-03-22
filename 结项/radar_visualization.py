import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import csv
import math

# 读取CSV文件数据
def read_radar_data(file_path):
    angles = []
    distances = []
    
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            if row and row[0]:  # 确保行不为空且第一个元素存在
                try:
                    # 解析角度和距离
                    angle_distance = row[0].strip('"').split(',')
                    if len(angle_distance) == 2:
                        angle = float(angle_distance[0])
                        distance = float(angle_distance[1])
                        angles.append(angle)
                        distances.append(distance)
                except (ValueError, IndexError) as e:
                    print(f"跳过无效行: {row}, 错误: {e}")
    
    return angles, distances

# 将极坐标转换为笛卡尔坐标
def polar_to_cartesian(angles, distances):
    x_values = []
    y_values = []
    
    for angle, distance in zip(angles, distances):
        # 将角度转换为弧度，注意角度需要调整方向
        # 在极坐标中，通常0度指向右侧，90度指向上方
        angle_rad = math.radians(angle)
        
        # 计算笛卡尔坐标
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        
        x_values.append(x)
        y_values.append(y)
    
    return x_values, y_values

# 绘制雷达扫描图
def plot_radar_scan(angles, distances):
    # 转换为笛卡尔坐标
    x_values, y_values = polar_to_cartesian(angles, distances)
    
    # 创建图形并设置风格
    plt.style.use('seaborn-v0_8-whitegrid')  # 使用更现代的风格
    fig, ax = plt.subplots(figsize=(12, 9), dpi=100)
    
    # 设置背景色
    ax.set_facecolor('#f8f9fa')
    fig.patch.set_facecolor('#f8f9fa')
    
    # 计算理论上的测量点（与墙的交点）
    theoretical_angles = np.array(angles)
    theoretical_distances = []
    wall_height = 100  # 墙的高度为100cm
    
    for angle in theoretical_angles:
        # 计算理论距离（从原点到墙的距离）
        # 墙在y=100cm处，所以理论距离 = 100 / sin(angle)
        angle_rad = math.radians(angle)
        if abs(math.sin(angle_rad)) > 1e-6:  # 避免除以零
            distance = wall_height / math.sin(angle_rad)
            # 只有当角度在45-135度之间时，才有可能与墙相交
            if 45 <= angle <= 135 and distance > 0:
                theoretical_distances.append(distance)
            else:
                theoretical_distances.append(float('nan'))  # 不相交的情况
        else:
            theoretical_distances.append(float('nan'))  # 平行于墙的情况
    
    # 转换理论点为笛卡尔坐标
    theoretical_x = []
    theoretical_y = []
    
    for angle, distance in zip(theoretical_angles, theoretical_distances):
        if not math.isnan(distance):
            angle_rad = math.radians(angle)
            x = distance * math.cos(angle_rad)
            y = distance * math.sin(angle_rad)
            theoretical_x.append(x)
            theoretical_y.append(y)
    
    # 绘制墙壁（在y=100cm处的水平线）
    wall_x = np.linspace(-150, 150, 100)
    wall_y = np.full_like(wall_x, 100)
    ax.plot(wall_x, wall_y, 'k-', linewidth=2.5, label='Wall')
    
    # 绘制理论测量点（蓝色）
    ax.scatter(theoretical_x, theoretical_y, c='blue', s=30, alpha=0.6, 
               edgecolor='navy', marker='o', label='Theoretical Points')
    
    # 绘制实际测量点（红色）
    scatter = ax.scatter(x_values, y_values, c='red', s=30, alpha=0.8, 
                         edgecolor='darkred', label='Measured Points')
    
    # 绘制原点（传感器位置）
    ax.scatter(0, 0, c='blue', s=150, marker='*', edgecolor='navy', 
               linewidth=1.5, label='Sensor Position')
    
    # 找到最左侧和最右侧的点
    leftmost_idx = x_values.index(min(x_values))
    rightmost_idx = x_values.index(max(x_values))
    
    # 绘制从原点到最左侧点的线
    ax.plot([0, x_values[leftmost_idx]], [0, y_values[leftmost_idx]], 
            color='green', linewidth=2, linestyle='-')
    
    # 绘制从原点到最右侧点的线
    ax.plot([0, x_values[rightmost_idx]], [0, y_values[rightmost_idx]], 
            color='green', linewidth=2, linestyle='-')
    
    # 计算并标注角度
    left_angle = angles[leftmost_idx]
    right_angle = angles[rightmost_idx]
    
    # 标注左侧角度
    ax.annotate(f'{left_angle:.1f}°', 
                xy=(x_values[leftmost_idx]/2, y_values[leftmost_idx]/2),
                xytext=(x_values[leftmost_idx]/2-15, y_values[leftmost_idx]/2-5),
                fontsize=11, fontweight='bold',
                bbox=dict(boxstyle="round,pad=0.4", fc="yellow", ec="orange", alpha=0.8))
    
    # 标注右侧角度
    ax.annotate(f'{right_angle:.1f}°', 
                xy=(x_values[rightmost_idx]/2, y_values[rightmost_idx]/2),
                xytext=(x_values[rightmost_idx]/2+5, y_values[rightmost_idx]/2-5),
                fontsize=11, fontweight='bold',
                bbox=dict(boxstyle="round,pad=0.4", fc="yellow", ec="orange", alpha=0.8))
    
    # 添加网格和坐标轴
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax.axvline(x=0, color='k', linestyle='-', alpha=0.3)
    
    # 设置坐标轴范围和标签
    ax.set_xlim(-150, 150)
    ax.set_ylim(-10, 120)  # 确保y轴范围为-10到120cm
    
    # 设置标签和标题
    ax.set_xlabel('X Position (cm)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y Position (cm)', fontsize=12, fontweight='bold')
    ax.set_title('Ultrasonic Radar Scan Visualization', fontsize=16, fontweight='bold', pad=20)
    
    # 添加图例并美化
    legend = ax.legend(loc='upper right', frameon=True, framealpha=0.95, 
                      shadow=True, fontsize=10)
    legend.get_frame().set_facecolor('#f8f9fa')
    legend.get_frame().set_edgecolor('gray')
    
    # 添加扫描范围的扇形背景
    theta = np.linspace(math.radians(min(angles)), math.radians(max(angles)), 100)
    r = max(distances) * 1.05  # 稍微大一点的半径
    x_arc = r * np.cos(theta)
    y_arc = r * np.sin(theta)
    ax.fill_between([0] + list(x_arc) + [0], [0] + list(y_arc) + [0], 
                   color='lightblue', alpha=0.1, zorder=0)
    
    # 计算误差但不显示在图上
    if theoretical_x and theoretical_y:
        # 计算平均误差
        error_distances = []
        for tx, ty, mx, my in zip(theoretical_x, theoretical_y, x_values, y_values):
            error = math.sqrt((tx - mx)**2 + (ty - my)**2)
            error_distances.append(error)
        
        avg_error = sum(error_distances) / len(error_distances)
        max_error = max(error_distances)
        
        # 打印误差信息到控制台，但不显示在图上
        print(f'Average Error: {avg_error:.2f} cm | Max Error: {max_error:.2f} cm')
    
    # 保持纵横比一致
    ax.set_aspect('equal')
    
    # 显示图形
    plt.tight_layout()
    plt.savefig('radar_scan.png', dpi=300, bbox_inches='tight')  # 保存高质量图像
    plt.show()

# 主函数
def main():
    file_path = r"d:\OneDrive - qq.com\OneDrive\学习\SIT\结项\rador-test.CSV"
    angles, distances = read_radar_data(file_path)
    
    if angles and distances:
        print(f"成功读取 {len(angles)} 个数据点")
        plot_radar_scan(angles, distances)
    else:
        print("未能读取有效数据")

if __name__ == "__main__":
    main()
