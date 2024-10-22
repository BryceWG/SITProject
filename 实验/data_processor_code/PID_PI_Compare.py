import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
from typing import List, Tuple
import os

# 设置字体为 SimHei 以支持中文显示
rcParams['font.sans-serif'] = ['SimHei']  # 用黑体显示中文
rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# 1. 数据导入与预处理
def import_data(filename: str) -> pd.DataFrame:
    """
    从CSV文件中导入实验数据，处理非数值数据，统一列名，并移除多余的未命名列。

    参数:
    filename (str): 数据文件的路径和文件名。

    返回:
    pd.DataFrame: 包含处理后的实验数据的DataFrame。
    """
    # 读取CSV文件，将所有列先作为字符串类型读入
    try:
        data = pd.read_csv(filename, dtype=str)
        print(f"成功读取文件: {filename}")
    except Exception as e:
        print(f"读取文件失败: {filename}\n错误信息: {e}")
        raise

    # 统一控制类型标识（去除空格并转为大写）
    if 'ControlType' in data.columns:
        data['ControlType'] = data['ControlType'].str.strip().str.upper()
    else:
        print("警告: 未找到 'ControlType' 列。")

    # 定义需要转换为数值的列（更新为实际数据的列名）
    numeric_columns = ['Timestamp_ms', 'k_p', 'k_i', 'k_d', 'TargetSpeed_mm_s', 'ActualSpeed_mm_s', 'Error_mm_s']

    # 创建一个布尔掩码来标识有效的数值行
    valid_rows = pd.Series(True, index=data.index)

    for col in numeric_columns:
        # 检查列是否存在
        if col in data.columns:
            # 尝试将列转换为数值，无法转换的设为NaN
            data[col] = pd.to_numeric(data[col], errors='coerce')
            # 更新有效行掩码（允许 'k_d' 为 NaN，适用于 PI 控制）
            if col == 'k_d':
                valid_rows &= data[col].notna() | (data['ControlType'] == 'PI')
            else:
                valid_rows &= data[col].notna()
        else:
            print(f"警告: 未找到 '{col}' 列。")

    # 应用掩码，只保留所有必需数值列都有效的行
    initial_length = len(data)
    data = data[valid_rows]
    print(f"保留前 {initial_length} 行数据，通过过滤后剩余 {len(data)} 行。")

    # 重置索引
    data = data.reset_index(drop=True)

    # 添加时间列（以秒为单位）
    if 'Timestamp_ms' in data.columns:
        data['Time'] = data['Timestamp_ms'] / 1000
        print("成功添加 'Time' 列。")
    else:
        print("警告: 未找到 'Timestamp_ms' 列，无法创建 'Time' 列。")

    # 检查关键列是否存在
    required_columns = ['ControlType', 'Time', 'k_p', 'k_i', 'k_d', 'TargetSpeed_mm_s', 'ActualSpeed_mm_s', 'Error_mm_s', 'Phase']
    missing_columns = [col for col in required_columns if col not in data.columns]
    if missing_columns:
        print(f"警告: 缺失必要的列: {missing_columns}")
    else:
        print("所有必要的列均存在。")

    # 将 'Phase' 列中的内容转为大写，去除空格
    if 'Phase' in data.columns:
        data['Phase'] = data['Phase'].str.strip().str.upper()

    # 展示数据的前几行
    print("\n数据预览:")
    print(data.head())

    return data

# 2. 数据分割
def split_data_by_control_type(data: pd.DataFrame) -> dict:
    """
    根据控制类型分割数据。

    参数:
    data (pd.DataFrame): 包含实验数据的DataFrame。

    返回:
    dict: 以控制类型为键，对应的DataFrame为值的字典。
    """
    split_data = {ct: data[data['ControlType'] == ct].reset_index(drop=True) for ct in data['ControlType'].unique()}
    # 打印每个控制类型的数据量
    print("\n数据按控制类型分割情况:")
    for ct, df in split_data.items():
        print(f"控制类型: {ct}, 数据量: {len(df)}")
    return split_data

# 2.5 添加参数ID
def add_param_id(data: pd.DataFrame) -> pd.DataFrame:
    """
    为每个参数组合添加一个唯一的参数ID，包括控制类型。
    """
    # 确保 'k_i' 和 'k_d' 列存在
    if 'k_i' not in data.columns:
        data['k_i'] = np.nan
    if 'k_d' not in data.columns:
        data['k_d'] = np.nan

    # 创建包含控制类型和参数组合的唯一标识
    data['Control_Type_and_Params'] = data['ControlType'] + '_' + data['k_p'].astype(str) + '_' + data['k_i'].astype(str) + '_' + data['k_d'].astype(str)
    params = data[['Control_Type_and_Params']].drop_duplicates().reset_index(drop=True)

    # 分配唯一的 ParamID
    params['ParamID'] = range(1, len(params) + 1)

    # 将 ParamID 合并回原始数据
    merged_data = pd.merge(data, params, on='Control_Type_and_Params', how='left')

    # 移除临时列
    merged_data = merged_data.drop(columns=['Control_Type_and_Params'])

    # 调试信息
    print(f"总的参数组合数: {len(params)}, ParamID 范围: 1 - {len(params)}")

    return merged_data

# 3. 数据分析
def calculate_overshoot(data: pd.DataFrame) -> Tuple[float, float]:
    """
    计算超调量（实际输出超过目标值的最大百分比）。

    参数:
    data (pd.DataFrame): 包含实验数据的DataFrame。

    返回:
    Tuple[float, float]: 超调量（mm/s）和超调百分比（%）。
    """
    if data.empty:
        return np.nan, np.nan

    target_speed = data['TargetSpeed_mm_s'].max()
    actual_speed = data['ActualSpeed_mm_s']
    overshoot = (actual_speed - data['TargetSpeed_mm_s']).max()

    if target_speed != 0:
        overshoot_percentage = (overshoot / target_speed) * 100
    else:
        # 处理目标速度为0的情况
        overshoot_percentage = np.nan

    # 确保超调量和百分比不为负
    overshoot = max(overshoot, 0)
    overshoot_percentage = max(overshoot_percentage, 0)

    return overshoot, overshoot_percentage

def calculate_steady_state_error(data: pd.DataFrame) -> float:
    """
    计算稳态误差（系统稳态时，实际值与目标值之差）。

    参数:
    data (pd.DataFrame): 包含实验数据的DataFrame。

    返回:
    float: 稳态误差（mm/s）。
    """
    if data.empty:
        return np.nan

    steady_data = data.iloc[len(data)//2:]  # 取后半部分数据计算平均稳态误差
    steady_state_error = np.mean(np.abs(steady_data['TargetSpeed_mm_s'] - steady_data['ActualSpeed_mm_s']))
    return steady_state_error

def calculate_response_time(data: pd.DataFrame, tolerance: float = 0.05) -> float:
    """
    计算响应时间，即系统输出达到并保持在目标值一定误差范围内的时间。

    参数:
    data (pd.DataFrame): 包含实验数据的DataFrame。
    tolerance (float): 误差容限，占目标值的百分比。

    返回:
    float: 响应时间（秒）。
    """
    if data.empty:
        return np.nan

    target_speed = data['TargetSpeed_mm_s'].iloc[0]  # 假设目标速度在该阶段内保持不变
    if target_speed == 0:
        # 对于目标速度为0的情况，响应时间定义为实际速度降到一定阈值以下的时间
        threshold = 0.05  # 5 mm/s 的阈值
        within_tolerance = data['ActualSpeed_mm_s'].abs() <= threshold
    else:
        threshold = tolerance * abs(target_speed)
        within_tolerance = abs(data['ActualSpeed_mm_s'] - target_speed) <= threshold

    # 找到首次达到误差容限的位置
    indices = np.where(within_tolerance)[0]
    if len(indices) == 0:
        return np.nan  # 未达到稳态
    else:
        first_index = indices[0]
        response_time = data['Time'].iloc[first_index] - data['Time'].iloc[0]
        # 检查在此之后是否一直保持在误差容限内
        remained_within_tolerance = within_tolerance.iloc[first_index:].all()
        if remained_within_tolerance:
            return response_time
        else:
            # 如果在之后又超出了误差范围，可以考虑修改逻辑，例如计算95%时间
            return response_time  # 或者返回np.nan
    return np.nan

def analyze_performance(data: pd.DataFrame, param_id: int, control_type: str) -> List[dict]:
    """
    分析特定参数组合的所有运动阶段的性能指标。
    """
    param_data = data[data['ParamID'] == param_id]
    performance_results = []

    # 提取控制参数
    param_row = param_data.iloc[0]
    k_p = param_row['k_p']
    k_i = param_row['k_i'] if 'k_i' in param_row and not pd.isna(param_row['k_i']) else np.nan
    k_d = param_row['k_d'] if 'k_d' in param_row else np.nan

    for phase in param_data['Phase'].unique():
        phase_data = param_data[param_data['Phase'] == phase]

        # 在 SUDDEN_STOP 阶段，将超调量设置为 0
        if phase == 'SUDDEN_STOP':
            overshoot = 0
            overshoot_percentage = 0
        else:
            overshoot, overshoot_percentage = calculate_overshoot(phase_data)

        steady_state_error = calculate_steady_state_error(phase_data)

        # 在除了 SUDDEN_ACCELERATE 和 SUDDEN_STOP 之外的阶段，将响应时间设置为 0
        if phase in ['SUDDEN_ACCELERATE', 'SUDDEN_STOP']:
            response_time = calculate_response_time(phase_data)
            if pd.isna(response_time):
                response_time = phase_data['Time'].max() * 1.5
        else:
            response_time = 0

        # 将 NaN 的超调量视为 0
        overshoot_percentage = 0 if pd.isna(overshoot_percentage) else overshoot_percentage
        overshoot = 0 if pd.isna(overshoot) else overshoot

        # 将 NaN 的稳态误差视为最大误差的 1.5 倍
        if pd.isna(steady_state_error):
            steady_state_error = phase_data['Error_mm_s'].abs().max() * 1.5

        performance = {
            'ControlType': control_type,
            'ParamID': param_id,
            'Phase': phase,
            'k_p': k_p,
            'k_i': k_i,
            'k_d': k_d,
            'Overshoot': overshoot,
            'OvershootPercentage': overshoot_percentage,
            'SteadyStateError': steady_state_error,
            'ResponseTime': response_time
        }

        performance_results.append(performance)

        print(f"分析性能 - 控制类型: {control_type}, ParamID: {param_id}, 阶段: {phase}, "
              f"超调量%: {overshoot_percentage:.4f}, 稳态误差: {steady_state_error:.4f}, 响应时间: {response_time:.4f}")

    return performance_results

def calculate_performance_score(performance_df: pd.DataFrame) -> pd.DataFrame:
    """
    计算性能得分并添加到 performance_df 中。
    """
    df = performance_df.copy()
    # 将 NaN 值替换为合适的默认值，表示性能较差
    max_overshoot = df['OvershootPercentage'].max(skipna=True)
    max_error = df['SteadyStateError'].max(skipna=True)
    max_response_time = df['ResponseTime'].max(skipna=True)

    # 使用赋值的方式替换 NaN 值，避免警告
    df['OvershootPercentage'] = df['OvershootPercentage'].fillna(max_overshoot * 1.5)
    df['SteadyStateError'] = df['SteadyStateError'].fillna(max_error * 1.5)
    df['ResponseTime'] = df['ResponseTime'].fillna(max_response_time * 1.5)

    # 定义性能指标的权重或重要性，可以根据需要调整
    df['PerformanceScore'] = df['OvershootPercentage'] + df['SteadyStateError'] + df['ResponseTime']

    return df

# 4. 综合排名
def comprehensive_ranking(performance_df: pd.DataFrame, phase_weights: dict) -> pd.DataFrame:
    """
    计算综合排名，确保同时处理 PI 和 PID 控制器数据。
    
    参数:
    performance_df (pd.DataFrame): 包含性能指标的DataFrame
    phase_weights (dict): 不同阶段的权重
    
    返回:
    pd.DataFrame: 包含综合排名的DataFrame
    """
    df = performance_df.copy()
    
    # 将权重标准化
    total_weight = sum(phase_weights.values())
    normalized_weights = {phase: weight / total_weight for phase, weight in phase_weights.items()}
    
    # 添加权重列
    df['PhaseWeight'] = df['Phase'].apply(lambda x: normalized_weights.get(x, 0))
    
    # 计算加权性能得分
    df['WeightedPerformanceScore'] = df['PerformanceScore'] * df['PhaseWeight']
    
    # 分组计算时明确指定要使用的列，避免 NaN 值影响
    group_columns = ['ParamID', 'ControlType', 'k_p', 'k_i']
    # 对于 PID 控制器，额外包含 k_d 列
    total_scores = []
    
    # 分别处理 PI 和 PID 控制器
    for control_type in df['ControlType'].unique():
        df_control = df[df['ControlType'] == control_type]
        
        if control_type == 'PID':
            group_cols = group_columns + ['k_d']
        else:  # PI 控制器
            group_cols = group_columns
            
        # 计算每组的综合得分
        scores = df_control.groupby(group_cols).agg({
            'WeightedPerformanceScore': 'sum'
        }).reset_index()
        
        # 如果是 PI 控制器，添加 k_d 列为 NaN
        if control_type == 'PI':
            scores['k_d'] = np.nan
            
        total_scores.append(scores)
    
    # 合并所有结果
    total_scores_df = pd.concat(total_scores, ignore_index=True)
    
    # 确保 WeightedPerformanceScore 大于 0
    total_scores_df = total_scores_df[total_scores_df['WeightedPerformanceScore'] > 0]
    
    # 按综合得分排序
    total_scores_df = total_scores_df.sort_values('WeightedPerformanceScore').reset_index(drop=True)
    
    # 添加排名列
    total_scores_df['Rank'] = total_scores_df.index + 1
    
    return total_scores_df


# 5. 绘图
def plot_performance_comparison(performance_df: pd.DataFrame, save_dir: str, phase: str = 'SUDDEN_ACCELERATE'):
    """
    绘制指定阶段的不同控制方法和参数组合的性能指标比较图，保存到本地。

    参数:
    performance_df (pd.DataFrame): 包含性能指标的DataFrame。
    save_dir (str): 保存图像的目录。
    phase (str): 要过滤的运动阶段，默认值为 'SUDDEN_ACCELERATE'。
    """
    os.makedirs(save_dir, exist_ok=True)

    # 过滤指定阶段的数据
    df_top = performance_df[performance_df['Phase'] == phase].copy()

    if df_top.empty:
        print(f"在阶段 {phase} 中没有数据，无法绘制性能比较图。")
        return

    # 定义颜色和标记
    markers = {'PID': 'o', 'PI': 's'}
    colors = {'PID': 'blue', 'PI': 'green'}

    # 绘制超调量与稳态误差的关系图
    plt.figure(figsize=(12, 8))
    for ctl_type in df_top['ControlType'].unique():
        df = df_top[df_top['ControlType'] == ctl_type]
        if not df.empty:
            plt.scatter(df['OvershootPercentage'], df['SteadyStateError'],
                        marker=markers.get(ctl_type, 'x'),
                        color=colors.get(ctl_type, 'black'),
                        label=f'{ctl_type}',
                        alpha=0.6)
    plt.xlabel('超调量 (%)')
    plt.ylabel('稳态误差 (mm/s)')
    plt.title(f'超调量与稳态误差关系图 - 阶段：{phase}')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, f'overshoot_vs_steady_state_error_{phase}.png'))
    plt.close()

def plot_full_speed_curve(data: pd.DataFrame, param_id: int, control_type: str, rank: int, save_dir: str):
    """
    绘制特定参数组合的完整速度-时间曲线（包含所有阶段）。

    参数:
    data (pd.DataFrame): 包含实验数据的DataFrame。
    param_id (int): 参数组合ID。
    control_type (str): 控制类型。
    rank (int): 综合排名。
    save_dir (str): 保存图像的目录。
    """
    param_data = data[data['ParamID'] == param_id]
    if param_data.empty:
        print(f"未找到参数ID {param_id} 的数据。")
        return

    os.makedirs(save_dir, exist_ok=True)

    plt.figure(figsize=(12, 8))
    plt.plot(param_data['Time'], param_data['TargetSpeed_mm_s'], 'r-', linewidth=2, label='目标速度')
    plt.plot(param_data['Time'], param_data['ActualSpeed_mm_s'], 'b-', linewidth=2, label='实际速度')
    plt.xlabel('时间 (秒)')
    plt.ylabel('速度 (mm/s)')
    plt.title(f"控制类型: {control_type}, 参数ID: {param_id}, 综合排名: {rank}")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    filename = f"full_speed_curve_{control_type}_ParamID_{param_id}_Rank_{rank}.png"
    plt.savefig(os.path.join(save_dir, filename))
    plt.close()

def plot_parameter_histograms(total_scores_df: pd.DataFrame, save_dir: str):
    """
    绘制参数值出现次数的直方图，针对 k_p、k_i、k_d，每个参数一张图。
    """
    os.makedirs(save_dir, exist_ok=True)

    # 取综合排名前 20% 的数据
    top_20_percent = int(len(total_scores_df) * 0.2)
    df_top = total_scores_df.head(top_20_percent)

    parameters = ['k_p', 'k_i', 'k_d']
    control_types = ['PID', 'PI']
    colors = {'PID': 'blue', 'PI': 'green'}

    for param in parameters:
        plt.figure(figsize=(12, 8))
        for ctl_type in control_types:
            df_ctl = df_top[df_top['ControlType'] == ctl_type]
            values = df_ctl[param].dropna()
            if not values.empty:
                min_val, max_val = values.min(), values.max()
                bins = np.linspace(min_val, max_val, 20)  # 使用20个等宽的bin
                plt.hist(values, bins=bins, alpha=0.5, label=ctl_type, color=colors[ctl_type])
        plt.xlabel(f'{param} 值')
        plt.ylabel('出现次数')
        plt.title(f'{param} 参数值出现次数分布（前20%综合排名）')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        filename = f'parameter_histogram_{param}.png'
        plt.savefig(os.path.join(save_dir, filename))
        plt.close()

# 生成总结性报告
def generate_summary_report(total_scores_df: pd.DataFrame, phase_weights: dict, filename: str):
    """
    生成总结性报告，将数据处理结果和简单分析输出到文件。
    """
    with open(filename, 'w', encoding='utf-8') as f:
        f.write("控制参数综合性能排名报告\n")
        f.write("========================\n\n")
        
        f.write("各阶段权重:\n")
        for phase, weight in phase_weights.items():
            f.write(f"- {phase}: {weight}%\n")
        f.write("\n")
        
        f.write("综合排名（前 20 名）:\n")
        f.write(total_scores_df.head(20).to_string(index=False))
        f.write("\n\n")
        
        f.write("参数出现次数统计（前 20% 综合排名）:\n")
        # 统计各参数值的出现次数
        top_20_percent = int(len(total_scores_df) * 0.2)
        df_top = total_scores_df.head(top_20_percent)
        
        parameters = ['k_p', 'k_i', 'k_d']
        for param in parameters:
            f.write(f"\n参数 {param} 的出现次数:\n")
            value_counts = df_top[param].value_counts().sort_index()
            f.write(value_counts.to_string())
            f.write("\n")
        
        f.write("\n报告结束。\n")

# 6. 主程序
def main():
    # ---- 您可以在此处调整自定义参数值 ----
    filename = 'code_2_data_3.csv'  # 请替换为您的数据文件名
    save_directory = 'plots'  # 保存图像的目录
    output_filename = 'performance_results.csv'  # 输出的性能指标文件名
    # ----------------------------------------

    # 定义要分析的运动阶段
    motion_phases_to_analyze = ['ACCELERATE', 'DECELERATE', 'STATIONARY', 'SUDDEN_ACCELERATE', 'SUDDEN_STOP']

    # 导入数据
    data = import_data(filename)
    print("数据导入与预处理完成。")

    # 添加参数ID
    data = add_param_id(data)

    # 数据分割
    data_by_control_type = split_data_by_control_type(data)

    # 数据分析
    performance_results = []
    for control_type, ctl_data in data_by_control_type.items():
        # 数据分析
        unique_param_ids = ctl_data['ParamID'].unique()
        for param_id in unique_param_ids:
            results = analyze_performance(ctl_data, param_id, control_type)
            performance_results.extend(results)

    # 汇总性能指标
    performance_df = pd.DataFrame(performance_results)
    # 计算性能得分并添加到 performance_df 中
    performance_df = calculate_performance_score(performance_df)

    # 检查不同控制类型的数据量
    print("\n性能数据中各控制类型的数据量:")
    print(performance_df['ControlType'].value_counts())

    # 将性能指标导出为CSV文件
    performance_df.to_csv(output_filename, index=False)
    print(f"性能指标已导出到文件: {output_filename}")

    # 绘制性能比较图（仅使用 SUDDEN_ACCELERATE 阶段的数据）
    plot_performance_comparison(performance_df, save_dir=save_directory, phase='SUDDEN_ACCELERATE')
    print("性能比较图绘制完成。")

    # 综合排名
    phase_weights = {
        'ACCELERATE': 25,
        'DECELERATE': 20,
        'STATIONARY': 5,
        'SUDDEN_ACCELERATE': 40,
        'SUDDEN_STOP': 10
    }
    total_scores_df = comprehensive_ranking(performance_df, phase_weights)

    # 检查不同控制类型的数据量
    print("\n综合排名中各控制类型的数据量:")
    print(total_scores_df['ControlType'].value_counts())

    # 将综合排名结果保存到文件
    total_scores_df.to_csv('comprehensive_ranking.csv', index=False)

    # 绘制总分与排名的关系图
    plt.figure(figsize=(12, 8))
    # 按照不同的控制类型绘制不同的曲线
    for control_type in total_scores_df['ControlType'].unique():
        df = total_scores_df[total_scores_df['ControlType'] == control_type]
        plt.plot(df['Rank'], df['WeightedPerformanceScore'], marker='o', label=f'{control_type}')
    plt.xlabel('排名')
    plt.ylabel('综合性能得分')
    plt.title('综合性能得分与排名关系图')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(save_directory, 'total_score_vs_rank.png'))
    plt.close()

    # 绘制热门参数排行图
    plot_parameter_histograms(total_scores_df, save_dir=save_directory)
    print("热门参数排行图绘制完成。")

    # 生成总结性报告
    summary_filename = 'summary_report.txt'
    generate_summary_report(total_scores_df, phase_weights, summary_filename)
    print(f"总结性报告已生成：{summary_filename}")

    # 绘制排名前三的完整速度-时间曲线
    top_3_params = total_scores_df.head(3)
    for idx, row in top_3_params.iterrows():
        param_id = row['ParamID']
        control_type = row['ControlType']
        rank = int(row['Rank'])
        # 获取对应控制类型的数据
        ctl_data = data_by_control_type[control_type]
        plot_full_speed_curve(ctl_data, param_id, control_type, rank, save_dir=save_directory)
    print("前三名参数组合的完整速度-时间曲线已绘制完成。")

if __name__ == "__main__":
    main()
