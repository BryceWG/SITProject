import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
from typing import List, Tuple
import os

# 设置字体为 SimHei 以支持中文显示
rcParams['font.sans-serif'] = ['SimHei'] 
rcParams['axes.unicode_minus'] = False

# 1. 数据导入与预处理
def import_data(filename: str) -> pd.DataFrame:
    """
    从CSV文件中导入实验数据，处理非数值数据，统一列名，并移除多余的未命名列。

    参数:
    filename (str): 数据文件的路径和文件名。

    返回:
    pd.DataFrame: 包含处理后的实验数据的DataFrame。
    """
    try:
        data = pd.read_csv(filename, dtype=str)
        print(f"成功读取文件: {filename}")
    except Exception as e:
        print(f"读取文件失败: {filename}\n错误信息: {e}")
        raise

    # 统一控制类型标识
    if 'ControlType' in data.columns:
        data['ControlType'] = data['ControlType'].str.strip().str.upper()
    else:
        print("警告: 未找到 'ControlType' 列。")

    numeric_columns = ['Timestamp_ms', 'k_p', 'k_i', 'k_d', 'TargetSpeed_mm_s', 'ActualSpeed_mm_s', 'Error_mm_s']
    valid_rows = pd.Series(True, index=data.index)

    for col in numeric_columns:
        if col in data.columns:
            data[col] = pd.to_numeric(data[col], errors='coerce')
            if col == 'k_d':
                valid_rows &= data[col].notna() | (data['ControlType'] == 'PI')
            else:
                valid_rows &= data[col].notna()
        else:
            print(f"警告: 未找到 '{col}' 列。")

    initial_length = len(data)
    data = data[valid_rows]
    print(f"保留前 {initial_length} 行数据，通过过滤后剩余 {len(data)} 行。")

    data = data.reset_index(drop=True)

    if 'Timestamp_ms' in data.columns:
        data['Time'] = data['Timestamp_ms'] / 1000
        print("成功添加 'Time' 列。")
    else:
        print("警告: 未找到 'Timestamp_ms' 列，无法创建 'Time' 列。")

    required_columns = ['ControlType', 'Time', 'k_p', 'k_i', 'k_d', 'TargetSpeed_mm_s', 'ActualSpeed_mm_s', 'Error_mm_s', 'Phase']
    missing_columns = [col for col in required_columns if col not in data.columns]
    if missing_columns:
        print(f"警告: 缺失必要的列: {missing_columns}")
    else:
        print("所有必要的列均存在。")

    if 'Phase' in data.columns:
        data['Phase'] = data['Phase'].str.strip().str.upper()

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
    print("\n数据按控制类型分割情况:")
    for ct, df in split_data.items():
        print(f"控制类型: {ct}, 数据量: {len(df)}")
    return split_data

def add_param_id(data: pd.DataFrame) -> pd.DataFrame:
    """
    为每个参数组合添加一个唯一的参数ID，包括控制类型。
    """
    if 'k_i' not in data.columns:
        data['k_i'] = np.nan
    if 'k_d' not in data.columns:
        data['k_d'] = np.nan

    # 创建包含控制类型和参数组合的唯一标识
    data['Control_Type_and_Params'] = data['ControlType'] + '_' + data['k_p'].astype(str) + '_' + data['k_i'].astype(str) + '_' + data['k_d'].astype(str)
    params = data[['Control_Type_and_Params']].drop_duplicates().reset_index(drop=True)
    params['ParamID'] = range(1, len(params) + 1)
    
    merged_data = pd.merge(data, params, on='Control_Type_and_Params', how='left')
    merged_data = merged_data.drop(columns=['Control_Type_and_Params'])
    
    print(f"总的参数组合数: {len(params)}, ParamID 范围: 1 - {len(params)}")
    return merged_data

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

    # 取后半部分数据计算平均稳态误差
    steady_data = data.iloc[len(data)//2:]
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
            return response_time
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

        # 处理异常值
        overshoot_percentage = 0 if pd.isna(overshoot_percentage) else overshoot_percentage
        overshoot = 0 if pd.isna(overshoot) else overshoot
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

class PerformanceConfig:
    """性能评价参数配置类"""
    def __init__(self):
        # 阶段权重配置
        self.phase_weights = {
            'ACCELERATE': 20,
            'DECELERATE': 20,
            'STATIONARY': 5, 
            'SUDDEN_ACCELERATE': 45,
            'SUDDEN_STOP': 10
        }
        
        # 性能指标权重
        self.metric_weights = {
            'OvershootPercentage': 0.4,
            'SteadyStateError': 0.4, 
            'ResponseTime': 0.2
        }
        
        # 性能指标阈值
        self.thresholds = {
            'max_overshoot': 40,      # 最大允许超调量(%)
            'max_steady_error': 15,    # 最大允许稳态误差(mm/s)
            'max_response_time': 1.0   # 最大允许响应时间(s)
        }
        
        # 分析配置
        self.analysis_config = {
            'top_n': 5,               # 展示前N个最优结果
            'error_tolerance': 0.05,   # 误差容限
            'steady_state_window': 0.3 # 稳态判定窗口(占总时长比例)
        }

def calculate_performance_score(performance_df: pd.DataFrame, config: PerformanceConfig) -> pd.DataFrame:
    """增强的性能评分计算"""
    df = performance_df.copy()
    
    # 归一化处理
    for metric in ['OvershootPercentage', 'SteadyStateError', 'ResponseTime']:
        max_val = df[metric].max()
        min_val = df[metric].min()
        if max_val != min_val:
            df[f'{metric}_Normalized'] = (df[metric] - min_val) / (max_val - min_val)
        else:
            df[f'{metric}_Normalized'] = 0
            
    # 计算加权得分
    df['WeightedScore'] = (
        config.metric_weights['OvershootPercentage'] * df['OvershootPercentage_Normalized'] +
        config.metric_weights['SteadyStateError'] * df['SteadyStateError_Normalized'] +
        config.metric_weights['ResponseTime'] * df['ResponseTime_Normalized']
    )
    
    # 添加阈值惩罚
    penalty = 0
    if df['OvershootPercentage'].max() > config.thresholds['max_overshoot']:
        penalty += 0.2
    if df['SteadyStateError'].max() > config.thresholds['max_steady_error']:
        penalty += 0.2
    if df['ResponseTime'].max() > config.thresholds['max_response_time']:
        penalty += 0.2
    
    df['FinalScore'] = df['WeightedScore'] * (1 + penalty)
    
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
    
    df['PhaseWeight'] = df['Phase'].apply(lambda x: normalized_weights.get(x, 0))
    df['WeightedPerformanceScore'] = df['FinalScore'] * df['PhaseWeight']
    
    group_columns = ['ParamID', 'ControlType', 'k_p', 'k_i']
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
        
        if control_type == 'PI':
            scores['k_d'] = np.nan
            
        total_scores.append(scores)
    
    total_scores_df = pd.concat(total_scores, ignore_index=True)
    total_scores_df = total_scores_df[total_scores_df['WeightedPerformanceScore'] > 0]
    total_scores_df = total_scores_df.sort_values('WeightedPerformanceScore').reset_index(drop=True)
    total_scores_df['Rank'] = total_scores_df.index + 1
    
    return total_scores_df

def plot_with_zoom_inset(total_scores_df: pd.DataFrame, save_dir: str):
    """
    绘制带有局部放大图的总分-排名关系图
    """
    plt.figure(figsize=(12, 8))
    ax = plt.axes([0.1, 0.1, 0.7, 0.8])
    
    # 计算前10%的数据范围
    top_10_percent = int(len(total_scores_df) * 0.1)
    df_plot = total_scores_df.head(top_10_percent)
    
    # 主图绘制
    for control_type in df_plot['ControlType'].unique():
        df = df_plot[df_plot['ControlType'] == control_type]
        ax.plot(df['Rank'], df['WeightedPerformanceScore'], 
                marker='o', label=f'{control_type}')
    
    # 创建放大图
    axins = ax.inset_axes([0.05, 0.55, 0.4, 0.4])
    
    # 在放大图中绘制前20%的数据
    top_20_percent = int(len(df_plot) * 0.2)
    for control_type in df_plot['ControlType'].unique():
        df = df_plot[df_plot['ControlType'] == control_type]
        df_top = df.head(top_20_percent)
        axins.plot(df_top['Rank'], df_top['WeightedPerformanceScore'], 
                   marker='o', label=f'{control_type}')
    
    # 设置放大图的范围
    axins.set_xlim(1, top_20_percent)
    y_min = df_plot.head(top_20_percent)['WeightedPerformanceScore'].min()
    y_max = df_plot.head(top_20_percent)['WeightedPerformanceScore'].max()
    y_margin = (y_max - y_min) * 0.1
    axins.set_ylim(y_min - y_margin, y_max + y_margin)
    
    # 主图设置
    ax.set_xlabel('排名')
    ax.set_ylabel('综合性能得分')
    ax.set_title('综合性能得分与排名关系图 (前10%)')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.grid(True)
    
    axins.grid(True)
    
    # 添加连接线
    from mpl_toolkits.axes_grid1.inset_locator import mark_inset
    mark_inset(ax, axins, loc1=2, loc2=4, fc="none", ec="0.5")
    
    plt.savefig(os.path.join(save_dir, 'total_score_vs_rank.png'), 
                bbox_inches='tight',
                dpi=300)
    plt.close()

def plot_performance_comparison_with_zoom(performance_df: pd.DataFrame, save_dir: str, phase: str = 'SUDDEN_ACCELERATE'):
    """
    绘制带有局部放大图的性能指标比较图
    """
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # 过滤指定阶段的数据
    df_top = performance_df[performance_df['Phase'] == phase].copy()
    
    if df_top.empty:
        print(f"在阶段 {phase} 中没有数据，无法绘制性能比较图。")
        return
    
    # 定义颜色和标记
    markers = {'PID': 'o', 'PI': 's'}
    colors = {'PID': 'blue', 'PI': 'green'}
    
    # 主图绘制
    for ctl_type in df_top['ControlType'].unique():
        df = df_top[df_top['ControlType'] == ctl_type]
        if not df.empty:
            ax.scatter(df['OvershootPercentage'], df['SteadyStateError'],
                      marker=markers.get(ctl_type, 'x'),
                      color=colors.get(ctl_type, 'black'),
                      label=f'{ctl_type}',
                      alpha=0.6)
    
    # 创建放大图
    axins = ax.inset_axes([0.05, 0.55, 0.4, 0.4])
    
    # 计算前20%的数据点
    top_20_percent = int(len(df_top) * 0.2)
    df_zoom = df_top.nsmallest(top_20_percent, 'FinalScore')
    
    # 在放大图中绘制前20%的数据点
    for ctl_type in df_zoom['ControlType'].unique():
        df = df_zoom[df_zoom['ControlType'] == ctl_type]
        if not df.empty:
            axins.scatter(df['OvershootPercentage'], df['SteadyStateError'],
                         marker=markers.get(ctl_type, 'x'),
                         color=colors.get(ctl_type, 'black'),
                         label=f'{ctl_type}',
                         alpha=0.6)
    
    # 设置放大图的范围
    x_min = df_zoom['OvershootPercentage'].min()
    x_max = df_zoom['OvershootPercentage'].max()
    y_min = df_zoom['SteadyStateError'].min()
    y_max = df_zoom['SteadyStateError'].max()
    margin_x = (x_max - x_min) * 0.1
    margin_y = (y_max - y_min) * 0.1
    axins.set_xlim(x_min - margin_x, x_max + margin_x)
    axins.set_ylim(y_min - margin_y, y_max + margin_y)
    
    # 主图设置
    ax.set_xlabel('超调量 (%)')
    ax.set_ylabel('稳态误差 (mm/s)')
    ax.set_title(f'超调量与稳态误差关系图 - 阶段：{phase}')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.grid(True)
    
    axins.grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, f'overshoot_vs_steady_state_error_{phase}.png'),
                bbox_inches='tight',
                dpi=300)
    plt.close()

def calculate_parameter_importance(total_scores_df: pd.DataFrame, top_percent: float = 0.3) -> dict:
    """
    计算参数值的重要程度
    
    参数:
    total_scores_df: 包含排名信息的DataFrame
    top_percent: 选取前多少比例的数据进行分析
    
    返回:
    dict: 包含每个参数值的重要程度得分
    """
    # 选取前30%的数据
    top_n = int(len(total_scores_df) * top_percent)
    df_top = total_scores_df.head(top_n)
    
    importance_scores = {}
    parameters = ['k_p', 'k_i', 'k_d']
    
    for param in parameters:
        param_values = df_top[param].dropna().unique()
        
        for value in param_values:
            ranks = df_top[df_top[param] == value]['Rank']
            if len(ranks) > 0:
                avg_rank = ranks.sum() / len(ranks)
                importance_score = avg_rank / len(ranks)
                importance_scores[f'{param}_{value:.4f}'] = importance_score
    
    # 对结果进行排序（得分越低越重要）
    sorted_scores = dict(sorted(importance_scores.items(), key=lambda x: x[1]))
    
    return sorted_scores

def print_parameter_importance(importance_scores: dict, output_file: str):
    """
    将参数重要度分析结果写入文件
    """
    with open(output_file, 'a', encoding='utf-8') as f:
        f.write("\n\n参数重要度分析结果\n")
        f.write("================\n")
        f.write("注：得分越低表示该参数值对优化越有效\n\n")
        
        params = {'k_p': [], 'k_i': [], 'k_d': []}
        
        for key, score in importance_scores.items():
            param_type = key.split('_')[0] + '_' + key.split('_')[1]
            params[param_type].append((key, score))
        
        for param_type, scores in params.items():
            if scores:
                f.write(f"\n{param_type} 参数分析:\n")
                for key, score in sorted(scores, key=lambda x: x[1]):
                    value = float(key.split('_')[-1])
                    f.write(f"{param_type} = {value:.4f}: {score:.4f}\n")

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
    # 取综合排名前 20% 的数据
    top_20_percent = int(len(total_scores_df) * 0.2)
    df_top = total_scores_df.head(top_20_percent)

    parameters = ['k_p', 'k_i', 'k_d']
    control_types = ['PID', 'PI']
    colors = {'PID': '#7777FF', 'PI': '#77FF77'}

    for param in parameters:
        plt.figure(figsize=(12, 8))
        ax = plt.gca()
        
        # 获取所有参数值并排序
        all_values = sorted(df_top[param].dropna().unique())
        bar_width = 0.35
        
        for i, ctl_type in enumerate(control_types):
            df_ctl = df_top[df_top['ControlType'] == ctl_type]
            if not df_ctl.empty:
                # 计算每个参数值的出现次数
                value_counts = df_ctl[param].value_counts()
                x_positions = np.arange(len(all_values))
                heights = [value_counts.get(val, 0) for val in all_values]
                
                # 绘制条形图
                bars = ax.bar(x_positions + (i - 0.5) * bar_width, 
                            heights,
                            bar_width,
                            label=ctl_type,
                            color=colors[ctl_type],
                            alpha=0.7)
                
                # 在每个条形上添加数值标签（只标注非零值）
                for x, y in zip(x_positions + (i - 0.5) * bar_width, heights):
                    if y > 0:
                        ax.text(x, y, f'{int(y)}', ha='center', va='bottom')
        
        # 设置x轴刻度和标签
        plt.xticks(np.arange(len(all_values)), 
                  [f'{x:.3f}' for x in all_values],
                  rotation=45)
        
        plt.xlabel(f'{param} 值')
        plt.ylabel('出现次数')
        plt.title(f'{param} 参数值出现次数分布（前20%综合排名）')
        plt.legend()
        plt.grid(True, axis='y')
        plt.tight_layout()
        
        filename = f'parameter_histogram_{param}.png'
        plt.savefig(os.path.join(save_dir, filename), dpi=300, bbox_inches='tight')
        plt.close()

def generate_summary_report(total_scores_df: pd.DataFrame, phase_weights: dict, filename: str, importance_scores: dict = None):
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
        
        # 添加参数重要度分析结果
        if importance_scores:
            f.write("\n\n参数重要度分析结果\n")
            f.write("================\n")
            f.write("注：得分越低表示该参数值对优化越有效\n\n")
            
            # 按参数类型分组输出
            params = {'k_p': [], 'k_i': [], 'k_d': []}
            
            for key, score in importance_scores.items():
                param_type = key.split('_')[0] + '_' + key.split('_')[1]
                params[param_type].append((key, score))
            
            for param_type, scores in params.items():
                if scores:
                    f.write(f"\n{param_type} 参数分析:\n")
                    for key, score in sorted(scores, key=lambda x: x[1]):
                        value = float(key.split('_')[-1])
                        f.write(f"{param_type} = {value:.4f}: {score:.4f}\n")
        
        f.write("\n报告结束。\n")
        
def plot_radar_chart(performance_df: pd.DataFrame, total_scores_df: pd.DataFrame, save_dir: str):
    """
    绘制参数性能雷达图
    """
    metrics = ['OvershootPercentage', 'SteadyStateError', 'ResponseTime']
    top_params = total_scores_df.head(5)  # 获取前5个最优参数组合
    
    plt.figure(figsize=(12, 8))
    ax = plt.subplot(111, projection='polar')
    
    angles = np.linspace(0, 2*np.pi, len(metrics), endpoint=False)
    angles = np.concatenate((angles, [angles[0]]))  # 闭合图形
    
    # 为每个参数组合绘制雷达图
    for idx, param_row in top_params.iterrows():
        param_data = performance_df[
            (performance_df['ParamID'] == param_row['ParamID']) & 
            (performance_df['ControlType'] == param_row['ControlType'])
        ]
        
        # 计算平均性能指标
        avg_metrics = []
        for metric in metrics:
            avg_value = param_data[metric].mean()
            max_val = performance_df[metric].max()
            min_val = performance_df[metric].min()
            if max_val != min_val:
                normalized_value = (avg_value - min_val) / (max_val - min_val)
            else:
                normalized_value = 0
            avg_metrics.append(normalized_value)
        
        values = np.concatenate((avg_metrics, [avg_metrics[0]]))
        label = f"{param_row['ControlType']} (Rank {idx+1})"
        ax.plot(angles, values, 'o-', linewidth=2, label=label)
        ax.fill(angles, values, alpha=0.25)
    
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(metrics)
    plt.title('前5名参数组合性能对比')
    plt.legend(loc='upper right', bbox_to_anchor=(0.1, 0.1))
    
    plt.savefig(os.path.join(save_dir, 'radar_chart.png'))
    plt.close()

def plot_parameter_space(total_scores_df: pd.DataFrame, save_dir: str):
    """
    绘制参数空间分布图,展示不同参数组合的性能分布
    """
    for control_type in total_scores_df['ControlType'].unique():
        df_control = total_scores_df[total_scores_df['ControlType'] == control_type]
        
        if df_control.empty:
            print(f"警告: {control_type} 控制器没有有效的数据")
            continue
            
        try:
            if control_type == 'PID':
                # 3D图 for PID
                fig = plt.figure(figsize=(14, 10))
                ax = fig.add_subplot(111, projection='3d')
                
                scores = df_control['WeightedPerformanceScore']
                if len(scores) == 0 or scores.isna().all():
                    print(f"警告: {control_type} 控制器没有有效的性能得分")
                    plt.close()
                    continue
                
                # 归一化得分
                normalized_scores = (scores - scores.min()) / (scores.max() - scores.min()) \
                    if scores.max() != scores.min() else np.zeros_like(scores)
                
                scatter = ax.scatter(df_control['k_p'], 
                                   df_control['k_i'],
                                   df_control['k_d'],
                                   c=normalized_scores,
                                   cmap='viridis',
                                   s=60)
                
                cbar = plt.colorbar(scatter)
                cbar.set_label('归一化性能得分', fontsize=14)
                cbar.ax.tick_params(labelsize=12)
                
                ax.set_xlabel('Kp', fontsize=12, labelpad=10)
                ax.set_ylabel('Ki', fontsize=12, labelpad=10)
                ax.set_zlabel('Kd', fontsize=12, labelpad=5)
                
                # 设置刻度值为实际的参数值
                unique_kd = sorted(df_control['k_d'].unique())
                ax.set_zticks(unique_kd)
                ax.set_zticklabels([f'{x:.3f}' for x in unique_kd])
                
                ax.view_init(elev=20, azim=45)
                ax.zaxis._axinfo['label']['space_factor'] = 2.0
                ax.tick_params(axis='both', which='major', labelsize=12)
                
            else:  # PI控制器
                # 2D图 for PI
                fig = plt.figure(figsize=(10, 8))
                ax = fig.add_subplot(111)
                
                scores = df_control['WeightedPerformanceScore']
                if len(scores) == 0 or scores.isna().all():
                    print(f"警告: {control_type} 控制器没有有效的性能得分")
                    plt.close()
                    continue
                
                # 归一化得分
                normalized_scores = (scores - scores.min()) / (scores.max() - scores.min()) \
                    if scores.max() != scores.min() else np.zeros_like(scores)
                
                scatter = ax.scatter(df_control['k_p'],
                                   df_control['k_i'],
                                   c=normalized_scores,
                                   cmap='viridis')
                
                plt.colorbar(scatter, label='归一化性能得分')
                ax.set_xlabel('Kp', fontsize=12)
                ax.set_ylabel('Ki', fontsize=12)
            
            plt.title(f'{control_type} 控制器参数空间分布\n(颜色越深表示性能越好)', 
                     pad=20, 
                     fontsize=16)
            
            # 标记最优参数点
            if len(df_control) > 0:
                min_score_idx = df_control['WeightedPerformanceScore'].idxmin()
                if pd.notna(min_score_idx):
                    best_params = df_control.loc[min_score_idx]
                    
                    if control_type == 'PID':
                        ax.scatter(best_params['k_p'], 
                                 best_params['k_i'], 
                                 best_params['k_d'],
                                 color='red', 
                                 s=150,
                                 marker='*', 
                                 label='最优参数点')
                    else:
                        ax.scatter(best_params['k_p'],
                                 best_params['k_i'],
                                 color='red',
                                 s=150,
                                 marker='*',
                                 label='最优参数点')
                    
                    print(f"\n{control_type} 控制器最优参数组合:")
                    print(f"Kp = {best_params['k_p']:.4f}")
                    print(f"Ki = {best_params['k_i']:.4f}")
                    if control_type == 'PID':
                        print(f"Kd = {best_params['k_d']:.4f}")
                    print(f"性能得分 = {best_params['WeightedPerformanceScore']:.4f}")
            
            plt.legend(fontsize=14)
            plt.savefig(os.path.join(save_dir, f'parameter_space_{control_type}.png'), 
                       bbox_inches='tight',
                       pad_inches=0.5,
                       dpi=300)
            plt.close()
            
        except Exception as e:
            print(f"绘制 {control_type} 控制器参数空间图时发生错误: {str(e)}")
            plt.close()
            continue

def main():
    # 基本配置
    filename = 'code_2_data_4.csv'
    save_directory = 'plots'
    output_filename = 'performance_results.csv'
    os.makedirs(save_directory, exist_ok=True)
  
    # 数据处理流程
    data = import_data(filename)
    print("数据导入与预处理完成。")

    data = add_param_id(data)
    data_by_control_type = split_data_by_control_type(data)

    # 性能分析
    performance_results = []
    for control_type, ctl_data in data_by_control_type.items():
        unique_param_ids = ctl_data['ParamID'].unique()
        for param_id in unique_param_ids:
            results = analyze_performance(ctl_data, param_id, control_type)
            performance_results.extend(results)

    # 性能评分计算
    performance_df = pd.DataFrame(performance_results)
    config = PerformanceConfig()
    performance_df = calculate_performance_score(performance_df, config)
    
    # 导出性能指标
    performance_df.to_csv(output_filename, index=False)
    print(f"性能指标已导出到文件: {output_filename}")

    # 计算综合排名
    total_scores_df = comprehensive_ranking(performance_df, config.phase_weights)
    print("\n综合排名中各控制类型的数据量:")
    print(total_scores_df['ControlType'].value_counts())
    total_scores_df.to_csv('comprehensive_ranking.csv', index=False)

    # 绘制可视化图表
    plot_with_zoom_inset(total_scores_df, save_directory)
    plot_performance_comparison_with_zoom(performance_df, save_directory, 'SUDDEN_ACCELERATE')
    
    # 参数重要度分析
    importance_scores = calculate_parameter_importance(total_scores_df)

    # 绘制参数分布图
    plot_parameter_histograms(total_scores_df, save_dir=save_directory)
    print("参数分布图绘制完成。")

    # 生成分析报告
    summary_filename = 'summary_report.txt'
    generate_summary_report(total_scores_df, config.phase_weights, summary_filename, importance_scores)
    print(f"分析报告已生成：{summary_filename}")
    
    # 绘制前三名参数组合的速度曲线
    top_3_params = total_scores_df.head(3)
    for idx, row in top_3_params.iterrows():
        param_id = row['ParamID']
        control_type = row['ControlType']
        rank = int(row['Rank'])
        ctl_data = data_by_control_type[control_type]
        plot_full_speed_curve(ctl_data, param_id, control_type, rank, save_dir=save_directory)
    print("前三名参数组合的速度曲线已绘制完成。")

    # 绘制性能分析图表
    plot_radar_chart(performance_df, total_scores_df.head(config.analysis_config['top_n']), save_directory)
    plot_parameter_space(total_scores_df, save_directory)

if __name__ == "__main__":
    main()