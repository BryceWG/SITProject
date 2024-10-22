import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
from typing import List, Tuple

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

    # 统一列名（已与数据一致，无需重命名）
    # 如果未来有不同的列名，可以在这里添加重命名逻辑
    # 例如：
    # data = data.rename(columns={
    #     'OldName': 'NewName',
    #     ...
    # })

    print("统一列名完成。当前列名为:", list(data.columns))

    # 删除所有 'Unnamed' 列
    unnamed_cols = [col for col in data.columns if col.startswith('Unnamed')]
    if unnamed_cols:
        data = data.drop(columns=unnamed_cols)
        print(f"已删除未命名的列: {unnamed_cols}")
    else:
        print("没有未命名的列需要删除。")

    # 检查关键列是否存在
    required_columns = ['ControlType', 'Time', 'k_p', 'k_i', 'k_d', 'TargetSpeed_mm_s', 'ActualSpeed_mm_s', 'Error_mm_s', 'Phase']
    missing_columns = [col for col in required_columns if col not in data.columns]
    if missing_columns:
        print(f"警告: 缺失必要的列: {missing_columns}")
    else:
        print("所有必要的列均存在。")

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

def add_param_id(data: pd.DataFrame, control_type: str) -> pd.DataFrame:
    """
    为每个参数组合添加一个唯一的参数ID。

    参数:
    data (pd.DataFrame): 包含实验数据的DataFrame。
    control_type (str): 控制类型，如'PID'或'PI'。

    返回:
    pd.DataFrame: 包含参数ID的DataFrame。
    """
    if control_type == 'PID':
        params = data[['k_p', 'k_i', 'k_d']].drop_duplicates()
        merge_on = ['k_p', 'k_i', 'k_d']
    elif control_type == 'PI':
        params = data[['k_p', 'k_i']].drop_duplicates()
        merge_on = ['k_p', 'k_i']
        # 对于 PI 控制器，设置 k_d 为 NaN
        params['k_d'] = np.nan
    else:
        params = data[['k_p']].drop_duplicates()
        merge_on = ['k_p']
        params['k_i'] = np.nan
        params['k_d'] = np.nan

    params = params.reset_index(drop=True)
    params['ParamID'] = range(1, len(params) + 1)
    merged_data = pd.merge(data, params, on=merge_on, how='left')

    # 确保 'k_d' 列存在
    if 'k_d' not in merged_data.columns:
        merged_data['k_d'] = np.nan

    # 调试信息：检查 'k_d' 列是否正确
    if control_type == 'PI':
        missing_k_d = merged_data['k_d'].isna().sum()
        print(f"PI 控制类型下，k_d 缺失数量: {missing_k_d}")

    # 调试信息：确认 'ParamID' 列是否存在
    if 'ParamID' not in merged_data.columns:
        print(f"错误: 'ParamID' 列在 {control_type} 控制类型的数据中不存在。")
    else:
        print(f"控制类型: {control_type}, 参数组合数: {len(params)}, ParamID 范围: 1 - {len(params)}")

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
    overshoot_percentage = (overshoot / target_speed) * 100 if target_speed != 0 else np.nan
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

def analyze_performance(data: pd.DataFrame, param_id: int, phase: str, control_type: str) -> dict:
    """
    分析特定参数组合和运动阶段的性能指标。

    参数:
    data (pd.DataFrame): 包含实验数据的DataFrame。
    param_id (int): 参数组合ID。
    phase (str): 运动阶段。
    control_type (str): 控制类型。

    返回:
    dict: 包含性能指标的字典。
    """
    phase_data = data[(data['ParamID'] == param_id) & (data['Phase'] == phase)]

    overshoot, overshoot_percentage = calculate_overshoot(phase_data)
    steady_state_error = calculate_steady_state_error(phase_data)

    # 提取控制参数
    param_rows = data[data['ParamID'] == param_id]
    if param_rows.empty:
        print(f"警告: 未找到 ParamID {param_id} 的数据。")
        return {}
    param_row = param_rows.iloc[0]
    
    # 检查 'k_i' 是否存在
    if 'k_i' not in param_row or pd.isna(param_row['k_i']):
        print(f"警告: ParamID {param_id} 的 'k_i' 缺失或为 NaN。")
        k_i = np.nan
    else:
        k_i = param_row['k_i']
    
    # 'k_d' 应该存在，即使是 NaN
    if 'k_d' not in param_row:
        print(f"警告: ParamID {param_id} 的 'k_d' 缺失。")
        k_d = np.nan
    else:
        k_d = param_row['k_d']

    k_p = param_row['k_p']

    performance = {
        'ControlType': control_type,
        'ParamID': param_id,
        'Phase': phase,
        'k_p': k_p,
        'k_i': k_i,
        'k_d': k_d,
        'Overshoot': overshoot,
        'OvershootPercentage': overshoot_percentage,
        'SteadyStateError': steady_state_error
    }

    # 打印每次性能指标计算结果
    print(f"分析性能 - 控制类型: {control_type}, ParamID: {param_id}, 阶段: {phase}, 超调量%: {overshoot_percentage:.4f}, 稳态误差: {steady_state_error:.4f}")

    return performance

# 4. 选择最佳参数组合
def select_best_parameters(performance_df: pd.DataFrame, control_type: str, phase: str, top_n: int =3) -> pd.DataFrame:
    """
    根据最小稳态误差和超调量选择最佳参数组合。

    参数:
    performance_df (pd.DataFrame): 包含性能指标的DataFrame。
    control_type (str): 控制类型。
    phase (str): 运动阶段。
    top_n (int): 选择前N个最佳参数组合。

    返回:
    pd.DataFrame: 最佳参数组合的信息。
    """
    # 筛选出指定控制类型和运动阶段的数据
    df = performance_df[
        (performance_df['ControlType'] == control_type) &
        (performance_df['Phase'] == phase)
    ].dropna(subset=['OvershootPercentage', 'SteadyStateError'])

    # 打印筛选后的数据量
    print(f"选择最佳参数 - 控制类型: {control_type}, 阶段: {phase}, 数据量: {len(df)}")

    if df.empty:
        return pd.DataFrame()

    # 定义性能指标的权重或重要性
    # 这里简单地将超调量百分比和稳态误差相加，可以根据需要调整权重
    df['PerformanceScore'] = df['OvershootPercentage'] + df['SteadyStateError']

    # 排序并选择前N个
    best_params = df.sort_values('PerformanceScore').head(top_n)

    return best_params

# 5. 绘图
def plot_performance_comparison(performance_df: pd.DataFrame):
    """
    绘制不同控制方法和参数组合的性能指标比较图，不使用过滤器。

    参数:
    performance_df (pd.DataFrame): 包含性能指标的DataFrame。
    """
    # 绘制超调量比较
    plt.figure(figsize=(12, 8))
    for ctl_type in performance_df['ControlType'].unique():
        df = performance_df[performance_df['ControlType'] == ctl_type]
        df_sorted = df.sort_values('ParamID')
        plt.plot(df_sorted['ParamID'], df_sorted['OvershootPercentage'], marker='o', linestyle='-', label=f'{ctl_type}')
    plt.xlabel('参数组合ID')
    plt.ylabel('超调量 (%)')
    plt.title('超调量比较图')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # 绘制稳态误差比较
    plt.figure(figsize=(12, 8))
    for ctl_type in performance_df['ControlType'].unique():
        df = performance_df[performance_df['ControlType'] == ctl_type]
        df_sorted = df.sort_values('ParamID')
        plt.plot(df_sorted['ParamID'], df_sorted['SteadyStateError'], marker='s', linestyle='-', label=f'{ctl_type}')
    plt.xlabel('参数组合ID')
    plt.ylabel('稳态误差 (mm/s)')
    plt.title('稳态误差比较图')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # 绘制超调量与稳态误差的关系图
    plt.figure(figsize=(12, 8))
    markers = {'PID': 'o', 'PI': 's'}  # 根据实际控制类型调整
    for ctl_type in performance_df['ControlType'].unique():
        df = performance_df[performance_df['ControlType'] == ctl_type]
        plt.scatter(df['OvershootPercentage'], df['SteadyStateError'], 
                    marker=markers.get(ctl_type, 'x'), label=f'{ctl_type}', alpha=0.6)
    plt.xlabel('超调量 (%)')
    plt.ylabel('稳态误差 (mm/s)')
    plt.title('超调量与稳态误差关系图')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_speed_curves(data: pd.DataFrame, param_id: int, control_type: str, phase: str, rank: int):
    """
    绘制特定参数组合在指定运动阶段的速度-时间曲线。

    参数:
    data (pd.DataFrame): 包含实验数据的DataFrame。
    param_id (int): 参数组合ID。
    control_type (str): 控制类型。
    phase (str): 运动阶段。
    rank (int): 参数组合的排名。
    """
    param_data = data[(data['ParamID'] == param_id) & (data['Phase'] == phase)]
    if param_data.empty:
        print(f"未找到参数ID {param_id} 在 {phase} 阶段的数据。")
        return

    plt.figure(figsize=(12, 8))
    plt.plot(param_data['Time'], param_data['TargetSpeed_mm_s'], 'r-', linewidth=2, label='目标速度')
    plt.plot(param_data['Time'], param_data['ActualSpeed_mm_s'], 'b-', linewidth=2, label='实际速度')
    plt.xlabel('时间 (秒)')
    plt.ylabel('速度 (mm/s)')
    plt.title(f"控制类型: {control_type}, 参数ID: {param_id}, 阶段: {phase}, 排名: {rank}")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_best_parameters_trends(all_best_params_df: pd.DataFrame):
    """
    绘制不同运动阶段的最佳参数（包括第二、第三名）的变化趋势图。

    参数:
    all_best_params_df (pd.DataFrame): 所有最佳参数组合的汇总DataFrame。
    """
    if all_best_params_df.empty:
        print("没有最佳参数组合数据，无法绘制趋势图。")
        return

    motion_phases = all_best_params_df['Phase'].unique()
    control_types = all_best_params_df['ControlType'].unique()

    # 绘制k_p变化趋势
    plt.figure(figsize=(14, 10))
    for ctl_type in control_types:
        df = all_best_params_df[all_best_params_df['ControlType'] == ctl_type]
        for rank in sorted(df['Rank'].unique()):
            subset = df[df['Rank'] == rank]
            if subset.empty:
                continue
            plt.plot(subset['Phase'], subset['k_p'], marker='o', label=f'{ctl_type} k_p 第{int(rank)}名')
    plt.xlabel('运动阶段')
    plt.ylabel('k_p 值')
    plt.title('不同运动阶段的最佳k_p值变化趋势')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # 绘制k_i变化趋势
    plt.figure(figsize=(14, 10))
    for ctl_type in control_types:
        df = all_best_params_df[all_best_params_df['ControlType'] == ctl_type]
        for rank in sorted(df['Rank'].unique()):
            subset = df[df['Rank'] == rank]
            if subset.empty:
                continue
            plt.plot(subset['Phase'], subset['k_i'], marker='s', label=f'{ctl_type} k_i 第{int(rank)}名')
    plt.xlabel('运动阶段')
    plt.ylabel('k_i 值')
    plt.title('不同运动阶段的最佳k_i值变化趋势')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # 绘制k_d变化趋势
    plt.figure(figsize=(14, 10))
    for ctl_type in control_types:
        df = all_best_params_df[all_best_params_df['ControlType'] == ctl_type]
        for rank in sorted(df['Rank'].unique()):
            subset = df[df['Rank'] == rank]
            if subset.empty:
                continue
            plt.plot(subset['Phase'], subset['k_d'], marker='^', label=f'{ctl_type} k_d 第{int(rank)}名')
    plt.xlabel('运动阶段')
    plt.ylabel('k_d 值')
    plt.title('不同运动阶段的最佳k_d值变化趋势')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# 6. 主程序
def main():
    """
    主程序，执行数据导入、分割、分析和绘图。
    """
    # ---- 您可以在此处调整自定义参数值 ----
    filename = 'code_2_data_3.csv'  # 请替换为您的数据文件名
    # ----------------------------------------

    # 定义要分析的运动阶段
    motion_phases_to_analyze = ['ACCELERATE', 'DECELERATE', 'STATIONARY', 'SUDDEN_ACCELERATE', 'SUDDEN_STOP']

    # 导入数据
    data = import_data(filename)
    print("\n数据导入与预处理完成。")

    # 检查是否存在 'Phase' 列
    if 'Phase' not in data.columns:
        print("错误: 数据中缺少 'Phase' 列。请确保实验数据中包含 'Phase' 列。")
        return
    else:
        print("存在 'Phase' 列，继续处理。")

    # 打印存在的控制类型及其数量
    print("\n存在的控制类型及其数量:")
    print(data['ControlType'].value_counts())

    # 检查每种控制类型在各运动阶段的数据量
    for control_type in data['ControlType'].unique():
        print(f"\n控制类型: {control_type}")
        ctl_data = data[data['ControlType'] == control_type]
        motion_counts = ctl_data['Phase'].value_counts()
        for phase in motion_phases_to_analyze:
            count = motion_counts.get(phase, 0)
            print(f"  {phase} 阶段的数据量: {count}")

    # 数据分割
    data_by_control_type = split_data_by_control_type(data)
    performance_results = []

    for control_type, ctl_data in data_by_control_type.items():
        print(f"\n处理控制类型: {control_type}")

        # 添加参数ID
        ctl_data = add_param_id(ctl_data, control_type)
        print(f"参数ID分配完成。当前数据量: {len(ctl_data)}")

        # 更新 data_by_control_type 字典
        data_by_control_type[control_type] = ctl_data

        # 确认 'Phase' 列存在
        if 'Phase' not in ctl_data.columns:
            print(f"错误: 控制类型 {control_type} 的数据中缺少 'Phase' 列。")
            continue

        # 确认 'Phase' 列是否存在有效数据
        if ctl_data['Phase'].isnull().all():
            print(f"错误: 控制类型 {control_type} 的数据中 'Phase' 列全为缺失值。")
            continue

        # 数据分析
        unique_param_ids = ctl_data['ParamID'].unique()
        print(f"开始分析控制类型: {control_type}, 参数组合数量: {len(unique_param_ids)}")
        for param_id in unique_param_ids:
            for phase in motion_phases_to_analyze:
                # 仅分析有数据的运动阶段
                if ((ctl_data['ParamID'] == param_id) & (ctl_data['Phase'] == phase)).any():
                    result = analyze_performance(ctl_data, param_id, phase, control_type)
                    if result:  # 仅添加非空结果
                        performance_results.append(result)

    # 汇总性能指标
    performance_df = pd.DataFrame(performance_results)
    print("\n性能指标计算完成。")

    # 检查 'ParamID' 列是否存在于 performance_df
    if 'ParamID' not in performance_df.columns:
        print("错误: 'ParamID' 列在性能指标 DataFrame 中不存在。")
        return

    # 绘制性能比较图
    plot_performance_comparison(performance_df)
    print("性能比较图绘制完成。")

    # 存储所有最佳参数组合
    all_best_params = []

    # 选择最佳参数组合并分析各运动阶段的最优参数
    for phase in motion_phases_to_analyze:
        print(f"\n运动阶段: {phase} 的最佳参数组合:")
        for control_type in performance_df['ControlType'].unique():
            best_params = select_best_parameters(performance_df, control_type, phase, top_n=3)
            if not best_params.empty:
                # 添加排名信息
                best_params = best_params.copy()
                best_params['Rank'] = best_params['PerformanceScore'].rank(method='dense', ascending=True).astype(int)

                print(f"\n控制类型: {control_type}")
                print(best_params[['ParamID', 'k_p', 'k_i', 'k_d', 'OvershootPercentage', 'SteadyStateError', 'PerformanceScore', 'Rank']].to_string(index=False))

                # 绘制最佳参数组合的速度曲线
                for idx, row in best_params.iterrows():
                    rank = int(row['Rank'])
                    param_id = row['ParamID']
                    # 检查 'ParamID' 是否存在于控制类型的数据中
                    if 'ParamID' not in data_by_control_type[control_type].columns:
                        print(f"错误: 'ParamID' 列在 {control_type} 控制类型的数据中不存在。")
                        continue
                    plot_speed_curves(data_by_control_type[control_type], param_id, control_type, phase, rank)
                    # 添加排名信息
                    all_best_params.append(row)
            else:
                print(f"未找到 {control_type} 控制类型在 {phase} 阶段的最佳参数组合。")

    # 研究不同运动阶段的最优参数组合的关系
    if all_best_params:
        # 将所有最佳参数组合汇总到一个DataFrame
        all_best_params_df = pd.DataFrame(all_best_params)
        # 重置索引
        all_best_params_df.reset_index(drop=True, inplace=True)
        print("\n所有最佳参数组合汇总完成。")

        # 检查 'k_d' 列是否存在
        if 'k_d' not in all_best_params_df.columns:
            print("警告: 'k_d' 列在最佳参数组合 DataFrame 中不存在。")
        else:
            print("所有最佳参数组合中存在 'k_d' 列。")

        # 绘制不同运动阶段的最佳k_p、k_i、k_d值变化趋势
        plot_best_parameters_trends(all_best_params_df)
        print("最佳参数趋势图绘制完成。")
    else:
        print("未找到任何最佳参数组合。")

if __name__ == "__main__":
    main()
