# PID/PI 控制器参数优化实验分析报告

## 1. 实验概述

- 总参数组合数: 370
- 控制器类型分布:
  * PID: 349 组
  * PI: 21 组

## 2. 评价标准

### 2.1 各阶段权重

| 阶段 | 权重 |
|------|------|
| ACCELERATE | 20% |
| DECELERATE | 20% |
| STATIONARY | 5% |
| SUDDEN_ACCELERATE | 45% |
| SUDDEN_STOP | 10% |

### 2.2 性能指标评价范围

| 指标 | 范围 |
|------|------|
| 超调量 | 0-50% |
| 稳态误差 | 0-20mm/s |
| 响应时间 | 0-2.0s |

## 3. 最优参数组合

### 第1名参数组合

- 控制器类型: PID
- Kp = 0.2100
- Ki = 0.0600
- Kd = 0.2750
- 综合性能得分: 0.5809

#### 性能指标

| 阶段 | 超调量 (%) | 稳态误差 (mm/s) | 响应时间 (s) |
|------|------------|------------------|--------------|
| ACCELERATE | 8.80 | 14.46 | 0.00 |
| DECELERATE | 26.63 | 14.54 | 0.00 |
| STATIONARY | 0.00 | 0.00 | 0.00 |
| SUDDEN_ACCELERATE | 16.62 | 13.01 | 0.18 |
| SUDDEN_STOP | 0.00 | 0.00 | 0.54 |

### 第2名参数组合

- 控制器类型: PID
- Kp = 0.1850
- Ki = 0.0500
- Kd = 0.2750
- 综合性能得分: 0.5990

#### 性能指标

| 阶段 | 超调量 (%) | 稳态误差 (mm/s) | 响应时间 (s) |
|------|------------|------------------|--------------|
| ACCELERATE | 4.45 | 13.26 | 0.00 |
| DECELERATE | 20.23 | 15.30 | 0.00 |
| STATIONARY | 0.00 | 0.00 | 0.00 |
| SUDDEN_ACCELERATE | 17.05 | 14.18 | 0.51 |
| SUDDEN_STOP | 0.00 | 0.00 | 0.87 |

### 第3名参数组合

- 控制器类型: PID
- Kp = 0.1950
- Ki = 0.0600
- Kd = 0.2750
- 综合性能得分: 0.6005

#### 性能指标

| 阶段 | 超调量 (%) | 稳态误差 (mm/s) | 响应时间 (s) |
|------|------------|------------------|--------------|
| ACCELERATE | 12.75 | 13.95 | 0.00 |
| DECELERATE | 18.13 | 13.24 | 0.00 |
| STATIONARY | 0.00 | 0.00 | 0.00 |
| SUDDEN_ACCELERATE | 18.37 | 14.54 | 0.30 |
| SUDDEN_STOP | 0.00 | 0.00 | 0.66 |

### 第4名参数组合

- 控制器类型: PID
- Kp = 0.2100
- Ki = 0.0500
- Kd = 0.2000
- 综合性能得分: 0.6027

#### 性能指标

| 阶段 | 超调量 (%) | 稳态误差 (mm/s) | 响应时间 (s) |
|------|------------|------------------|--------------|
| ACCELERATE | 9.79 | 17.63 | 0.00 |
| DECELERATE | 25.20 | 17.17 | 0.00 |
| STATIONARY | 0.00 | 0.00 | 0.00 |
| SUDDEN_ACCELERATE | 14.52 | 12.60 | 0.18 |
| SUDDEN_STOP | 0.00 | 0.00 | 0.78 |

### 第5名参数组合

- 控制器类型: PID
- Kp = 0.1850
- Ki = 0.0500
- Kd = 0.2000
- 综合性能得分: 0.6028

#### 性能指标

| 阶段 | 超调量 (%) | 稳态误差 (mm/s) | 响应时间 (s) |
|------|------------|------------------|--------------|
| ACCELERATE | 6.30 | 18.24 | 0.00 |
| DECELERATE | 21.78 | 13.93 | 0.00 |
| STATIONARY | 0.00 | 0.00 | 0.00 |
| SUDDEN_ACCELERATE | 16.95 | 14.54 | 0.15 |
| SUDDEN_STOP | 0.00 | 0.00 | 0.45 |

## 4. 参数重要度分析

> 注：得分越低表示该参数值对优化越重要

### k_p 参数分析

| 参数值 | 重要度得分 |
|--------|------------|
| 0.1900 | 3.3080 |
| 0.2200 | 3.6844 |
| 0.1850 | 4.7569 |
| 0.1950 | 4.8300 |
| 0.1800 | 5.1240 |

### k_i 参数分析

| 参数值 | 重要度得分 |
|--------|------------|
| 0.0550 | 2.4140 |
| 0.0600 | 2.5512 |
| 0.0500 | 2.5760 |
| 0.0650 | 3.4048 |
| 0.0700 | 4.7751 |

### k_d 参数分析

| 参数值 | 重要度得分 |
|--------|------------|
| 0.2750 | 1.5021 |
| 0.2250 | 1.9679 |
| 0.2500 | 2.2704 |
| 0.2000 | 3.7908 |
| 0.1750 | 5.8681 |

## 5. 控制器类型比较分析

| 控制器类型 | 参数组数 | 最佳得分 | 平均得分 | 得分标准差 |
|------------|----------|----------|-----------|------------|
| PID | 349 | 0.5809 | 0.6996 | 0.0555 |
| PI | 21 | 0.6661 | 0.7721 | 0.0730 |

## 6. 结论与建议

1. 最优控制方案为 **PID** 控制器
2. 建议参数设置:
   - Kp = 0.2100
   - Ki = 0.0600
   - Kd = 0.2750
