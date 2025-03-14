# PID/PI 控制器参数优化实验分析报告

## 1. 实验概述

- 总参数组合数: 110
- 控制器类型分布:
  * PID: 108 组
  * PI: 2 组

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
- Ki = 0.0800
- Kd = 0.2750
- 综合性能得分: 0.6611

#### 性能指标

| 阶段 | 超调量 (%) | 稳态误差 (mm/s) | 响应时间 (s) |
|------|------------|------------------|--------------|
| ACCELERATE | 15.75 | 17.37 | 0.00 |
| DECELERATE | 22.35 | 14.16 | 0.00 |
| STATIONARY | 0.00 | 0.00 | 0.00 |
| SUDDEN_ACCELERATE | 21.44 | 15.10 | 0.15 |
| SUDDEN_STOP | 0.00 | 0.00 | 0.63 |

### 第2名参数组合

- 控制器类型: PID
- Kp = 0.1750
- Ki = 0.0700
- Kd = 0.2750
- 综合性能得分: 0.6727

#### 性能指标

| 阶段 | 超调量 (%) | 稳态误差 (mm/s) | 响应时间 (s) |
|------|------------|------------------|--------------|
| ACCELERATE | 16.91 | 14.08 | 0.00 |
| DECELERATE | 24.42 | 14.32 | 0.00 |
| STATIONARY | 0.00 | 0.00 | 0.00 |
| SUDDEN_ACCELERATE | 24.16 | 14.45 | 0.30 |
| SUDDEN_STOP | 0.00 | 0.00 | 1.02 |

### 第3名参数组合

- 控制器类型: PID
- Kp = 0.1750
- Ki = 0.0700
- Kd = 0.2250
- 综合性能得分: 0.6772

#### 性能指标

| 阶段 | 超调量 (%) | 稳态误差 (mm/s) | 响应时间 (s) |
|------|------------|------------------|--------------|
| ACCELERATE | 15.90 | 12.51 | 0.00 |
| DECELERATE | 23.84 | 18.96 | 0.00 |
| STATIONARY | 0.00 | 0.00 | 0.00 |
| SUDDEN_ACCELERATE | 21.28 | 15.24 | 0.21 |
| SUDDEN_STOP | 0.00 | 0.00 | 1.05 |

### 第4名参数组合

- 控制器类型: PID
- Kp = 0.1800
- Ki = 0.0600
- Kd = 0.2250
- 综合性能得分: 0.6785

#### 性能指标

| 阶段 | 超调量 (%) | 稳态误差 (mm/s) | 响应时间 (s) |
|------|------------|------------------|--------------|
| ACCELERATE | 15.00 | 15.18 | 0.00 |
| DECELERATE | 25.37 | 14.61 | 0.00 |
| STATIONARY | 0.00 | 0.00 | 0.00 |
| SUDDEN_ACCELERATE | 21.30 | 15.88 | 0.30 |
| SUDDEN_STOP | 0.00 | 0.00 | 0.72 |

### 第5名参数组合

- 控制器类型: PID
- Kp = 0.1550
- Ki = 0.0900
- Kd = 0.2750
- 综合性能得分: 0.6894

#### 性能指标

| 阶段 | 超调量 (%) | 稳态误差 (mm/s) | 响应时间 (s) |
|------|------------|------------------|--------------|
| ACCELERATE | 18.10 | 17.00 | 0.00 |
| DECELERATE | 22.40 | 15.95 | 0.00 |
| STATIONARY | 0.00 | 0.00 | 0.00 |
| SUDDEN_ACCELERATE | 27.24 | 12.97 | 0.09 |
| SUDDEN_STOP | 0.00 | 0.01 | 1.53 |

## 4. 参数重要度分析

> 注：得分越低表示该参数值对优化越重要

### k_p 参数分析

| 参数值 | 重要度得分 |
|--------|------------|
| 0.1750 | 2.1944 |
| 0.1850 | 2.7656 |
| 0.1550 | 3.0625 |
| 0.2100 | 3.3333 |
| 0.1650 | 3.5000 |

### k_i 参数分析

| 参数值 | 重要度得分 |
|--------|------------|
| 0.0700 | 3.0625 |
| 0.0600 | 3.1600 |
| 0.0800 | 3.8889 |
| 0.0500 | 4.1600 |
| 0.0900 | 4.1875 |

### k_d 参数分析

| 参数值 | 重要度得分 |
|--------|------------|
| 0.2750 | 1.2153 |
| 0.2250 | 2.9375 |
| 0.2000 | 3.0800 |
| 0.2500 | 3.3889 |
| 0.1750 | 3.8889 |

## 5. 控制器类型比较分析

| 控制器类型 | 参数组数 | 最佳得分 | 平均得分 | 得分标准差 |
|------------|----------|----------|-----------|------------|
| PID | 108 | 0.6611 | 0.7627 | 0.0471 |
| PI | 2 | 0.7718 | 0.8264 | 0.0772 |

## 6. 结论与建议

1. 最优控制方案为 **PID** 控制器
2. 建议参数设置:
   - Kp = 0.2100
   - Ki = 0.0800
   - Kd = 0.2750
