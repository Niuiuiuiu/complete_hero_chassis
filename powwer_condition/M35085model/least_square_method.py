import numpy as np
import pandas as pd
from scipy.optimize import curve_fit

# 定义要拟合的非线性函数
def func_power(data, k0, k1, k2, k3, k4, k5):
    I, ω = data
    return (k0 + k1 * I + k2 * ω + k3 * I * ω + k4 * I**2 + k5 * ω**2)

# 读取数据
array_data = pd.read_csv('vofa+.csv').values

# 提取数据
array_power_data = array_data[:, 0]
array_I_data = array_data[:, 1]
array_ω_data = array_data[:, 2]

# 使用 curve_fit 进行最小二乘拟合
params, covariance = curve_fit(func_power, (array_I_data, array_ω_data), array_power_data, p0=[1, 1, 1, 1, 1, 1])

# 获取拟合的参数
k0, k1, k2, k3, k4, k5 = params

# 输出拟合的参数
print('k0:', k0)
print('k1:', k1)
print('k2:', k2)
print('k3:', k3)
print('k4:', k4)
print('k5:', k5)

# 计算 R²
predicted = func_power((array_I_data, array_ω_data), *params)
ss_res = np.sum((array_power_data - predicted) ** 2)
ss_tot = np.sum((array_power_data - np.mean(array_power_data)) ** 2)
r2 = 1 - ss_res / ss_tot
print('R2:', r2)

# ============================================================
# 绘图：实际功率 vs 预测功率 + 残差图
# ============================================================
import matplotlib.pyplot as plt

residual = array_power_data - predicted
sample_idx = np.arange(len(array_power_data))

fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True,
                         gridspec_kw={'height_ratios': [3, 1]})

# 上图：实际 vs 预测
ax1 = axes[0]
ax1.plot(sample_idx, array_power_data, alpha=0.8, linewidth=0.5,
         label='Actual', color='#d62728')
ax1.plot(sample_idx, predicted, alpha=0.8, linewidth=0.6,
         label='Predicted', color='#1f77b4')
ax1.set_ylabel('Power')
ax1.set_title(f'Actual vs Predicted Power  (R2={r2:.4f})')
ax1.legend(loc='upper right', fontsize=10, framealpha=0.9)
ax1.grid(True, alpha=0.3)

# 下图：残差
ax2 = axes[1]
ax2.fill_between(sample_idx, 0, residual, alpha=0.35, color='#d62728')
ax2.plot(sample_idx, residual, alpha=0.7, linewidth=0.3, color='#d62728')
ax2.axhline(0, color='black', linestyle='--', linewidth=0.8)
ax2.set_xlabel('Sample Index (Time Order)')
ax2.set_ylabel('Residual')
ax2.set_title('Residual (Actual - Predicted)')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('power_time_residual.png', dpi=150)
print('Plot saved to power_time_residual.png')
