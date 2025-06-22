#!/usr/bin/python3
import pandas as pd
import matplotlib.pyplot as plt

# 读取数据
data = pd.read_csv('yolov8n4.csv')

# 绘制图像
plt.figure(figsize=(12, 10))


# 第一个子图：X数据
plt.subplot(2, 1, 1)  # 2行1列中的第1个位置
plt.plot(data['time'], data['raw_x'], label='Raw X', color='blue', alpha=0.5)
plt.plot(data['time'], data['filtered_x'], label='Filtered X', color='red', linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('X Value')
plt.title('X-Axis Data: Raw vs Filtered')
plt.legend()
plt.grid(True)

# 第二个子图：Z数据
plt.subplot(2, 1, 2)  # 2行1列中的第2个位置
plt.plot(data['time'], data['z_value'], label='Raw Z', color='green', alpha=0.5)
plt.xlabel('Time (s)')
plt.ylabel('Z Value')
plt.legend()
plt.grid(True)

# 调整布局，防止标签重叠
plt.tight_layout()

# 显示图像
plt.show()