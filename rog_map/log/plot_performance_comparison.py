import os
import pandas as pd
import matplotlib.pyplot as plt

# 指定文件夹路径
folder_path = 'performance'


# 获取文件夹中的所有CSV文件
csv_files = [file for file in os.listdir(folder_path) if file.endswith('.csv')]
num_columns = len(csv_files)
# 遍历每个CSV文件绘制数据
for i, file in enumerate(csv_files):
    # 构建CSV文件的完整路径
    file_path = os.path.join(folder_path, file)

    # 读取CSV文件数据
    data = pd.read_csv(file_path)
    num_columns = data.columns.size
    break

figw = num_columns//2+1
fig, axes = plt.subplots(nrows=2, ncols=figw, figsize=(6 * num_columns /2, 6 ))

# 遍历每个CSV文件绘制数据
for i, file in enumerate(csv_files):
    # 构建CSV文件的完整路径
    file_path = os.path.join(folder_path, file)

    # 读取CSV文件数据
    data = pd.read_csv(file_path)
    # 获取列标签
    columns = data.columns.tolist()
    # 遍历每一列数据
    for j, column in enumerate(data.columns):
        row = j // (num_columns//2+1)
        col = j - figw*row
        print(row,col)
        ax = axes[row, col]

        # 绘制当前CSV文件的第j列数据
        ax.plot(data[column], label=f'{file}')

        ax.set_xlabel(columns[j])
        ax.legend()

# 调整子图之间的间距
plt.subplots_adjust(wspace=0.3)

# 显示图表
plt.show()