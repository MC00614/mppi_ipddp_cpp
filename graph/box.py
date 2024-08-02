import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

handling_datas = ['MPPI', 'Log_MPPI', 'Smooth_MPPI', 'MPPI_IPDDP']

fig, axs = plt.subplots(2, 2, figsize=(20, 18), sharey=True)

for i, handling_data in enumerate(handling_datas):
    row, col = divmod(i, 2)
    ax1 = axs[row, col]
    ax1.set_title(handling_data, fontsize=25, fontweight='bold', pad=25)

    file_path = 'other_mppi.xlsx'
    df = pd.read_excel(file_path, sheet_name=handling_data)
    avg_time = df['avg_time'].values
    a_msc_x = df['a_msc_x'].values
    a_msc_u = df['a_msc_u'].values
    
    avg_time = avg_time[avg_time != 1]
    a_msc_x = a_msc_x[a_msc_x != 0]
    a_msc_u = a_msc_u[a_msc_u != 0]

    Z1 = avg_time
    Z2 = a_msc_x + a_msc_u

    box1 = ax1.boxplot(Z1, positions=[1], widths=0.5, patch_artist=True)
    ax1.set_ylabel('Average Time', fontsize=15, labelpad=15)
    ax1.set_ylim(-0.1, 1.1)
    ax1.set_xticklabels(['Average Time', 'Curvature X + U'])

    # 색상 추가
    for patch in box1['boxes']:
        patch.set_facecolor('lightblue')  # 박스의 배경 색상

    stats1 = box1['whiskers']
    median1 = box1['medians'][0].get_ydata()[0]
    q1 = stats1[0].get_ydata()[1]
    q3 = stats1[1].get_ydata()[1]
    print(f'{q1:.2f}\t{median1:.2f}\t{q3:.2f}')

    ax1.text(1.260, q1, f'Q1: {q1:.2f}', fontsize=12, ha='left')
    ax1.text(1.260, median1, f'Median: {median1:.2f}', fontsize=12, ha='left')
    ax1.text(1.260, q3, f'Q3: {q3:.2f}', fontsize=12, ha='left')

    ax2 = ax1.twinx()
    box2 = ax2.boxplot(Z2, positions=[2], widths=0.5, patch_artist=True)
    ax2.set_ylabel('Curvature X + U', fontsize=15, labelpad=15)
    ax2.set_ylim(-0.1, 6.1)

    # 색상 추가
    for patch in box2['boxes']:
        patch.set_facecolor('lightgreen')  # 박스의 배경 색상

    stats2 = box2['whiskers']
    median2 = box2['medians'][0].get_ydata()[0]
    q1 = stats2[0].get_ydata()[1]
    q3 = stats2[1].get_ydata()[1]

    ax2.text(2.260, q1, f'Q1: {q1:.2f}', fontsize=12, ha='left')
    ax2.text(2.260, median2, f'Median: {median2:.2f}', fontsize=12, ha='left')
    ax2.text(2.260, q3, f'Q3: {q3:.2f}', fontsize=12, ha='left')
    print(f'{q1:.2f}\t{median1:.2f}\t{q3:.2f}')

plt.subplots_adjust(wspace=0.3, hspace=0.3)
plt.show()
