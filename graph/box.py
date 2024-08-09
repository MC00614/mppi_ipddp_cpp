import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

handling_datas = ['MPPI', 'Log_MPPI', 'Smooth_MPPI', 'MPPI_IPDDP']

# fig, axs = plt.subplots(2, 2, figsize=(20, 18), sharey=True)

Z1 = {}
Z2 = {}
Z3 = {}

for i, handling_data in enumerate(handling_datas):
    file_path = 'other_mppi.xlsx'
    df = pd.read_excel(file_path, sheet_name=handling_data)
    avg_time = df['avg_time'].values
    a_msc_x = df['a_msc_x'].values
    a_msc_u = df['a_msc_u'].values
    
    avg_time = avg_time[avg_time != 10]
    a_msc_x = a_msc_x[a_msc_x != 0]
    a_msc_u = a_msc_u[a_msc_u != 0]

    Z1[handling_data] = avg_time
    Z2[handling_data] = a_msc_x
    Z3[handling_data] = a_msc_u

offset = 0.22
print('Average Time')
fig, ax = plt.subplots(figsize=(15, 10))
ax.set_title('Average Time', fontsize=35, fontweight='bold', pad=25)
ax.set_ylabel('seconds', fontsize=25, labelpad=15)
ax.set_xticklabels(handling_datas, fontsize=25)
ax.set_ylim(-0.1, 10.1)
ax.tick_params(axis='y', labelsize=20)
for i, handling_data in enumerate(handling_datas):
    box1 = ax.boxplot(Z1[handling_data], positions=[i], widths=0.4, patch_artist=True)
    for patch in box1['boxes']:
        patch.set_facecolor('lightblue')
    stats1 = box1['whiskers']
    median = box1['medians'][0].get_ydata()[0]
    q1 = stats1[0].get_ydata()[1]
    q3 = stats1[1].get_ydata()[1]
    print(f'{handling_data}\t{q1:.6f}\t{median:.6f}\t{q3:.6f}')

    ax.text(i+offset, q1, f'Q1: {q1:.3f}', fontsize=15, ha='left')
    ax.text(i+offset, median, f'Median: {median:.3f}', fontsize=15, ha='left')
    ax.text(i+offset, q3, f'Q3: {q3:.3f}', fontsize=15, ha='left')

offset = -0.29
print('Mean Squared Curvature X')
fig, ax = plt.subplots(figsize=(15, 10))
ax.set_title('Mean Squared Curvature X', fontsize=35, fontweight='bold', pad=25)
ax.set_ylabel('curvature', fontsize=25, labelpad=15)
ax.set_xticklabels(handling_datas, fontsize=25)
ax.set_ylim(-0.001, 0.015)
ax.yaxis.set_major_formatter(ScalarFormatter(useMathText=True))
ax.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
ax.tick_params(axis='y', labelsize=20)
ax.yaxis.get_offset_text().set_fontsize(20)
for i, handling_data in enumerate(handling_datas[:3]):
    box1 = ax.boxplot(Z2[handling_data], positions=[i], widths=0.5, patch_artist=True)
    for patch in box1['boxes']:
        patch.set_facecolor('lightblue')
    stats1 = box1['whiskers']
    median = box1['medians'][0].get_ydata()[0]
    q1 = stats1[0].get_ydata()[1]
    q3 = stats1[1].get_ydata()[1]
    print(f'{handling_data}\t{q1:.6f}\t{median:.6f}\t{q3:.6f}')

    ax.text(i+offset, q1, f'Q1: {q1:.6f}', fontsize=15, ha='right')
    ax.text(i+offset, median, f'Median: {median:.6f}', fontsize=15, ha='right')
    ax.text(i+offset, q3, f'Q3: {q3:.6f}', fontsize=15, ha='right')
ax3 = ax.twinx()
ax3.spines['right'].set_position(('outward', -450))
ax3.set_ylim(-0.00001, 0.00033)
ax3.yaxis.set_major_formatter(ScalarFormatter(useMathText=True))
ax3.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
ax3.tick_params(axis='y', labelsize=20)
ax3.yaxis.get_offset_text().set_fontsize(20)
ax3.yaxis.get_offset_text().set_position((0.76, 0))
box3 = ax3.boxplot(Z2['MPPI_IPDDP'], positions=[3.5], widths=0.5, patch_artist=True)
for patch in box3['boxes']:
    patch.set_facecolor('lightblue')
stats1 = box3['whiskers']
median = box3['medians'][0].get_ydata()[0]
q1 = stats1[0].get_ydata()[1]
q3 = stats1[1].get_ydata()[1]
offset = 0.2
ax3.text(3+offset, q1, f'Q1: {q1:.6f}', fontsize=15, ha='right')
ax3.text(3+offset, median, f'Median: {median:.6f}', fontsize=15, ha='right')
ax3.text(3+offset, q3, f'Q3: {q3:.6f}', fontsize=15, ha='right')
print(f'{"MPPI_IPDDP"}\t{q1:.6f}\t{median:.6f}\t{q3:.6f}')


offset = -0.29
print('Mean Squared Curvature U')
fig, ax = plt.subplots(figsize=(15, 10))
ax.set_title('Mean Squared Curvature U', fontsize=35, fontweight='bold', pad=25)
ax.set_ylabel('curvature', fontsize=25, labelpad=15)
ax.set_xticklabels(handling_datas, fontsize=25)
ax.set_ylim(-0.0, 6.5)
ax.yaxis.set_major_formatter(ScalarFormatter(useMathText=True))
ax.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
ax.tick_params(axis='y', labelsize=20)
ax.yaxis.get_offset_text().set_fontsize(20)
for i, handling_data in enumerate(handling_datas[:3]):
    box1 = ax.boxplot(Z3[handling_data], positions=[i], widths=0.5, patch_artist=True)
    for patch in box1['boxes']:
        patch.set_facecolor('lightblue')
    stats1 = box1['whiskers']
    median = box1['medians'][0].get_ydata()[0]
    q1 = stats1[0].get_ydata()[1]
    q3 = stats1[1].get_ydata()[1]
    print(f'{handling_data}\t{q1:.6f}\t{median:.6f}\t{q3:.6f}')

    ax.text(i+offset, q1, f'Q1: {q1:.6f}', fontsize=15, ha='right')
    ax.text(i+offset, median, f'Median: {median:.6f}', fontsize=15, ha='right')
    ax.text(i+offset, q3, f'Q3: {q3:.6f}', fontsize=15, ha='right')
ax3 = ax.twinx()
ax3.spines['right'].set_position(('outward', -450))
# ax3.set_ylabel(r'curvature [x $10^{-1}$]', fontsize=25, labelpad=15)
ax3.set_ylim(-0.0, 0.15)
ax3.yaxis.set_major_formatter(ScalarFormatter(useMathText=True))
ax3.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
ax3.tick_params(axis='y', labelsize=20)
ax3.yaxis.get_offset_text().set_fontsize(20)
ax3.yaxis.get_offset_text().set_position((0.76, 0))
box3 = ax3.boxplot(Z3['MPPI_IPDDP'], positions=[3.5], widths=0.5, patch_artist=True)
for patch in box3['boxes']:
    patch.set_facecolor('lightblue')
stats1 = box3['whiskers']
median = box3['medians'][0].get_ydata()[0]
q1 = stats1[0].get_ydata()[1]
q3 = stats1[1].get_ydata()[1]
offset = 0.2
# ax3.text(3+offset, q1, f'Q1: {q1:.6f}', fontsize=15, ha='right')
ax3.text(3+offset, q1 - 0.001, f'Q1: {q1:.6f}', fontsize=15, ha='right')
ax3.text(3+offset, median, f'Median: {median:.6f}', fontsize=15, ha='right')
ax3.text(3+offset, q3, f'Q3: {q3:.6f}', fontsize=15, ha='right')
print(f'{"MPPI_IPDDP"}\t{q1:.6f}\t{median:.6f}\t{q3:.6f}')

plt.subplots_adjust(wspace=0.3, hspace=0.3)
plt.show()
