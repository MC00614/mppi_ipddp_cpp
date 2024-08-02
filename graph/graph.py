import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

handling_datas = ['MPPI', 'Log_MPPI', 'Smooth_MPPI', 'MPPI_IPDDP']

for handling_data in handling_datas:
    file_path = 'other_mppi.xlsx'
    df = pd.read_excel(file_path, sheet_name=handling_data)

    # N	S_u	P
    X = [i/10 for i in range(1,10)]
    Y = [100 * (2**i) for i in range(9)]
    X, Y = np.meshgrid(X, Y)

    Z1 = df['P'].values.reshape(X.shape)
    Z2 = df['avg_time'].values.reshape(X.shape)
    Z3 = df['a_msc_x'].values.reshape(X.shape)
    Z4 = df['a_msc_u'].values.reshape(X.shape)

    fig = plt.figure()
    fig.suptitle(handling_data, fontsize=50, fontweight='bold')

    ax1 = fig.add_subplot(221, projection='3d')
    surf = ax1.plot_surface(X, Y, Z1, cmap='coolwarm')
    ax1.set_xlabel('Sigma_u', fontsize=16, labelpad=15)
    ax1.set_ylabel('N', fontsize=16, labelpad=15)
    ax1.set_zlabel('Success', fontsize=16, labelpad=15)
    ax1.view_init(azim=-130, elev=50)
    ax1.set_zlim(0, 10)
    ax1.set_title('Success', fontsize=30, pad=40, fontweight='bold')

    ax1 = fig.add_subplot(222, projection='3d')
    surf = ax1.plot_surface(X, Y, Z2, cmap='coolwarm')
    ax1.set_xlabel('Sigma_u', fontsize=16, labelpad=15)
    ax1.set_ylabel('N', fontsize=16, labelpad=15)
    ax1.set_zlabel('Time', fontsize=16, labelpad=15)
    ax1.set_zlim(0, 1)
    ax1.view_init(azim=-130, elev=50)
    ax1.set_title('Average Time', fontsize=30, pad=40, fontweight='bold')

    ax1 = fig.add_subplot(223, projection='3d')
    surf = ax1.plot_surface(X, Y, Z3, cmap='coolwarm')
    ax1.set_xlabel('Sigma_u', fontsize=16, labelpad=15)
    ax1.set_ylabel('N', fontsize=16, labelpad=15)
    ax1.set_zlabel('Curvature', fontsize=16, labelpad=15)
    ax1.view_init(azim=-130, elev=50)
    ax1.set_zlim(0, 0.01)
    ax1.set_title('Mean Squared Curvature X', fontsize=30, pad=40, fontweight='bold')

    ax1 = fig.add_subplot(224, projection='3d')
    surf = ax1.plot_surface(X, Y, Z4, cmap='coolwarm')
    ax1.set_xlabel('Sigma_u', fontsize=16, labelpad=15)
    ax1.set_ylabel('N', fontsize=16, labelpad=15)
    ax1.set_zlabel('Curvature', fontsize=16, labelpad=15)
    ax1.view_init(azim=-130, elev=50)
    ax1.set_zlim(0, 6)
    ax1.set_title('Mean Squared Curvature U', fontsize=30, pad=40, fontweight='bold')

    plt.show()