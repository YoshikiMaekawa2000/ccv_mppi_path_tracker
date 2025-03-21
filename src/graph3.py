import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# CSVファイルを読み込む
file_path = '../log/full_body/robo_sym/2.0.csv'
data = pd.read_csv(file_path)

# 各列のデータを取得
path_x = data['path_x']
path_y = data['path_y']
x_tf = data['x_tf']
y_tf = data['y_tf']
x = data['x']
y = data['y']
yaw = data['yaw']
omega = data['omega']
steer_l = data['steer_l']
steer_r = data['steer_r']
time = data['time']
v = data['v']
v = np.abs(v)
f_size = 10
path_yaw=[]

for i in range(len(path_x)):
    if(i==0):
        path_yaw.append(np.arctan2(path_y[i+1]-path_y[i], path_x[i+1]-path_x[i]))
    elif(i==len(path_x)-1):
        path_yaw.append(np.arctan2(path_y[i]-path_y[i-1], path_x[i]-path_x[i-1]))
    else:
        path_yaw.append(np.arctan2(path_y[i+1]-path_y[i-1], path_x[i+1]-path_x[i-1]))
# print(path_yaw)

# グラフの描画（2段構成）
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

# 上段のグラフ：軌道 (X-Y)
ax1.plot(path_x, path_y, label='Desired Path', color='blue', linewidth=2, linestyle='dashed')
ax1.plot(x, y, label='Trajectory', color='red', linewidth=2)


# 軸ラベルと設定
ax1.set_xlabel('X [m]', fontsize=f_size)
ax1.set_ylabel('Y [m]', fontsize=f_size)
ax1.tick_params(labelsize=10)
ax1.legend(loc='lower right', fontsize=f_size)
ax1.grid(True)

# 下段のグラフ：omegaの時刻変化
# ax2.plot(time, v, label='Velocity', color='blue', linewidth=2)
ax2.plot(path_x, path_yaw, label='Path Yaw', color='green', linewidth=2)
ax2.plot(x, yaw, label='Yaw', color='blue', linewidth=2)

# 軸ラベルと設定
ax2.set_xlabel('X [m]', fontsize=f_size)
ax2.set_ylabel('Yaw [rad]', fontsize=f_size)
ax2.tick_params(labelsize=10)
ax2.legend(loc='lower right', fontsize=f_size)
ax2.grid(True)

# レイアウト調整
plt.tight_layout(pad=2.0)
plt.show()