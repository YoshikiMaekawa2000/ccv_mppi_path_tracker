import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# CSVファイルを読み込む（ファイルパスは適宜修正してください）
# file_path = '../log/mppi/v:2.0_A:1.0_f:0.25.csv'  # CSVファイルのパスを指定してください
# file_path = '../log/pure_pursuit/v:1.2_A:1.0_f:0.25.csv'  # CSVファイルのパスを指定してください
file_path = '../log/pure_pursuit/master_thesis/1.2_steering.csv'
# file_path = '../log/mppi/master_thesis/2.0_steering.csv'

data = pd.read_csv(file_path)

# 各列のデータを取得
path_x = data['path_x']
path_y = data['path_y']
x_tf = data['x_tf']
y_tf = data['y_tf']
no_steer_x = data['no_steer_x']
no_steer_y = data['no_steer_y']

steer_l = data['steer_l']
steer_r = data['steer_r']

time = data['no_steer_t']
v = data['v']
no_steer_v = data['no_steer_v']
# for i in range(len(v)):
#     if(v[i] < 0):
#         v[i] = -v[i]
v=np.abs(v)
f_size=10

# グラフの描画
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(9, 9))

# 上段のグラフ：軌道 (X-Y)
ax1.plot(path_x, path_y, label='Desired Path', color='blue', linewidth=5, linestyle='dashed')
ax1.plot(x_tf, y_tf, label='With Steering Robot', color='red', linewidth=5)
ax1.plot(no_steer_x, no_steer_y, label='Without Steering Robot', color='green', linewidth=5)
# ax1.plot(x_tf, y_tf, label='Dual Pure Pursuit', color='green', linewidth=5)
# ax1.plot(no_steer_x, no_steer_y, label='MPPI', color='red', linewidth=5)

# 軸ラベルとタイトルを設定
ax1.set_xlabel('X [m]', fontsize=f_size)
ax1.set_ylabel('Y [m]', fontsize=f_size)
ax1.tick_params(labelsize=10)

# タイトルをグラフの下に表示
ax1.text(0.5, -0.3, '(a) Trajectory of robot.', ha='center', va='center', transform=ax1.transAxes, fontsize=20)

# 凡例とグリッドを追加
ax1.legend(loc = 'lower right', fontsize=f_size)
ax1.grid(True)
# ax1.set_ylim(-9, 1.5)

# 下段のグラフ：速度 (Time-Velocity)
# ax2.plot(time, v, label='Dual Pure Pursuit', color='blue', linewidth=5)
# ax2.plot(time, no_steer_v, label='MPPI', color='red', linewidth=5)
ax2.plot(time, v, label='With Steering Robot', color='red', linewidth=5)
ax2.plot(time, no_steer_v, label='Without Steering Robot', color='green', linewidth=5)


# 軸ラベルとタイトルを設定
ax2.set_xlabel('Time [s]', fontsize=f_size)
ax2.set_ylabel('Velocity [m/s]', fontsize=f_size)
ax2.tick_params(labelsize=10)

# タイトルをグラフの下に表示
ax2.text(0.5, -0.3, '(b) Linear Speed of Robot.', ha='center', va='center', transform=ax2.transAxes, fontsize=20)

# 凡例とグリッドを追加
ax2.legend(loc = 'lower right', fontsize=f_size)
ax2.grid(True)

ax3.plot(time, steer_l*180/3.14, label='Left Steering Angle', color='blue', linewidth=5)
ax3.plot(time, steer_r*180/3.14, label='Right Steering Angle', color='red', linewidth=5)

# 軸ラベルとタイトルを設定
ax3.set_xlabel('Time [s]', fontsize=f_size)
ax3.set_ylabel('Angle [degree]', fontsize=f_size)
ax3.tick_params(labelsize=10)

# タイトルをグラフの下に表示
ax3.text(0.5, -0.3, '(c) Steering Angle of Robot.', ha='center', va='center', transform=ax3.transAxes, fontsize=20)

# 凡例とグリッドを追加
ax3.legend(loc = 'lower right', fontsize=f_size)
ax3.grid(True)


# グラフを表示
plt.tight_layout(pad=5.0)
plt.show()
