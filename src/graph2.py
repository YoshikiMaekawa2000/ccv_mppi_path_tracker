import pandas as pd
import matplotlib.pyplot as plt

# CSVファイルを読み込む（ファイルパスは適宜修正してください）
file_path = '../log/mppi/254.csv'  # CSVファイルのパスを指定してください
data = pd.read_csv(file_path)

# 各列のデータを取得
path_x = data['path_x']
path_y = data['path_y']
x_tf = data['x_tf']
y_tf = data['y_tf']
no_steer_x = data['no_steer_x']
no_steer_y = data['no_steer_y']

time = data['no_steer_t']
v = data['v']
no_steer_v = data['no_steer_v']

# グラフの描画
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 9))

# 上段のグラフ：軌道 (X-Y)
ax1.plot(path_x, path_y, label='Desired Path', color='blue', linewidth=5, linestyle='dashed')
ax1.plot(x_tf, y_tf, label='steering robot', color='red', linewidth=5)
ax1.plot(no_steer_x, no_steer_y, label='NOT steering robot', color='green', linewidth=5)
# ax1.plot(x_tf, y_tf, label='Dual Pure Pursuit', color='green', linewidth=5)
# ax1.plot(no_steer_x, no_steer_y, label='MPPI', color='red', linewidth=5)

# 軸ラベルとタイトルを設定
ax1.set_xlabel('X [m]', fontsize=15)
ax1.set_ylabel('Y [m]', fontsize=15)
ax1.tick_params(labelsize=10)

# タイトルをグラフの下に表示
ax1.text(0.5, -0.3, '(a) Trajectory of robot.', ha='center', va='center', transform=ax1.transAxes, fontsize=20)

# 凡例とグリッドを追加
ax1.legend(loc = 'lower right', fontsize=15)
ax1.grid(True)
# ax1.set_ylim(-9, 1.5)

# 下段のグラフ：速度 (Time-Velocity)
# ax2.plot(time, v, label='Dual Pure Pursuit', color='blue', linewidth=5)
# ax2.plot(time, no_steer_v, label='MPPI', color='red', linewidth=5)
ax2.plot(time, no_steer_v, label='NOT steering robot', color='green', linewidth=5)
ax2.plot(time, v, label='steering robot', color='red', linewidth=5)

# 軸ラベルとタイトルを設定
ax2.set_xlabel('Time [s]', fontsize=15)
ax2.set_ylabel('Velocity [m/s]', fontsize=15)
ax2.tick_params(labelsize=10)

# タイトルをグラフの下に表示
ax2.text(0.5, -0.3, '(b) Linear speed of robot.', ha='center', va='center', transform=ax2.transAxes, fontsize=20)

# 凡例とグリッドを追加
ax2.legend(loc = 'lower right', fontsize=15)
ax2.grid(True)

# グラフを表示
plt.tight_layout(pad=5.0)
plt.show()
