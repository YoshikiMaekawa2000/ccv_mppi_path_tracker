import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# CSVファイルを読み込む（ファイルパスは適宜修正してください）
# file_path = '../log/mppi/v:2.0_A:1.0_f:0.25.csv'  # CSVファイルのパスを指定してください
# file_path = '../log/pure_pursuit/v:1.2_A:1.0_f:0.25.csv'  # CSVファイルのパスを指定してください
# file_path = '../log/pure_pursuit/master_thesis/2.0_steering.csv'
file_path = '../log/full_body/master_thesis/not_controlled.csv'

data = pd.read_csv(file_path)

# 各列のデータを取得
path_x = data['path_x']
path_y = data['path_y']
x_tf = data['x_tf']
y_tf = data['y_tf']
# no_steer_x = data['no_steer_x']
# no_steer_y = data['no_steer_y']

steer_l = data['steer_l']
steer_r = data['steer_r']

time = data['time']
v = data['v']
# no_steer_v = data['no_steer_v']
zmp_y = data['zmp_y']
treu_zmp_y = data['true_zmp']
roll = data['roll']

v=np.abs(v)
f_size=15

zmp_sum = 0
# for i in range(len(treu_zmp_y)):
#     zmp_sum += abs(treu_zmp_y[i])
# zmp_sum = zmp_sum/len(treu_zmp_y)
# print("ZMP sum: ", zmp_sum)

# グラフの描画
# fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(9, 9))

# 上段のグラフ：ZMP
plt.figure(figsize=(10, 4))
plt.plot(time, treu_zmp_y, label='ZMP-Y', color='blue')
# plt.plot(time, zmp_y, label='ZMP-Y', color='red', linewidth=2)
plt.plot(time, 0*np.ones(len(time)), color='black', linestyle='dashed')

# 軸ラベルとタイトルを設定
plt.xlabel('Time [s]', fontsize=f_size)
plt.ylabel('zmp y [m]', fontsize=f_size)
plt.tick_params(labelsize=10)


# タイトルをグラフの下に表示
# plt.text(0.5, -0.3, '(a) Displacement from the Center of ZMP.', ha='center', va='center', transform=ax1.transAxes, fontsize=20, clip_on=True)

# 凡例とグリッドを追加
plt.legend(loc = 'lower right', fontsize=f_size)
plt.grid(True)
plt.ylim(-0.15, 0.15)

# グラフを表示
plt.tight_layout(pad=5.0)
plt.show()
