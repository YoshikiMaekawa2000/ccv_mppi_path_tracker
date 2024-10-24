import pandas as pd
import matplotlib.pyplot as plt

# CSVファイルを読み込む（ファイルパスは適宜修正してください）
file_path = '../log/pure_pursuit/ALL_pp_single_cos.csv'  # CSVファイルのパスを指定してください
data = pd.read_csv(file_path)

# オドメトリのX座標とY座標を取得（C列とD列のデータ）
# odom_x = data['x_tf']  # 1行目の列名に対応
# odom_y = data['y_tf']  # 1行目の列名に対応
# desired_x = data['path_x']
# desired_y = data['path_y']
# no_steer_x = data['no_steer_x']
# no_steer_y = data['no_steer_y']
# true_x = data['x']
# true_y = data['y']
# no_steer_true_x = data['no_steer_true_x']
# no_steer_true_y = data['no_steer_true_y']
cmd_v = data['cmd_v']
true_v = data['v']
no_steer_cmd_v = data['no_steer_cmd_v']
no_steer_v = data['no_steer_v']

# グラフを作成
plt.figure()
plt.rcParams["font.size"] =30
# plt.plot(odom_x, odom_y, label='Robot trajectory WITH steering', linewidth=5,color='red')
# plt.plot(no_steer_x, no_steer_y, label='Robot trajectory WITHOUT steering', linewidth=5,color='blue')
# plt.plot(true_x, true_y, label='Robot trajectory', linewidth=5,color='red')
# plt.plot(no_steer_true_x, no_steer_true_y, label='Robot trajectory WITHOUT steering', linewidth=5,color='blue')
# plt.plot(desired_x, desired_y, label='Desired Path', linewidth=5, linestyle='dashed', color='green')
plt.plot(true_v, label='Linear Velocity WITH steering robot', linewidth=5,color='red', linestyle='dashed')
plt.plot(no_steer_v, label='Linear Velocity WITHOUT steering robot', linewidth=5,color='blue', linestyle='dashed')

# グラフのタイトルと軸ラベル
# plt.title('Odometry X-Y Trajectory')
plt.xlabel('x [m]', fontsize=30)
plt.ylabel('y [m]', fontsize=30)
# plt.ylim(-7, 1.5)
plt.tight_layout()

# グリッドを追加
plt.grid(True)

# 凡例を追加
plt.legend(loc='lower right', fontsize=30)

# グラフを表示
plt.show()
