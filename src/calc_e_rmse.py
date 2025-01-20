import numpy as np
import pandas as pd


# file_path = '../log/mppi/v:2.0_A:1.0_f:0.25.csv'  # CSVファイルのパスを指定してください
file_path = '../log/mppi/v:2.0_A:1.0_f:0.25WOsteer.csv'  # CSVファイルのパスを指定してください
# file_path = '../log/mppi/v:1.2_A:1.0_f:0.25.csv'  # CSVファイルのパスを指定してください
# file_path = '../log/mppi/v:1.2_A:1.0_f:0.25WOsteer.csv'  # CSVファイルのパスを指定してください

# file_path = '../log/pure_pursuit/v:2.0_A:1.0_f:0.25.csv'  # CSVファイルのパスを指定してください
# file_path = '../log/pure_pursuit/v:2.0_A:1.0_f:0.25WOsteer.csv'  # CSVファイルのパスを指定してください
# file_path = '../log/pure_pursuit/v:1.2_A:1.0_f:0.25_WOsteer.csv'  # CSVファイルのパスを指定してください
# file_path = '../log/pure_pursuit/v:1.2_A:1.0_f:0.25.csv'  # CSVファイルのパスを指定してください
data = pd.read_csv(file_path)

import numpy as np

# 欠損値を含む行を削除
valid_data = data[['x_tf', 'y_tf', 'path_x', 'path_y']].dropna()

# path_x と path_y のデータを配列として取得
path_points = np.vstack((valid_data['path_x'], valid_data['path_y'])).T  # 目標軌道の点 (path_x, path_y)

# x_tf と y_tf のロボット軌跡のデータ
robot_points = np.vstack((valid_data['x_tf'], valid_data['y_tf'])).T  # ロボットの位置 (x_tf, y_tf)

# 1. 各ロボットの位置に最も近い目標軌道の点を探す関数
def find_closest_point(robot_point, path_points):
    distances = np.linalg.norm(path_points - robot_point, axis=1)  # 各path点との距離を計算
    return np.min(distances), np.argmin(distances)  # 最小距離とそのインデックスを返す

# 各ロボット位置について最近傍の軌道点を探し、誤差を計算
errors = []
for robot_point in robot_points:
    min_distance, _ = find_closest_point(robot_point, path_points)
    errors.append(min_distance)
time = data['time']
last = time.iloc[-1]
print("Time:", round(last, 1))

# 2. 最大誤差を計算
max_error = np.max(errors)
print("Max Error:", round(max_error, 3))

# 3. RMSE（平均二乗誤差）を計算
rmse_error = np.sqrt(np.mean(np.square(errors)))
print("RMSE Error:", round(rmse_error, 3))
max_error, rmse_error
