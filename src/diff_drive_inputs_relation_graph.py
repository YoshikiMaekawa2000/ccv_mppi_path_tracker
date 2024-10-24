import numpy as np
import matplotlib.pyplot as plt

# 定数
L = 1.0  # 車輪間の距離
v_max = 3  # 車輪の最大速度

# 前進速度と旋回速度の範囲
V_values = np.linspace(-v_max, v_max, 100)
omega_values = np.linspace(-2 * v_max / L, 2 * v_max / L, 100)

# グラフを描画する
plt.figure(figsize=(8, 6))
plt.rcParams["font.size"] = 18

# 満たす領域を計算してプロット
for V in V_values:
    for omega in omega_values:
        # v_r と v_l の計算
        v_r = V + (L * omega) / 2
        v_l = V - (L * omega) / 2
        
        # v_r と v_l が -v_max <= v_r, v_l <= v_max を満たすか確認
        if -v_max <= v_r <= v_max and -v_max <= v_l <= v_max:
            plt.scatter(V, omega, color='blue', s=1)  # 有効な点をプロット

# plt.title("Region where valid v_r and v_l exist for given V and ω")
plt.xlabel("Forward Velocity (V[m/s])")
plt.ylabel("Angular Velocity (ω[rad/s])")
plt.grid(True)
plt.show()
