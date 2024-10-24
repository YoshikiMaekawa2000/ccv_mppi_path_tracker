import numpy as np
import matplotlib.pyplot as plt

# 定数
L = 0.5  # 車輪間の距離
v_max = 3.0  # 車輪の最大速度
delta_max = 30 * np.pi / 180  # 車輪の最大旋回角度

# 前進速度と旋回速度の範囲
N = 100
vr_values = np.linspace(-v_max, v_max, N)
vl_values = np.linspace(-v_max, v_max, N)
dr_values = np.append(np.linspace(-delta_max, delta_max, N), 0)
dl_values = np.append(np.linspace(-delta_max, delta_max, N), 0)

# vr, vlの全組み合わせのVを事前計算
vr_mesh, vl_mesh = np.meshgrid(vr_values, vl_values)
V_values = (vr_mesh + vl_mesh) / 2

# グラフを描画する
plt.figure(figsize=(8, 6))

# リストを用意して、プロットする点を保存する
points_zero = []  # dr == dl == 0 の場合の点
points_else = []  # その他の条件の場合の点

i = 0
for dr in dr_values:
    for dl in dl_values:
        i += 1
        print(f"loop {i} / {(N+1)**2}")

        if dr == 0 and dl == 0:
            # dr == dl == 0 の場合
            omega_values = (vr_mesh - vl_mesh) / L
            points_zero.append((V_values, omega_values))  # 青のプロット
        elif dr == dl:
            continue
        elif (dr < 0 and 0 < dl) or (dr > 0 and 0 > dl):
            continue
        else:
            # それ以外の場合
            Rl = np.sin(abs(dr)) * L / np.sin(abs(dl - dr))
            Rr = np.sin(abs(dl)) * L / np.sin(abs(dr - dl))
            omega_values = (vr_mesh - vl_mesh) / abs(Rr - Rl)
            points_else.append((V_values, omega_values))  # 赤のプロット

# リストに保存した結果を一括でプロット
j = 0
k = 0
for V, omega in points_else:
    plt.scatter(V, omega, color='r', s=1, label="steering robot" if j == 0 else "")
    j+=1
for V, omega in points_zero:
    plt.scatter(V, omega, color='b', s=1, label="NOT steering robot" if k == 0 else "")
    k+=1


# プロットの設定
# plt.title("Region where valid v_r and v_l exist for given V and ω")
plt.xlabel("Forward Velocity V[m/s]", fontsize=15)
plt.ylabel("Angular Velocity ω[rad/s]", fontsize=15)
#label size up
plt.tick_params(labelsize=10)
plt.grid(True)
plt.legend(loc="upper right")
plt.show()