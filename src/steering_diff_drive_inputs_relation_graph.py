import numpy as np
import matplotlib.pyplot as plt

# 定数
L = 1.0  # 車輪間の距離
v_max = 10  # 車輪の最大速度
steer_max = 30*np.pi/180  # ステアリングの最大角度

# 前進速度と旋回速度の範囲
N = 30
V_values = np.linspace(-v_max, v_max, N)
omega_values = np.linspace(-2 * v_max / L, 2 * v_max / L, N)
# omega_values = np.linspace(-2 * v_max / (L/2*np.cos(steer_max)), 2 * v_max / (L/2*np.cos(steer_max)), 10)
steer_values = np.linspace(-steer_max, steer_max, N)

# グラフを描画する
fig = plt.figure(figsize=(8, 6))
# ax = fig.add_subplot(111, projection='3d')

def calc_vr_vl(V, omega, steer_left, steer_right):

    if(steer_left == steer_right == 0):
        v_r = V + (L * omega) / 2
        v_l = V - (L * omega) / 2
    elif(steer_left == steer_right and steer_left != 0):
        v_r = 100000
        v_l = 100000
    elif(steer_left < 0 and 0 < steer_right):
        v_r = 1000000
        v_l = 1000000
    elif(steer_right < 0 and 0 < steer_left):
        v_r = 1000000
        v_l = 1000000
    else:
        R_l = np.sin(abs(steer_right)) * L / np.sin(abs(steer_left - steer_right))
        R_r = np.sin(abs(steer_left)) * L / np.sin(abs(steer_right - steer_left))
        v_r = V + (abs(R_r - R_l) * omega) / 2
        v_l = V - (abs(R_r - R_l) * omega) / 2
    return v_r, v_l

# 満たす領域を計算してプロット
i=0
for V in V_values:
    for omega in omega_values:
        for steer in steer_values:
            i+=1
            R = V/omega
            steer_in = np.arctan(R*np.sin(steer)/(R*np.cos(steer)-L/2))
            steer_out = np.arctan(R*np.sin(steer)/(R*np.cos(steer)+L/2))
            if(steer_in < -steer_max or steer_in > steer_max or steer_out < -steer_max or steer_out > steer_max):
                continue

            if(omega > 0):
                steer_left = steer_in
                steer_right = steer_out
            else:
                steer_left = steer_out
                steer_right = steer_in

            v_r, v_l = calc_vr_vl(V, omega, steer_left, steer_right)
            if -v_max <= v_r <= v_max and -v_max <= v_l <= v_max and -steer_max <= steer_left <= steer_max and -steer_max <= steer_right <= steer_max:
                # ax.scatter(V, omega, steer, color='blue')
                plt.scatter(V, omega, color='blue', s=1)
                # ax.scatter(V, omega, steer, color='blue', s=1)
        # # v_r と v_l の計算
        # v_r = V + (L * omega) / 2
        # v_l = V - (L * omega) / 2
        
        # # v_r と v_l が -v_max <= v_r, v_l <= v_max を満たすか確認
        # if -v_max <= v_r <= v_max and -v_max <= v_l <= v_max:
        #     plt.scatter(V, omega, color='blue', s=1)  # 有効な点をプロット
            print("roop %d / %d" % (i, N**3))

# plt.title("Region where valid v_r and v_l exist for given V and ω")
plt.xlabel("Forward Velocity (V)")
plt.ylabel("Angular Velocity (ω)")
# plt.zlabel("Steering Angle")
plt.grid(True)
plt.show()
