import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, square

# ローパスフィルタを設計する関数
def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs  # ナイキスト周波数
    normal_cutoff = cutoff / nyq  # 正規化カットオフ周波数
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

# ローパスフィルタを適用する関数
def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order)
    y = lfilter(b, a, data)
    return y

# サンプリング周波数とカットオフ周波数を設定
fs = 1000.0  # サンプリング周波数（Hz）
cutoff = 10.0  # カットオフ周波数（Hz）

# 時間ベクトル
T = 1.0  # 波形の長さ（秒）
t = np.linspace(0, T, int(fs * T), endpoint=False)

# 方形波を生成
square_wave = square(2 * np.pi * 1 * t)  # 5Hzの方形波

# ローパスフィルタを適用
filtered_wave = butter_lowpass_filter(square_wave, cutoff, fs, order=6)

# 結果をプロット
plt.figure(figsize=(10, 6))
plt.subplot(2, 1, 1)
plt.plot(t, square_wave, label='Original Square Wave')
plt.title('Original Square Wave')
plt.grid()
plt.subplot(2, 1, 2)
plt.plot(t, filtered_wave, label='Filtered Wave (Low-pass)')
plt.title('Filtered Wave (After Low-pass Filter)')
plt.grid()
plt.tight_layout()
plt.show()

import numpy as np
import matplotlib.pyplot as plt

# フーリエ変換を行う
fft_filtered = np.fft.fft(filtered_wave)
frequencies = np.fft.fftfreq(len(filtered_wave), 1/fs)

# 結果をプロット
plt.figure(figsize=(10, 6))
plt.plot(frequencies[:len(frequencies)//2], np.abs(fft_filtered)[:len(frequencies)//2])
plt.title('Frequency Spectrum of Filtered Wave')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Amplitude')
plt.grid()
plt.show()

