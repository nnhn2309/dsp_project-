"""
Công cụ chuyển đổi âm thanh giữa WAV và File Text cho Testbench Verilog:
1. wav_to_txt: Chuyển đổi file .wav thành file .txt (chứa các mẫu 16-bit signed) để Verilog $fscanf đọc.
2. txt_to_wav: Chuyển đổi file .txt kết quả từ Verilog testbench ngược lại thành file .wav để NGHE THỬ.
"""
import wave
import struct
import os
import sys
import io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

def wav_to_txt(wav_path, txt_path, target_fs=8000, max_samples=16000):
    print(f"[*] Đang đọc file WAV: {wav_path}")
    with wave.open(wav_path, 'r') as wf:
        n_channels = wf.getnchannels()
        sampwidth  = wf.getsampwidth()
        fs         = wf.getframerate()
        n_frames   = wf.getnframes()
        raw_bytes  = wf.readframes(n_frames)

    # Đọc mẫu 16-bit
    step = n_channels * sampwidth
    samples = []
    for i in range(0, len(raw_bytes), step):
        val = struct.unpack('<h', raw_bytes[i:i+2])[0]
        samples.append(val)

    # Đơn giản hóa downsampling nếu fs != target_fs
    ratio = fs / target_fs
    resampled = []
    idx = 0.0
    while int(idx) < len(samples) and len(resampled) < max_samples:
        resampled.append(samples[int(idx)])
        idx += ratio

    print(f"[*] Đang xuất {len(resampled)} mẫu sang file text: {txt_path}")
    with open(txt_path, 'w') as f:
        for s in resampled:
            f.write(f"{s}\n")
    print(f"[OK] Đã xuất thành công: {txt_path}")

def txt_to_wav(txt_path, wav_path, fs=8000):
    if not os.path.exists(txt_path):
        print(f"[!] File không tồn tại: {txt_path}")
        return
    print(f"[*] Đang chuyển đổi kết quả từ Testbench {txt_path} thành file WAV để nghe...")
    samples = []
    with open(txt_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    val = int(line)
                    # Kẹp ngưỡng 16-bit
                    val = max(-32768, min(32767, val))
                    samples.append(val)
                except ValueError:
                    continue

    with wave.open(wav_path, 'w') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(fs)
        raw_bytes = bytearray()
        for s in samples:
            raw_bytes.extend(struct.pack('<h', s))
        wf.writeframes(raw_bytes)
    print(f"[OK] Đã tạo xong file âm thanh sau lọc: {wav_path}")

if __name__ == "__main__":
    base_dir = r"E:\dsp_doan\tb"
    wav_in = r"E:\dsp_doan\model\noise_samples\noise_300hz.wav"
    txt_out = os.path.join(base_dir, "noise_input.txt")
    wav_to_txt(wav_in, txt_out, target_fs=8000, max_samples=24000)
