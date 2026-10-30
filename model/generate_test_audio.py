"""
Script tự động tạo các file âm thanh hình sin chuẩn (WAV) ở tần số 200 Hz, 300 Hz, 400 Hz
để làm nguồn phát tiếng ồn sơ cấp (Primary Noise) cho dàn thí nghiệm ống dẫn.
"""
import wave
import struct
import math
import os
import sys
import io

# Force UTF-8 on Windows console
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

def generate_tone(filename, freq_hz, duration_sec=30, sample_rate=44100, amplitude=0.7):
    num_samples = int(duration_sec * sample_rate)
    print(f"[*] Đang tạo file: {filename} (Tần số = {freq_hz} Hz, Thời lượng = {duration_sec}s)...")
    
    with wave.open(filename, 'w') as wav_file:
        # Cấu hình: 1 channel (Mono), 2 bytes/sample (16-bit), samplerate 44.1kHz
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(sample_rate)
        
        raw_data = bytearray()
        for n in range(num_samples):
            t = n / sample_rate
            sample_val = amplitude * math.sin(2.0 * math.pi * freq_hz * t)
            # Chuyển về số nguyên 16-bit signed (-32768 đến 32767)
            int_val = int(sample_val * 32767.0)
            raw_data.extend(struct.pack('<h', int_val))
            
        wav_file.writeframes(raw_data)
    print(f"    -> Đã tạo xong: {filename}")

if __name__ == "__main__":
    out_dir = r"E:\dsp_doan\model\noise_samples"
    os.makedirs(out_dir, exist_ok=True)
    
    generate_tone(os.path.join(out_dir, "noise_200hz.wav"), 200)
    generate_tone(os.path.join(out_dir, "noise_300hz.wav"), 300)
    generate_tone(os.path.join(out_dir, "noise_400hz.wav"), 400)
    print("\n[OK] Đã tạo đủ 3 file âm thanh tại: " + out_dir)
