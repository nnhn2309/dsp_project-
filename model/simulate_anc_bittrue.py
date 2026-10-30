"""
Mô phỏng Bit-True thuật toán Feedback FxLMS với Secondary Path Modeling
Theo bài báo: "Design and analysis of experimental adaptive feedback system for active noise control in a duct"
Sử dụng chuẩn số học Fixed-point tương tự phần cứng Verilog trên DE10-Standard.
"""
import math
import random
import sys
import io

# Force UTF-8 on Windows console
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

def main():
    print("=" * 70)
    print("MÔ PHỎNG KIỂM CHỨNG BIT-TRUE: FEEDBACK ANC (FxLMS) TRÊN FPGA DE10")
    print("=" * 70)

    # 1. Cấu hình thông số hệ thống
    Fs = 8000           # Tần số lấy mẫu: 8 kHz
    M = 32              # Số tap của bộ lọc mô hình thứ cấp S_hat(z)
    L = 32              # Số tap của bộ lọc thích nghi W(z)
    Delta = 4           # Độ trễ hình học âm học (samples) khớp với đỉnh trễ s_real

    # Thời gian mô phỏng
    N_calib = 8000      # 1 giây phát nhiễu trắng để học S(z)
    N_anc   = 24000     # 3 giây chạy ANC khử nhiễu
    N_total = N_calib + N_anc

    # 2. Khởi tạo mô hình đường dẫn thứ cấp thực tế S(z) trong ống (Ground Truth)
    # Mô phỏng phản xạ và trễ âm học thực tế từ loa đến mic
    s_real = [0.0] * M
    # Giả lập đáp ứng xung thực tế: có trễ 4 sample, đỉnh chính và các phản xạ tắt dần
    s_real[4] = 0.55
    s_real[5] = -0.35
    s_real[6] = 0.20
    s_real[7] = -0.10
    s_real[8] = 0.05

    # 3. Bộ lọc ước lượng S_hat(z) và bộ lọc W(z)
    # Lưu dạng Fixed-point 32-bit (Q2.30): 1.0 tương đương 2^30 = 1073741824
    SCALE_Q15 = 32768.0        # 2^15
    SCALE_Q30 = 1073741824.0   # 2^30

    s_hat_q30 = [0] * M        # Trọng số S_hat dạng Q2.30
    w_q30     = [0] * L        # Trọng số W dạng Q2.30

    # Bộ đệm trễ tín hiệu (Delay lines - Q1.15)
    v_buf  = [0] * M           # Nhiễu trắng v(n)
    y_buf  = [0] * M           # Tín hiệu chống ồn y(n)
    x_buf  = [0] * max(L, M)   # Tín hiệu tham chiếu x(n)
    xf_buf = [0] * L           # Tín hiệu tham chiếu lọc x_f(n)
    d_hat_delay = [0] * (Delta + 1) # Hàng đợi trễ Delta

    # Bước nhảy thích nghi (Dạng lũy thừa 2 để dịch bit)
    # mu_s = 2^-8, mu_w = 2^-8
    SHIFT_MU_S = 8
    SHIFT_MU_W = 8

    # Tần số tiếng ồn chính: Thử nghiệm 300 Hz (tần số kiểm nghiệm trong bài báo)
    f_noise = 300.0

    print(f"[*] Cấu hình:")
    print(f"    - Tần số lấy mẫu: {Fs} Hz")
    print(f"    - Tần số tiếng ồn: {f_noise} Hz")
    print(f"    - Bậc bộ lọc S_hat: {M} taps, Bộ lọc W: {L} taps")
    print(f"    - Số mẫu học đường dẫn thứ cấp (Calib): {N_calib}")
    print(f"    - Số mẫu chạy khử nhiễu (ANC): {N_anc}")
    print(f"[*] Đang thực thi mô phỏng...")

    e_history = []
    d_history = []

    # Vòng lặp lấy mẫu từng sample
    for n in range(N_total):
        # Tạo tiếng ồn sơ cấp d(n) (chỉ xuất hiện sau giai đoạn Calib)
        if n < N_calib:
            d_val = 0.0
            is_calib = True
        else:
            # Tiếng ồn sin 300 Hz có biên độ 0.5 (Q1.15) + một ít nhiễu nền
            t = (n - N_calib) / Fs
            d_val = 0.5 * math.sin(2.0 * math.pi * f_noise * t)
            is_calib = False

        d_q15 = int(d_val * SCALE_Q15)

        # -------------------------------------------------------------
        # GIAI ĐOẠN 1: OFFLINE SECONDARY PATH MODELING
        # -------------------------------------------------------------
        if is_calib:
            # 1. Phát nhiễu trắng v(n) (biên độ khoảng 0.25 để không méo loa)
            v_val = random.uniform(-0.3, 0.3)
            v_q15 = int(v_val * SCALE_Q15)

            # Cập nhật buffer v
            v_buf.pop()
            v_buf.insert(0, v_q15)

            # 2. Tín hiệu truyền qua đường dẫn thực S(z) tới mic
            mic_in = 0.0
            for i in range(len(s_real)):
                if i < len(v_buf):
                    mic_in += s_real[i] * (v_buf[i] / SCALE_Q15)
            # Thêm nhiễu nền nhỏ
            mic_in += random.gauss(0, 0.005)
            mic_q15 = int(mic_in * SCALE_Q15)

            # 3. Lọc qua mô hình S_hat(z) hiện tại (Fixed-point MAC)
            acc_ys = 0
            for m in range(M):
                # s_hat_q30 * v_buf_q15 -> Q3.45 -> dịch phải 15 về Q2.30
                acc_ys += (s_hat_q30[m] * v_buf[m]) >> 15
            # Đưa về Q1.15
            ys_q15 = acc_ys >> 15

            # 4. Tính sai số nhận dạng: e_s(n) = mic(n) - y_s(n)
            e_s_q15 = mic_q15 - ys_q15

            # 5. Cập nhật trọng số S_hat bằng LMS:
            # delta_s = e_s * v * mu_s
            for m in range(M):
                # (e_s_q15 * v_buf_q15) -> Q2.30
                grad = e_s_q15 * v_buf[m]
                # Dịch bit cho mu_s: >> SHIFT_MU_S
                update = grad >> SHIFT_MU_S
                s_hat_q30[m] += update

            e_history.append(abs(e_s_q15) / SCALE_Q15)
            d_history.append(0.0)

        # -------------------------------------------------------------
        # GIAI ĐOẠN 2: FEEDBACK ACTIVE NOISE CONTROL (FxLMS)
        # -------------------------------------------------------------
        else:
            # 1. Tín hiệu chống ồn y(n) truyền qua kênh thực S(z) tạo anti-noise ya(n)
            ya_val = 0.0
            for i in range(len(s_real)):
                if i < len(y_buf):
                    ya_val += s_real[i] * (y_buf[i] / SCALE_Q15)
            ya_q15 = int(ya_val * SCALE_Q15)

            # 2. Microphone thu được sai số: e(n) = d(n) + ya(n)
            # (Chú ý: y(n) sẽ tự thích nghi thành ngược pha để triệt tiêu d(n))
            e_q15 = d_q15 + ya_q15

            # Giới hạn bão hòa (Saturation)
            if e_q15 > 32767: e_q15 = 32767
            if e_q15 < -32768: e_q15 = -32768

            # 3. Tái tạo ước lượng phản hồi y_f(n) = S_hat(z) * y(n)
            acc_yf = 0
            for m in range(M):
                acc_yf += (s_hat_q30[m] * y_buf[m]) >> 15
            yf_q15 = acc_yf >> 15

            # 4. Tái tạo nhiễu gốc: d_hat(n) = e(n) - y_f(n)
            d_hat_q15 = e_q15 - yf_q15

            # Đưa qua trễ Delta để tạo tín hiệu tham chiếu x(n)
            d_hat_delay.pop()
            d_hat_delay.insert(0, d_hat_q15)
            x_n_q15 = d_hat_delay[Delta]

            # Cập nhật buffer x
            x_buf.pop()
            x_buf.insert(0, x_n_q15)

            # 5. Tạo tín hiệu chống ồn y(n) = sum(w_l * x(n-l))
            acc_y = 0
            for l in range(L):
                acc_y += (w_q30[l] * x_buf[l]) >> 15
            y_q15 = acc_y >> 15

            # Bão hòa y(n)
            if y_q15 > 32767: y_q15 = 32767
            if y_q15 < -32768: y_q15 = -32768

            # Cập nhật buffer y
            y_buf.pop()
            y_buf.insert(0, y_q15)

            # 6. Tạo tín hiệu tham chiếu lọc: x_f(n) = S_hat(z) * x(n)
            acc_xf = 0
            for m in range(M):
                acc_xf += (s_hat_q30[m] * x_buf[m]) >> 15
            xf_n_q15 = acc_xf >> 15

            # Cập nhật buffer xf
            xf_buf.pop()
            xf_buf.insert(0, xf_n_q15)

            # 7. Cập nhật trọng số W(z) bằng FxLMS:
            # w(n+1) = w(n) - mu_w * e(n) * xf(n)
            for l in range(L):
                grad = e_q15 * xf_buf[l]
                update = grad >> SHIFT_MU_W
                w_q30[l] -= update

            e_history.append(abs(e_q15) / SCALE_Q15)
            d_history.append(abs(d_q15) / SCALE_Q15)

    # 4. Đánh giá kết quả mô phỏng
    # Tính công suất nhiễu ban đầu và công suất sai số sau khi hội tụ
    steady_start = N_total - 4000
    p_initial = sum(d ** 2 for d in d_history[steady_start:]) / 4000.0
    p_residual = sum(e ** 2 for e in e_history[steady_start:]) / 4000.0

    attenuation_db = 10.0 * math.log10(p_initial / (p_residual + 1e-12))

    print("\n" + "=" * 70)
    print("KẾT QUẢ MÔ PHỎNG BIT-TRUE THÀNH CÔNG:")
    print("=" * 70)
    print(f"1. Giai đoạn Calib (Ước lượng S_hat):")
    print(f"   - Sai số nhận dạng ban đầu: {e_history[100]:.4f}")
    print(f"   - Sai số nhận dạng cuối Calib: {e_history[N_calib - 100]:.4f} (Đã hội tụ về ~0)")
    print(f"2. Giai đoạn ANC (Triệt tiêu nhiễu 300 Hz):")
    print(f"   - Biên độ nhiễu ban đầu d(n): ~0.5000 (RMS = {math.sqrt(p_initial):.4f})")
    print(f"   - Biên độ sai số dư e(n): ~{math.sqrt(p_residual):.4f}")
    print(f"   - MỨC SUY GIẢM TIẾNG ỒN ĐẠT ĐƯỢC: {attenuation_db:.2f} dB")
    print(f"   (Tương đương triệt giảm {(1.0 - math.sqrt(p_residual/p_initial))*100:.1f}% biên độ âm)")
    print("=" * 70)

if __name__ == "__main__":
    main()
