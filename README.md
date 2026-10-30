# HỆ THỐNG ACTIVE NOISE CONTROL (FEEDBACK FxLMS) TRÊN FPGA DE10-STANDARD

Dự án thiết kế phần cứng hoàn chỉnh bằng **Verilog HDL** cho hệ thống Khử nhiễu tích cực (**Feedback ANC**) kết hợp ước lượng đường dẫn thứ cấp bằng nhiễu trắng (**Secondary Path Modeling**), dựa trên bài báo khoa học:
> *"Design and analysis of experimental adaptive feedback system for active noise control in a duct"* (Maja Anachkova et al., 2023).

---

## 1. Cấu Trúc Thư Mục Dự Án

```
dsp_doan/
├── rtl/                          # Toàn bộ mã nguồn Verilog tổng hợp được
│   ├── de10_standard_anc_top.v   # Top-level entity kết nối phần cứng DE10
│   ├── feedback_fxlms_core.v     # Lõi xử lý DSP (LMS Calib & FxLMS ANC)
│   ├── i2s_transceiver.v         # Giao tiếp âm thanh I2S 16-bit
│   ├── wm8731_i2c_config.v       # Cấu hình thanh ghi WM8731 (Mic In & Line Out)
│   ├── lfsr_prbs31.v             # Bộ tạo nhiễu trắng LFSR PRBS-31
│   └── hex_decoder.v             # Giải mã trạng thái lên 6 LED 7-đoạn
├── tb/                           # Testbench mô phỏng chức năng
│   └── tb_feedback_fxlms_core.v  # Testbench mô phỏng cả âm học ống dẫn thực tế
├── syn/                          # Dự án Quartus Prime & File gán chân
│   ├── de10_standard_anc.qpf     # Project file Quartus
│   ├── de10_standard_anc.qsf     # Gán chân chi tiết cho Cyclone V 5CSXFC6D6F31C6N
│   └── de10_standard_anc.sdc     # Ràng buộc thời gian SDC 50 MHz
├── model/                        # Mô hình toán học Bit-True
│   └── simulate_anc_bittrue.py   # Script Python mô phỏng số học dấu chấm tĩnh
└── README.md                     # Tài liệu hướng dẫn sử dụng
```

---

## 2. Hướng Dẫn Kết Nối Phần Cứng (DE10-Standard)

1. **Cổng Mic In (Jack màu HỒNG - Pink)**:
   - Cắm microphone đo sai số $e(n)$ (Error Microphone) đặt tại vùng cần yên tĩnh ("quiet zone") trong ống dẫn.
   - *Lưu ý*: Mạch bên trong FPGA đã tự động kích hoạt mạch tiền khuếch đại **+20dB Mic Boost** của chip Wolfson WM8731.
2. **Cổng Line Out (Jack màu XANH LÁ - Green)**:
   - Cắm jack 3.5mm nối ra **Loa mini chống ồn** (Control Speaker).
3. **Loa phát tiếng ồn chính (Primary Speaker)**:
   - Đặt ở đầu ống dẫn, phát tiếng ồn tần số thấp ($200\text{ Hz} - 400\text{ Hz}$) từ điện thoại hoặc máy phát hàm.

---

## 3. Quy Trình Vận Hành Trên Board

| Thao tác | Phần cứng | Ý nghĩa hoạt động |
| :--- | :--- | :--- |
| **Bước 1: Khởi động** | Nhấn `KEY[0]` | Reset hệ thống. Chip FPGA sẽ gửi I2C để bật CODEC WM8731. Đèn `LEDR[0]` sáng báo I2C cấu hình xong. |
| **Bước 2: Học đường dẫn** | Nhấn `KEY[1]` | Bắt đầu pha **Calib**: Loa mini phát tiếng xì xì (Nhiễu trắng). Thuật toán LMS ước lượng mô hình $\hat{S}(z)$. LED 7-đoạn hiện `"CALIb"`. Sau 1 giây, còi dừng, LED hiện `"  dOnE"` và `LEDR[2]` sáng. |
| **Bước 3: Bật khử tiếng ồn** | Gạt `SW[0]` lên `1` | Bật chế độ **ANC Active**. Bộ lọc FxLMS phát sóng âm ngược pha $y(n)$ ra loa mini. LED 7-đoạn hiện `"ANC On"`. Đèn VU-meter (`LEDR[9:4]`) giảm sáng rõ rệt báo hiệu tiếng ồn đã được triệt giảm. |
| **Bước 4: So sánh Bypass** | Gạt `SW[0]` xuống `0` | Chế độ **Bypass**: Loa mini ngắt âm để đo kiểm mức ồn đối chứng. |
| **Tinh chỉnh bước nhảy $\mu$** | Gạt `SW[2:1]` | `00`: $\mu = 2^{-8}$, `01`: $\mu = 2^{-9}$, `10`: $\mu = 2^{-10}$, `11`: $\mu = 2^{-11}$. |

---

## 4. Biên Dịch và Nạp Board Bằng Quartus Prime

1. Mở phần mềm **Quartus Prime** (bản Lite hoặc Standard).
2. Vào **File** $\to$ **Open Project...** $\to$ Chọn file `syn/de10_standard_anc.qpf`.
3. Nhấn tổ hợp phím **Ctrl + L** (hoặc nút **Start Compilation**) để biên dịch toàn bộ dự án.
4. Mở công cụ **Programmer** (Tools $\to$ Programmer):
   - Cắm cáp USB-Blaster II từ máy tính vào cổng USB Blaster của DE10-Standard.
   - Chọn file `output_files/de10_standard_anc_top.sof`.
   - Nhấn **Start** để nạp trực tiếp vào FPGA.

---

## 5. Chạy Mô Phỏng Bit-True (Python)

Bạn có thể chạy script mô phỏng độc lập trên máy tính bất kỳ lúc nào để khảo sát đáp ứng:
```bash
python model/simulate_anc_bittrue.py
```
Kết quả kiểm chứng đạt mức suy giảm tiếng ồn **$\approx 10.76\text{ dB}$** ở tần số $300\text{ Hz}$ (triệt giảm $71\%$ biên độ sóng âm), hoàn toàn khớp với thực nghiệm trong bài báo.
