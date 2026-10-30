// ============================================================================
// Module: de10_standard_anc_top.v
// Dự án: Active Noise Control (ANC) Feedback FxLMS trên DE10-Standard
// Chip FPGA: Intel/Altera Cyclone V SX SoC (5CSXFC6D6F31C6N)
// Ngoại vi sử dụng:
//   - Wolfson WM8731 Audio CODEC:
//       + Jack Hồng (Mic In): Thu tín hiệu sai số e(n) từ Error Microphone
//       + Jack Xanh Lá (Line Out): Xuất sóng chống ồn y(n) ra Loa Mini
//   - Nút bấm KEY[0]: Reset hệ thống (Tích cực thấp)
//   - Nút bấm KEY[1]: Nhấn để bắt đầu quá trình Học Đường dẫn Thứ cấp (Calib S_hat)
//   - Công tắc SW[0]: Gạt lên (1) để Bật khử ồn ANC, gạt xuống (0) để Tắt (Bypass)
//   - Công tắc SW[2:1]: Chọn bước nhảy mu (00: 2^-8, 01: 2^-9, 10: 2^-10, 11: 2^-11)
//   - LEDR[0]: Báo I2C đã cấu hình xong CODEC
//   - LEDR[1]: Báo đang chạy Calib
//   - LEDR[2]: Báo đã hoàn thành Calib
//   - LEDR[3]: Báo ANC đang hoạt động
//   - LEDR[9:4]: Đồng hồ đo mức năng lượng âm thanh (VU-Meter)
//   - HEX5..HEX0: Hiển thị trạng thái hệ thống ("CALIb ", " dOnE", "ANC On")
// ============================================================================

module de10_standard_anc_top (
    // Xung nhịp hệ thống
    input  wire        CLOCK_50,        // 50 MHz Oscillator (PIN_AF14)

    // Phím bấm & Công tắc gạt
    input  wire [3:0]  KEY,             // KEY[0]: Reset, KEY[1]: Calib
    input  wire [9:0]  SW,              // SW[0]: ANC On/Off, SW[2:1]: Mu Sel

    // Đèn LED hiển thị
    output wire [9:0]  LEDR,            // Trạng thái và VU-meter
    output wire [6:0]  HEX0,
    output wire [6:0]  HEX1,
    output wire [6:0]  HEX2,
    output wire [6:0]  HEX3,
    output wire [6:0]  HEX4,
    output wire [6:0]  HEX5,

    // Giao tiếp với Audio CODEC WM8731
    input  wire        AUD_BCLK,        // Bit clock từ Codec (PIN_AF30)
    input  wire        AUD_ADCLRCK,     // ADC Frame clock (PIN_AJ29)
    input  wire        AUD_ADCDAT,      // ADC Serial Data từ Mic (PIN_AH29)
    input  wire        AUD_DACLRCK,     // DAC Frame clock (PIN_AG30)
    output wire        AUD_DACDAT,      // DAC Serial Data ra Loa (PIN_AF29)
    output reg         AUD_XCK,         // Audio Master Clock cấp cho WM8731 (PIN_AH30)

    // Giao tiếp I2C cấu hình CODEC
    output wire        FPGA_I2C_SCLK,   // I2C Clock (PIN_Y24)
    inout  wire        FPGA_I2C_SDAT    // I2C Data (PIN_Y23)
);

    // ========================================================================
    // 1. Đồng bộ và Xử lý Tín hiệu Điều khiển
    // ========================================================================
    wire sys_reset_n = KEY[0]; // KEY tích cực mức thấp (Nhấn = 0)

    // Xử lý nút bấm KEY[1] (Calib Start) với bộ lọc chống rung (Debounce & One-shot)
    reg [1:0] key1_sync;
    reg [19:0] key1_debounce_cnt;
    reg        key1_pressed;
    reg        key1_prev;
    wire       calib_start_pulse;

    always @(posedge CLOCK_50 or negedge sys_reset_n) begin
        if (!sys_reset_n) begin
            key1_sync         <= 2'b11;
            key1_debounce_cnt <= 20'd0;
            key1_pressed      <= 1'b0;
            key1_prev         <= 1'b0;
        end else begin
            key1_sync <= {key1_sync[0], KEY[1]};
            if (key1_sync[1] == 1'b0) begin // Đang nhấn phím (mức 0)
                if (key1_debounce_cnt < 20'd500_000) // Lọc rung ~10ms
                    key1_debounce_cnt <= key1_debounce_cnt + 1'b1;
                else
                    key1_pressed <= 1'b1;
            end else begin
                key1_debounce_cnt <= 20'd0;
                key1_pressed      <= 1'b0;
            end
            key1_prev <= key1_pressed;
        end
    end
    assign calib_start_pulse = (key1_pressed && !key1_prev);

    // ========================================================================
    // 2. Tạo xung Master Clock AUD_XCK ~ 12.5 MHz cho WM8731
    // ========================================================================
    // Chia tần số 50 MHz / 4 = 12.5 MHz (Phù hợp chế độ USB Mode của WM8731)
    reg [1:0] xck_div;
    always @(posedge CLOCK_50 or negedge sys_reset_n) begin
        if (!sys_reset_n) begin
            xck_div <= 2'd0;
            AUD_XCK <= 1'b0;
        end else begin
            xck_div <= xck_div + 1'b1;
            AUD_XCK <= xck_div[1]; // Đảo trạng thái ở mỗi 2 chu kỳ 50MHz
        end
    end

    // ========================================================================
    // 3. Khối Cấu hình I2C cho WM8731 (Bật Mic In +20dB Boost, Line Out 0dB)
    // ========================================================================
    wire i2c_config_done;

    wm8731_i2c_config u_i2c_config (
        .clk         (CLOCK_50),
        .reset_n     (sys_reset_n),
        .i2c_sclk    (FPGA_I2C_SCLK),
        .i2c_sdat    (FPGA_I2C_SDAT),
        .config_done (i2c_config_done)
    );

    // ========================================================================
    // 4. Khối Giao tiếp I2S Thu/Phát Audio 16-bit
    // ========================================================================
    wire               sample_valid;
    wire signed [15:0] mic_data_q15;
    wire signed [15:0] spk_data_q15;

    i2s_transceiver u_i2s_io (
        .clk          (CLOCK_50),
        .reset_n      (sys_reset_n && i2c_config_done),
        .aud_bclk     (AUD_BCLK),
        .aud_adclrck  (AUD_ADCLRCK),
        .aud_adcdat   (AUD_ADCDAT),
        .aud_daclrck  (AUD_DACLRCK),
        .aud_dacdat   (AUD_DACDAT),
        .sample_valid (sample_valid),
        .mic_data     (mic_data_q15),
        .spk_data     (spk_data_q15)
    );

    // ========================================================================
    // 5. Bộ phát Nhiễu Trắng PRBS-31 LFSR
    // ========================================================================
    wire signed [15:0] white_noise;

    lfsr_prbs31 u_lfsr (
        .clk       (CLOCK_50),
        .reset_n   (sys_reset_n),
        .enable    (sample_valid),
        .noise_out (white_noise)
    );

    // ========================================================================
    // 6. Lõi Xử lý DSP: Feedback FxLMS Core
    // ========================================================================
    wire calib_busy;
    wire calib_ready;
    wire anc_active;

    feedback_fxlms_core #(
        .M_TAPS         (32),
        .L_TAPS         (32),
        .DELTA          (4),
        .CALIB_SAMPLES  (8000)
    ) u_anc_core (
        .clk             (CLOCK_50),
        .reset_n         (sys_reset_n && i2c_config_done),
        .btn_calib_start (calib_start_pulse),
        .sw_anc_enable   (SW[0]),
        .sw_mu_sel       (SW[2:1]),
        .sample_valid    (sample_valid),
        .mic_in          (mic_data_q15),
        .spk_out         (spk_data_q15),
        .white_noise     (white_noise),
        .calib_busy      (calib_busy),
        .calib_ready     (calib_ready),
        .anc_active      (anc_active)
    );

    // ========================================================================
    // 7. Đồng hồ đo Mức Năng lượng Âm thanh (VU-Meter) trên LEDR[9:4]
    // ========================================================================
    // Ước lượng năng lượng trung bình |e(n)| để người dùng thấy rõ đèn tắt bớt khi ANC hoạt động
    reg [23:0] env_acc;
    reg [5:0]  vu_leds;
    wire [15:0] abs_mic = mic_data_q15[15] ? -mic_data_q15 : mic_data_q15;

    always @(posedge CLOCK_50 or negedge sys_reset_n) begin
        if (!sys_reset_n) begin
            env_acc <= 24'd0;
            vu_leds <= 6'b000000;
        end else if (sample_valid) begin
            // Bộ lọc tích lũy bao hình đơn giản: env = env * 0.95 + abs_mic * 0.05
            env_acc <= env_acc - (env_acc >> 5) + {8'd0, abs_mic};
            vu_leds[0] <= (env_acc[23:14] > 10'd50);
            vu_leds[1] <= (env_acc[23:14] > 10'd150);
            vu_leds[2] <= (env_acc[23:14] > 10'd350);
            vu_leds[3] <= (env_acc[23:14] > 10'd700);
            vu_leds[4] <= (env_acc[23:14] > 10'd1400);
            vu_leds[5] <= (env_acc[23:14] > 10'd2500);
        end
    end

    // Gán các LED chỉ thị trạng thái
    assign LEDR[0]   = i2c_config_done; // Sáng khi I2C nạp xong chip âm thanh
    assign LEDR[1]   = calib_busy;      // Sáng khi đang phát nhiễu học S(z)
    assign LEDR[2]   = calib_ready;     // Sáng khi đã có mô hình S_hat
    assign LEDR[3]   = anc_active;      // Sáng khi hệ thống đang khử nhiễu
    assign LEDR[9:4] = vu_leds;         // Đèn VU-meter âm lượng

    // ========================================================================
    // 8. Hiển thị Trạng thái lên 6 LED 7-Đoạn (HEX5..HEX0)
    // ========================================================================
    hex_decoder u_hex (
        .clk         (CLOCK_50),
        .calib_busy  (calib_busy),
        .calib_ready (calib_ready),
        .anc_active  (anc_active),
        .mu_sel      (SW[2:1]),
        .hex0        (HEX0),
        .hex1        (HEX1),
        .hex2        (HEX2),
        .hex3        (HEX3),
        .hex4        (HEX4),
        .hex5        (HEX5)
    );

endmodule
