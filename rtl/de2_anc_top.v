// ============================================================================
// Module: de2_anc_top.v
// Dự án: Active Noise Control (ANC) Feedback FxLMS trên DE2 (Cyclone II)
// Chip FPGA: Altera Cyclone II EP2C35F672C6N
//
// Phần cứng sử dụng:
//   - Wolfson WM8731 Audio CODEC (có sẵn trên DE2):
//       + Jack Hồng (Mic In): Thu tín hiệu sai số e(n) từ Error Microphone
//       + Jack Xanh Lá (Line Out): Xuất sóng chống ồn y(n) ra Loa Mini
//
// Giao diện điều khiển:
//   KEY[0]    (PIN_G26): Reset hệ thống  (Tích cực thấp)
//   KEY[1]    (PIN_N23): Nhấn để Calib học S_hat(z)
//   SW[0]     (PIN_N25): Bật / Tắt ANC (1 = ANC ON, 0 = Bypass)
//   SW[2:1]   (PIN_P25, PIN_N26): Chọn bước nhảy mu
//   LEDR[0]   : Sáng khi I2C cấu hình xong CODEC
//   LEDR[1]   : Sáng khi đang Calib (phát nhiễu trắng)
//   LEDR[2]   : Sáng khi Calib xong (có mô hình S_hat)
//   LEDR[3]   : Sáng khi ANC đang hoạt động
//   LEDR[9:4] : VU-meter mức năng lượng âm thanh tại microphone
//   HEX7..HEX0: Hiển thị trạng thái ("CALIb", "  dOnE", "ANC On")
// ============================================================================

module de2_anc_top (
    // Xung nhịp hệ thống
    input  wire        CLOCK_50,        // 50 MHz (PIN_N2)

    // Phím bấm & Công tắc gạt
    input  wire [3:0]  KEY,             // Active-low
    input  wire [17:0] SW,

    // Đèn LED
    output wire [17:0] LEDR,
    output wire [8:0]  LEDG,

    // LED 7-đoạn (DE2 có 8 màn hình: HEX0..HEX7, active-low)
    output wire [6:0]  HEX0,
    output wire [6:0]  HEX1,
    output wire [6:0]  HEX2,
    output wire [6:0]  HEX3,
    output wire [6:0]  HEX4,
    output wire [6:0]  HEX5,
    output wire [6:0]  HEX6,
    output wire [6:0]  HEX7,

    // Giao tiếp với Audio CODEC WM8731
    input  wire        AUD_BCLK,        // PIN_B4
    input  wire        AUD_ADCLRCK,     // PIN_C5
    input  wire        AUD_ADCDAT,      // PIN_B5
    input  wire        AUD_DACLRCK,     // PIN_C6
    output wire        AUD_DACDAT,      // PIN_A4
    output reg         AUD_XCK,         // PIN_A5

    // Giao tiếp I2C cấu hình CODEC (trên DE2 tên là I2C, không có FPGA_ prefix)
    output wire        I2C_SCLK,        // PIN_A6
    inout  wire        I2C_SDAT         // PIN_B6
);

    // ========================================================================
    // 1. Tín hiệu Reset & Debounce KEY[1] (Calib)
    // ========================================================================
    wire sys_reset_n = KEY[0]; // KEY tích cực thấp

    reg [2:0] key1_sync;
    reg [19:0] key1_cnt;
    reg        key1_pressed, key1_prev;
    wire       calib_pulse;

    always @(posedge CLOCK_50 or negedge sys_reset_n) begin
        if (!sys_reset_n) begin
            key1_sync    <= 3'b111;
            key1_cnt     <= 20'd0;
            key1_pressed <= 1'b0;
            key1_prev    <= 1'b0;
        end else begin
            key1_sync <= {key1_sync[1:0], KEY[1]};
            if (!key1_sync[1]) begin
                if (key1_cnt < 20'd500_000) key1_cnt <= key1_cnt + 1'b1;
                else                         key1_pressed <= 1'b1;
            end else begin
                key1_cnt     <= 20'd0;
                key1_pressed <= 1'b0;
            end
            key1_prev <= key1_pressed;
        end
    end
    assign calib_pulse = (key1_pressed && !key1_prev);

    // ========================================================================
    // 2. Tạo AUD_XCK ~ 12.5 MHz (50 MHz / 4)
    // ========================================================================
    reg [1:0] xck_div;
    always @(posedge CLOCK_50 or negedge sys_reset_n) begin
        if (!sys_reset_n) begin xck_div <= 2'd0; AUD_XCK <= 1'b0; end
        else              begin xck_div <= xck_div + 1'b1; AUD_XCK <= xck_div[1]; end
    end

    // ========================================================================
    // 3. I2C Config WM8731 (Mic In +20dB Boost, Line Out 0dB, I2S 16-bit 8kHz)
    // ========================================================================
    wire i2c_done;

    wm8731_i2c_config u_i2c (
        .clk         (CLOCK_50),
        .reset_n     (sys_reset_n),
        .i2c_sclk    (I2C_SCLK),
        .i2c_sdat    (I2C_SDAT),
        .config_done (i2c_done)
    );

    // ========================================================================
    // 4. I2S Transceiver 16-bit
    // ========================================================================
    wire               sample_valid;
    wire signed [15:0] mic_data;
    wire signed [15:0] spk_data;

    i2s_transceiver u_i2s (
        .clk          (CLOCK_50),
        .reset_n      (sys_reset_n && i2c_done),
        .aud_bclk     (AUD_BCLK),
        .aud_adclrck  (AUD_ADCLRCK),
        .aud_adcdat   (AUD_ADCDAT),
        .aud_daclrck  (AUD_DACLRCK),
        .aud_dacdat   (AUD_DACDAT),
        .sample_valid (sample_valid),
        .mic_data     (mic_data),
        .spk_data     (spk_data)
    );

    // ========================================================================
    // 5. LFSR Nhiễu Trắng PRBS-31
    // ========================================================================
    wire signed [15:0] white_noise;

    lfsr_prbs31 u_lfsr (
        .clk       (CLOCK_50),
        .reset_n   (sys_reset_n),
        .enable    (sample_valid),
        .noise_out (white_noise)
    );

    // ========================================================================
    // 6. Lõi ANC DSP: Feedback FxLMS
    // ========================================================================
    wire calib_busy, calib_ready, anc_active;

    feedback_fxlms_core #(
        .M_TAPS         (32),
        .L_TAPS         (32),
        .DELTA          (4),
        .CALIB_SAMPLES  (8000)
    ) u_anc (
        .clk             (CLOCK_50),
        .reset_n         (sys_reset_n && i2c_done),
        .btn_calib_start (calib_pulse),
        .sw_anc_enable   (SW[0]),
        .sw_mu_sel       (SW[2:1]),
        .sample_valid    (sample_valid),
        .mic_in          (mic_data),
        .spk_out         (spk_data),
        .white_noise     (white_noise),
        .calib_busy      (calib_busy),
        .calib_ready     (calib_ready),
        .anc_active      (anc_active)
    );

    // ========================================================================
    // 7. VU-Meter đo mức âm thanh (LEDR[9:4])
    // ========================================================================
    reg [23:0] env_acc;
    reg [5:0]  vu_leds;
    wire [15:0] abs_mic = mic_data[15] ? (~mic_data + 1'b1) : mic_data;

    always @(posedge CLOCK_50 or negedge sys_reset_n) begin
        if (!sys_reset_n) begin
            env_acc <= 24'd0; vu_leds <= 6'd0;
        end else if (sample_valid) begin
            env_acc    <= env_acc - (env_acc >> 5) + {8'd0, abs_mic};
            vu_leds[0] <= (env_acc[23:14] > 10'd50);
            vu_leds[1] <= (env_acc[23:14] > 10'd150);
            vu_leds[2] <= (env_acc[23:14] > 10'd350);
            vu_leds[3] <= (env_acc[23:14] > 10'd700);
            vu_leds[4] <= (env_acc[23:14] > 10'd1400);
            vu_leds[5] <= (env_acc[23:14] > 10'd2500);
        end
    end

    // Gán LED (DE2 có 18 LEDR)
    assign LEDR[0]   = i2c_done;
    assign LEDR[1]   = calib_busy;
    assign LEDR[2]   = calib_ready;
    assign LEDR[3]   = anc_active;
    assign LEDR[9:4] = vu_leds;
    assign LEDR[17:10] = 8'd0;  // Không dùng
    assign LEDG      = 9'd0;    // Không dùng LEDG xanh lá

    // ========================================================================
    // 8. LED 7-đoạn (DE2 có 8 màn: HEX7..HEX0)
    //    Hiển thị: "    ANC On" / "    CALIb " / "      dOnE"
    // ========================================================================
    wire [6:0] hex_seg0, hex_seg1, hex_seg2, hex_seg3, hex_seg4, hex_seg5;

    hex_decoder u_hex (
        .clk         (CLOCK_50),
        .calib_busy  (calib_busy),
        .calib_ready (calib_ready),
        .anc_active  (anc_active),
        .mu_sel      (SW[2:1]),
        .hex0        (hex_seg0),
        .hex1        (hex_seg1),
        .hex2        (hex_seg2),
        .hex3        (hex_seg3),
        .hex4        (hex_seg4),
        .hex5        (hex_seg5)
    );

    // HEX0..HEX5 từ module decoder
    assign HEX0 = hex_seg0;
    assign HEX1 = hex_seg1;
    assign HEX2 = hex_seg2;
    assign HEX3 = hex_seg3;
    assign HEX4 = hex_seg4;
    assign HEX5 = hex_seg5;
    // HEX6, HEX7 tắt hết (DE2 có thêm 2 màn so với DE10)
    assign HEX6 = 7'b111_1111;
    assign HEX7 = 7'b111_1111;

endmodule
