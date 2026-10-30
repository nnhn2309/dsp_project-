// ============================================================================
// Module: feedback_fxlms_core.v
// Mục đích: Lõi xử lý DSP chính của hệ thống Active Noise Control (ANC)
// Thuật toán: Feedback FxLMS kết hợp Secondary Path Identification
// Cấu trúc phần cứng:
//   - Sequential MAC Unit (tính tuần tự trong ~150 chu kỳ clock 50MHz)
//   - Tín hiệu âm thanh: 16-bit Signed Q1.15
//   - Trọng số bộ lọc S_hat và W: 32-bit Signed Q2.30
//   - Bước nhảy mu_w, mu_s: Dịch bit số học (Arithmetic Right Shift)
//   - Bộ đệm bão hòa (Saturation Logic) chống hú còi khi tràn số
// ============================================================================

module feedback_fxlms_core #(
    parameter M_TAPS = 32,            // Số tap của mô hình đường dẫn thứ cấp S_hat(z)
    parameter L_TAPS = 32,            // Số tap của bộ lọc thích nghi W(z)
    parameter DELTA  = 4,             // Trễ truyền âm học trong ống (samples)
    parameter CALIB_SAMPLES = 8000    // 8000 mẫu lấy mẫu ở 8kHz = 1 giây Calib
)(
    input  wire        clk,           // Clock 50 MHz
    input  wire        reset_n,       // Reset tích cực thấp (KEY[0])

    // Điều khiển từ Switch & Nút bấm
    input  wire        btn_calib_start, // Nhấn nút KEY[1] để kích hoạt học S(z)
    input  wire        sw_anc_enable,   // SW[0]: 1 = Bật ANC, 0 = Tắt (Bypass)
    input  wire [1:0]  sw_mu_sel,       // SW[2:1]: Chọn bước nhảy mu (2^-8, 2^-9, 2^-10, 2^-11)

    // Giao tiếp với I2S Transceiver
    input  wire        sample_valid,    // Xung báo mẫu mới từ ADC (Mic In)
    input  wire signed [15:0] mic_in,   // Tín hiệu sai số e(n) từ Mic In (Q1.15)
    output reg  signed [15:0] spk_out,  // Tín hiệu phát ra Line Out tới loa mini (Q1.15)

    // Bộ phát nhiễu trắng LFSR
    input  wire signed [15:0] white_noise, // Tín hiệu từ module lfsr_prbs31

    // Trạng thái báo ra LED và hiển thị
    output reg         calib_busy,      // Đang trong quá trình Calib
    output reg         calib_ready,     // Đã có mô hình S_hat, sẵn sàng khử nhiễu
    output reg         anc_active       // Đang chạy khử nhiễu FxLMS
);

    // ========================================================================
    // 1. Khai báo Bộ nhớ Trọng số & Bộ đệm trễ tín hiệu
    // ========================================================================
    // Trọng số S_hat (M taps, 32-bit Q2.30)
    reg signed [31:0] s_hat_ram [0:M_TAPS-1];
    // Trọng số W (L taps, 32-bit Q2.30)
    reg signed [31:0] w_ram [0:L_TAPS-1];

    // Bộ đệm trễ tín hiệu (16-bit Q1.15)
    reg signed [15:0] v_buf [0:M_TAPS-1];   // Lịch sử nhiễu trắng v(n)
    reg signed [15:0] y_buf [0:M_TAPS-1];   // Lịch sử tín hiệu chống ồn y(n)
    reg signed [15:0] x_buf [0:L_TAPS-1];   // Lịch sử tín hiệu tham chiếu x(n)
    reg signed [15:0] xf_buf [0:L_TAPS-1];  // Lịch sử tín hiệu tham chiếu lọc x_f(n)
    reg signed [15:0] delay_d_buf [0:DELTA]; // Hàng đợi trễ Delta mẫu cho d_hat

    // Bước nhảy thích nghi tùy biến qua Switch SW[2:1]
    // 2'b00 -> shift 8 (mu = 2^-8)
    // 2'b01 -> shift 9 (mu = 2^-9)
    // 2'b10 -> shift 10 (mu = 2^-10)
    // 2'b11 -> shift 11 (mu = 2^-11)
    wire [4:0] shift_mu_w = (sw_mu_sel == 2'b00) ? 5'd8  :
                            (sw_mu_sel == 2'b01) ? 5'd9  :
                            (sw_mu_sel == 2'b10) ? 5'd10 : 5'd11;
    localparam [4:0] SHIFT_MU_S = 5'd8; // Bước nhảy cho học S_hat

    // ========================================================================
    // 2. Định nghĩa FSM Hệ thống
    // ========================================================================
    localparam STATE_IDLE        = 3'd0;
    localparam STATE_CALIB_RUN   = 3'd1;
    localparam STATE_CALIB_DONE  = 3'd2;
    localparam STATE_ANC_RUN     = 3'd3;
    localparam STATE_BYPASS      = 3'd4;

    reg [2:0] sys_state;
    reg [15:0] calib_counter;

    // FSM tính toán chi tiết của Sequential MAC Unit
    localparam CALC_IDLE       = 4'd0;
    // Các bước tính trong Calib:
    localparam CALC_YS_FIR     = 4'd1;
    localparam CALC_SHAT_UP    = 4'd2;
    // Các bước tính trong ANC:
    localparam CALC_YF_FIR     = 4'd3;
    localparam CALC_SYNTH_X    = 4'd4;
    localparam CALC_Y_FIR      = 4'd5;
    localparam CALC_XF_FIR     = 4'd6;
    localparam CALC_W_UP       = 4'd7;
    localparam CALC_DONE       = 4'd8;

    reg [3:0] calc_state;
    reg [6:0] tap_idx;

    // Các biến tích lũy trung gian (64-bit và 32-bit)
    reg signed [47:0] acc;
    reg signed [15:0] cur_e;
    reg signed [15:0] cur_yf;
    reg signed [15:0] cur_dhat;
    reg signed [15:0] cur_x;
    reg signed [15:0] cur_y;
    reg signed [15:0] cur_xf;

    // Biến tạm cho các bước tính toán (Khai báo ở module-level để tương thích chuẩn Verilog-2001 trên Vivado/Quartus)
    reg signed [15:0] tmp_ys_val;
    reg signed [15:0] tmp_e_s;
    reg signed [31:0] tmp_grad_s;
    reg signed [15:0] tmp_d_hat;
    reg signed [15:0] tmp_sat_y;
    reg signed [15:0] tmp_sat_xf;
    reg signed [31:0] tmp_grad_w;

    integer i;

    // ========================================================================
    // 3. Tiến trình Xử lý Chính (FSM & MAC Engine)
    // ========================================================================
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            sys_state       <= STATE_IDLE;
            calc_state      <= CALC_IDLE;
            calib_counter   <= 16'd0;
            calib_busy      <= 1'b0;
            calib_ready     <= 1'b0;
            anc_active      <= 1'b0;
            spk_out         <= 16'd0;
            tap_idx         <= 7'd0;
            acc             <= 48'd0;
            cur_e           <= 16'd0;
            cur_yf          <= 16'd0;
            cur_dhat        <= 16'd0;
            cur_x           <= 16'd0;
            cur_y           <= 16'd0;
            cur_xf          <= 16'd0;

            for (i = 0; i < M_TAPS; i = i + 1) begin
                s_hat_ram[i] <= 32'd0;
                v_buf[i]     <= 16'd0;
                y_buf[i]     <= 16'd0;
            end
            for (i = 0; i < L_TAPS; i = i + 1) begin
                w_ram[i]     <= 32'd0;
                x_buf[i]     <= 16'd0;
                xf_buf[i]    <= 16'd0;
            end
            for (i = 0; i <= DELTA; i = i + 1) begin
                delay_d_buf[i] <= 16'd0;
            end
        end else begin
            // Xử lý chuyển trạng thái toàn cục
            if (btn_calib_start && !calib_busy) begin
                // Bắt đầu pha Calib học đường dẫn thứ cấp
                sys_state     <= STATE_CALIB_RUN;
                calib_counter <= 16'd0;
                calib_busy    <= 1'b1;
                calib_ready   <= 1'b0;
                anc_active    <= 1'b0;
                // Khởi tạo lại trọng số S_hat và W về 0
                for (i = 0; i < M_TAPS; i = i + 1) s_hat_ram[i] <= 32'd0;
                for (i = 0; i < L_TAPS; i = i + 1) w_ram[i]     <= 32'd0;
            end

            // Xử lý tính toán khi có xung lấy mẫu audio mới (sample_valid = 1)
            case (calc_state)
                CALC_IDLE: begin
                    if (sample_valid) begin
                        cur_e <= mic_in; // Đọc sai số từ microphone

                        case (sys_state)
                            STATE_CALIB_RUN: begin
                                // 1. Cập nhật hàng đợi v(n)
                                for (i = M_TAPS-1; i > 0; i = i - 1)
                                    v_buf[i] <= v_buf[i-1];
                                v_buf[0] <= white_noise;

                                // Phát nhiễu trắng ra Loa mini
                                spk_out <= white_noise;

                                // Bắt đầu tính tích chập y_s(n) = sum(s_hat_m * v_{n-m})
                                tap_idx    <= 7'd0;
                                acc        <= 48'd0;
                                calc_state <= CALC_YS_FIR;
                            end

                            STATE_ANC_RUN: begin
                                // Bắt đầu tính toán các bước của Feedback FxLMS
                                tap_idx    <= 7'd0;
                                acc        <= 48'd0;
                                calc_state <= CALC_YF_FIR;
                            end

                            STATE_BYPASS, STATE_CALIB_DONE, STATE_IDLE: begin
                                spk_out <= 16'd0; // Tắt tiếng chống ồn
                                if (sys_state == STATE_CALIB_DONE || sys_state == STATE_BYPASS) begin
                                    if (sw_anc_enable && calib_ready) begin
                                        sys_state  <= STATE_ANC_RUN;
                                        anc_active <= 1'b1;
                                    end else begin
                                        sys_state  <= STATE_BYPASS;
                                        anc_active <= 1'b0;
                                    end
                                end
                            end
                        endcase
                    end else begin
                        // Kiểm tra chuyển trạng thái từ công tắc SW khi không có mẫu
                        if (sys_state == STATE_ANC_RUN && !sw_anc_enable) begin
                            sys_state  <= STATE_BYPASS;
                            anc_active <= 1'b0;
                            spk_out    <= 16'd0;
                        end else if (sys_state == STATE_BYPASS && sw_anc_enable && calib_ready) begin
                            sys_state  <= STATE_ANC_RUN;
                            anc_active <= 1'b1;
                        end
                    end
                end

                // -------------------------------------------------------------
                // CÁC BƯỚC TÍNH TOÁN PHA CALIBRATION
                // -------------------------------------------------------------
                CALC_YS_FIR: begin
                    // acc += (s_hat[m] * v[m]) >>> 15
                    acc <= acc + ((s_hat_ram[tap_idx] * v_buf[tap_idx]) >>> 15);
                    if (tap_idx == M_TAPS - 1) begin
                        tap_idx    <= 7'd0;
                        calc_state <= CALC_SHAT_UP;
                    end else begin
                        tap_idx <= tap_idx + 1'b1;
                    end
                end

                CALC_SHAT_UP: begin
                    // Sai số nhận dạng: e_s = mic_in - y_s
                    // Cập nhật: s_hat[m] += (e_s * v[m]) >>> SHIFT_MU_S
                    tmp_ys_val  = acc[30:15];
                    tmp_e_s     = cur_e - tmp_ys_val;
                    tmp_grad_s  = (tmp_e_s * v_buf[tap_idx]) >>> SHIFT_MU_S;

                    s_hat_ram[tap_idx] <= s_hat_ram[tap_idx] + tmp_grad_s;

                    if (tap_idx == M_TAPS - 1) begin
                        calc_state <= CALC_DONE;
                        if (calib_counter >= CALIB_SAMPLES) begin
                            sys_state   <= STATE_CALIB_DONE;
                            calib_busy  <= 1'b0;
                            calib_ready <= 1'b1;
                            spk_out     <= 16'd0;
                        end else begin
                            calib_counter <= calib_counter + 1'b1;
                        end
                    end else begin
                        tap_idx <= tap_idx + 1'b1;
                    end
                end

                // -------------------------------------------------------------
                // CÁC BƯỚC TÍNH TOÁN PHA FEEDBACK ANC
                // -------------------------------------------------------------
                // Bước 1: Tính y_f(n) = S_hat(z) * y(n)
                CALC_YF_FIR: begin
                    acc <= acc + ((s_hat_ram[tap_idx] * y_buf[tap_idx]) >>> 15);
                    if (tap_idx == M_TAPS - 1) begin
                        cur_yf     <= acc[30:15];
                        calc_state <= CALC_SYNTH_X;
                    end else begin
                        tap_idx <= tap_idx + 1'b1;
                    end
                end

                // Bước 2: Tái tạo nhiễu sơ cấp d_hat(n) = e(n) - y_f(n) và trễ Delta
                CALC_SYNTH_X: begin
                    tmp_d_hat = cur_e - cur_yf;

                    // Đẩy vào hàng đợi trễ Delta
                    for (i = DELTA; i > 0; i = i - 1)
                        delay_d_buf[i] <= delay_d_buf[i-1];
                    delay_d_buf[0] <= tmp_d_hat;

                    // Tín hiệu tham chiếu tổng hợp x(n) = d_hat(n - Delta)
                    cur_x <= delay_d_buf[DELTA];

                    // Đẩy x(n) vào buffer lọc
                    for (i = L_TAPS-1; i > 0; i = i - 1)
                        x_buf[i] <= x_buf[i-1];
                    x_buf[0] <= delay_d_buf[DELTA];

                    tap_idx    <= 7'd0;
                    acc        <= 48'd0;
                    calc_state <= CALC_Y_FIR;
                end

                // Bước 3: Tính tín hiệu chống ồn y(n) = W(z) * x(n)
                CALC_Y_FIR: begin
                    acc <= acc + ((w_ram[tap_idx] * x_buf[tap_idx]) >>> 15);
                    if (tap_idx == L_TAPS - 1) begin
                        // Kiểm tra bão hòa chống tràn số (Saturation)
                        if (acc[47:31] > 0 && acc[30] == 1'b0)
                            tmp_sat_y = 16'sh7FFF; // Kẹp +32767
                        else if (acc[47:31] < -1 && acc[30] == 1'b1)
                            tmp_sat_y = 16'sh8000; // Kẹp -32768
                        else
                            tmp_sat_y = acc[30:15];

                        cur_y   <= tmp_sat_y;
                        spk_out <= tmp_sat_y; // Xuất ngay tín hiệu chống ồn ra Loa mini!

                        // Cập nhật buffer y
                        for (i = M_TAPS-1; i > 0; i = i - 1)
                            y_buf[i] <= y_buf[i-1];
                        y_buf[0] <= tmp_sat_y;

                        tap_idx    <= 7'd0;
                        acc        <= 48'd0;
                        calc_state <= CALC_XF_FIR;
                    end else begin
                        tap_idx <= tap_idx + 1'b1;
                    end
                end

                // Bước 4: Lọc tín hiệu tham chiếu qua S_hat: x_f(n) = S_hat(z) * x(n)
                CALC_XF_FIR: begin
                    acc <= acc + ((s_hat_ram[tap_idx] * x_buf[tap_idx]) >>> 15);
                    if (tap_idx == M_TAPS - 1) begin
                        tmp_sat_xf = acc[30:15];
                        cur_xf     <= tmp_sat_xf;

                        // Cập nhật buffer xf
                        for (i = L_TAPS-1; i > 0; i = i - 1)
                            xf_buf[i] <= xf_buf[i-1];
                        xf_buf[0] <= tmp_sat_xf;

                        tap_idx    <= 7'd0;
                        calc_state <= CALC_W_UP;
                    end else begin
                        tap_idx <= tap_idx + 1'b1;
                    end
                end

                // Bước 5: Cập nhật trọng số FxLMS: w[l] <= w[l] - (e * xf[l]) >>> mu_w
                CALC_W_UP: begin
                    tmp_grad_w = (cur_e * xf_buf[tap_idx]) >>> shift_mu_w;
                    w_ram[tap_idx] <= w_ram[tap_idx] - tmp_grad_w;

                    if (tap_idx == L_TAPS - 1) begin
                        calc_state <= CALC_DONE;
                    end else begin
                        tap_idx <= tap_idx + 1'b1;
                    end
                end

                CALC_DONE: begin
                    calc_state <= CALC_IDLE; // Hoàn thành tính toán cho mẫu này
                end

                default: calc_state <= CALC_IDLE;
            endcase
        end
    end

endmodule
