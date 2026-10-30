// ============================================================================
// Testbench: tb_feedback_fxlms_core.v
// Mục đích: Kiểm chứng toàn diện chức năng RTL của lõi Feedback FxLMS Core
//   1. Tự động đọc dữ liệu tiếng ồn từ file âm thanh bên ngoài: "noise_input.txt"
//   2. Mô phỏng đáp ứng âm học của ống dẫn thực tế S(z)
//   3. Chạy qua 3 giai đoạn:
//        - Pha 1: Calib học S_hat(z) bằng nhiễu trắng
//        - Pha 2: Chạy tiếng ồn ở chế độ Bypass (ANC OFF) -> ghi âm tiếng ồn gốc
//        - Pha 3: Bật Switch SW[0] = 1 (ANC ON) -> ghi âm tiếng sau khi khử
//   4. Xuất toàn bộ dạng sóng sai số ra file "anc_output.txt" để có thể chuyển
//      ngược lại thành file .wav nghe thử trực tiếp!
// ============================================================================

`timescale 1ns / 1ps

module tb_feedback_fxlms_core;

    reg clk;
    reg reset_n;
    reg btn_calib_start;
    reg sw_anc_enable;
    reg [1:0] sw_mu_sel;
    reg sample_valid;
    reg signed [15:0] mic_in;
    wire signed [15:0] spk_out;

    wire calib_busy;
    wire calib_ready;
    wire anc_active;

    // Bộ phát nhiễu trắng LFSR
    wire signed [15:0] white_noise;
    lfsr_prbs31 u_lfsr_tb (
        .clk       (clk),
        .reset_n   (reset_n),
        .enable    (sample_valid),
        .noise_out (white_noise)
    );

    // Lõi ANC DUT
    localparam TB_M_TAPS = 32;
    localparam TB_L_TAPS = 32;
    localparam TB_DELTA  = 4;
    localparam TB_CALIB  = 1200; // 1200 mẫu Calib trong TB để mô phỏng nhanh

    feedback_fxlms_core #(
        .M_TAPS         (TB_M_TAPS),
        .L_TAPS         (TB_L_TAPS),
        .DELTA          (TB_DELTA),
        .CALIB_SAMPLES  (TB_CALIB)
    ) dut (
        .clk             (clk),
        .reset_n         (reset_n),
        .btn_calib_start (btn_calib_start),
        .sw_anc_enable   (sw_anc_enable),
        .sw_mu_sel       (sw_mu_sel),
        .sample_valid    (sample_valid),
        .mic_in          (mic_in),
        .spk_out         (spk_out),
        .white_noise     (white_noise),
        .calib_busy      (calib_busy),
        .calib_ready     (calib_ready),
        .anc_active      (anc_active)
    );

    // ========================================================================
    // 1. Tạo xung nhịp 50 MHz (Chu kỳ 20ns)
    // ========================================================================
    initial begin
        clk = 0;
        forever #10 clk = ~clk;
    end

    // ========================================================================
    // 2. Mô phỏng Kênh truyền Âm học S(z) thực tế trong ống dẫn (Duct Acoustics)
    // ========================================================================
    reg signed [15:0] s_real_shift [0:TB_M_TAPS-1];
    reg signed [31:0] acc_s_real;
    reg signed [15:0] ya_acoustic; // Sóng âm anti-noise thực tế truyền đến mic
    integer idx;

    always @(posedge clk or negedge reset_n) begin: ACOUSTIC_SIM
        if (!reset_n) begin
            for (idx = 0; idx < TB_M_TAPS; idx = idx + 1)
                s_real_shift[idx] <= 16'd0;
            ya_acoustic <= 16'd0;
        end else if (sample_valid) begin
            for (idx = TB_M_TAPS-1; idx > 0; idx = idx - 1)
                s_real_shift[idx] <= s_real_shift[idx-1];
            s_real_shift[0] <= spk_out;

            // Đáp ứng xung S(z) thực tế trong ống:
            acc_s_real = (s_real_shift[4] * 18022) +
                         (s_real_shift[5] * -11468) +
                         (s_real_shift[6] * 6553) +
                         (s_real_shift[7] * -3276);

            ya_acoustic <= acc_s_real[30:15];
        end
    end

    // ========================================================================
    // 3. Tiến trình Đọc File và Điều khiển Testbench
    // ========================================================================
    integer file_in, file_out;
    integer scan_status;
    integer sample_idx;
    integer d_val_int;
    reg signed [15:0] d_noise_q15;

    // Biến thống kê biên độ
    integer bypass_peak;
    integer anc_peak;

    initial begin
        reset_n         = 0;
        btn_calib_start = 0;
        sw_anc_enable   = 0;
        sw_mu_sel       = 2'b00; // mu = 2^-8
        sample_valid    = 0;
        mic_in          = 0;
        d_noise_q15     = 0;
        bypass_peak     = 0;
        anc_peak        = 0;

        $display("============================================================");
        $display("[TESTBENCH] MO PHONG HE THONG ANC VOI FILE AM THANH THUC TE");
        $display("============================================================");

        // Mở file âm thanh đầu vào (Thử đường dẫn tuyệt đối trước, sau đó thử đường dẫn tương đối)
        file_in = $fopen("E:/dsp_doan/tb/noise_input.txt", "r");
        if (file_in == 0) begin
            file_in = $fopen("noise_input.txt", "r");
        end

        file_out = $fopen("E:/dsp_doan/tb/anc_output.txt", "w");
        if (file_out == 0) begin
            file_out = $fopen("anc_output.txt", "w");
        end

        if (file_in == 0) begin
            $display("[CANH BAO] Khong mo duoc file 'noise_input.txt'. Se tu tao song sin 300Hz thay the!");
        end else begin
            $display("[INFO] Da mo file am thanh dau vao 'noise_input.txt' thanh cong.");
        end

        // Nhả Reset sau 100ns
        #100;
        reset_n = 1;
        #100;

        // -------------------------------------------------------------
        // GIAI ĐOẠN 1: CALIB HỌC ĐƯỜNG DẪN THỨ CẤP BẰNG NHIỄU TRẮNG
        // -------------------------------------------------------------
        $display("[TESTBENCH] [1] Nhan KEY[1]: Phat nhieu trang de hoc S_hat(z)...");
        @(posedge clk);
        btn_calib_start = 1;
        #60;
        btn_calib_start = 0;

        for (sample_idx = 0; sample_idx < TB_CALIB + 50; sample_idx = sample_idx + 1) begin
            #3980;
            @(posedge clk);
            sample_valid = 1;
            // Lúc Calib, tiếng ồn d(n) = 0, mic chỉ nhận tín hiệu từ loa qua ống S(z)
            mic_in = ya_acoustic;
            @(posedge clk);
            sample_valid = 0;
        end

        if (calib_ready)
            $display("[TESTBENCH] [2] Calib HOAN THANH! Mo hinh S_hat da hoi tu.");
        else
            $display("[TESTBENCH] [ERROR] Calib chua hoan thanh!");

        // -------------------------------------------------------------
        // GIAI ĐOẠN 2: CHẠY TIẾNG ỒN Ở CHẾ ĐỘ BYPASS (ANC TẮT)
        // -------------------------------------------------------------
        $display("[TESTBENCH] [3] Phat file am thanh o che do BYPASS (ANC OFF - do tieng on goc)...");
        sw_anc_enable = 0;

        for (sample_idx = 0; sample_idx < 1000; sample_idx = sample_idx + 1) begin
            #3980;
            @(posedge clk);
            sample_valid = 1;

            // Đọc mẫu âm thanh từ file
            if (file_in != 0 && !$feof(file_in)) begin
                scan_status = $fscanf(file_in, "%d\n", d_val_int);
                d_noise_q15 = d_val_int[15:0];
            end else begin
                // Tạo sóng sin nếu không có file
                d_noise_q15 = 16384 * $sin(sample_idx * 300.0 / 8000.0 * 2.0 * 3.14159265);
            end

            mic_in = d_noise_q15 + ya_acoustic; // ya_acoustic = 0 vì ANC đang tắt

            if (sample_idx > 500 && (mic_in > bypass_peak || -mic_in > bypass_peak))
                bypass_peak = (mic_in >= 0) ? mic_in : -mic_in;

            // Ghi ra file output
            if (file_out != 0) $fdisplay(file_out, "%d", mic_in);

            @(posedge clk);
            sample_valid = 0;
        end

        // -------------------------------------------------------------
        // GIAI ĐOẠN 3: BẬT SWITCH SW[0] = 1 (ANC ON - BẮT ĐẦU KHỬ TIẾNG ỒN)
        // -------------------------------------------------------------
        $display("[TESTBENCH] [4] Gat SW[0] = 1: BAT ANC KHU NHIEU!");
        sw_anc_enable = 1;

        for (sample_idx = 0; sample_idx < 3000; sample_idx = sample_idx + 1) begin
            #3980;
            @(posedge clk);
            sample_valid = 1;

            if (file_in != 0 && !$feof(file_in)) begin
                scan_status = $fscanf(file_in, "%d\n", d_val_int);
                d_noise_q15 = d_val_int[15:0];
            end else begin
                d_noise_q15 = 16384 * $sin((sample_idx + 1000) * 300.0 / 8000.0 * 2.0 * 3.14159265);
            end

            // Âm thanh thực tế tại mic: tổng sóng ồn + sóng chống ồn
            mic_in = d_noise_q15 + ya_acoustic;

            if (sample_idx > 2000 && (mic_in > anc_peak || -mic_in > anc_peak))
                anc_peak = (mic_in >= 0) ? mic_in : -mic_in;

            // Ghi ra file output
            if (file_out != 0) $fdisplay(file_out, "%d", mic_in);

            @(posedge clk);
            sample_valid = 0;
        end

        // Đóng file
        if (file_in != 0)  $fclose(file_in);
        if (file_out != 0) $fclose(file_out);

        $display("============================================================");
        $display("[TESTBENCH KET QUA]:");
        $display("  - Bien do dinh tieng on khi Bypass (ANC OFF): %d", bypass_peak);
        $display("  - Bien do dinh sai so khi ANC ON (Sau hoi tu): %d", anc_peak);
        if (bypass_peak > 0 && anc_peak > 0) begin
            $display("  - Do suy giam bien do tieng on: %.2f %%", (1.0 - (1.0*anc_peak/bypass_peak)) * 100.0);
        end
        $display("  - Du lieu ket qua da duoc ghi vao file: 'anc_output.txt'");
        $display("============================================================");
        $stop;
    end

endmodule
