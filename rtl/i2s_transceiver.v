// Module: i2s_transceiver.v
// Mục đích: Giao tiếp I2S Audio 16-bit với IC WM8731
//   - Nhận tín hiệu ADC từ Mic In (AUD_ADCDAT) -> chuyển thành mẫu song song 16-bit signed
//   - Truyền tín hiệu DAC ra Line Out (AUD_DACDAT) -> phát ra loa mini chống ồn
//   - Tạo xung đồng bộ sample_valid (tần số fs = 8 kHz) ở miền clock 50 MHz

module i2s_transceiver (
    input  wire        clk,             // Clock hệ thống 50 MHz
    input  wire        reset_n,         // Reset tích cực thấp

    // Giao tiếp với chân phần cứng CODEC WM8731
    input  wire        aud_bclk,        // Audio Bit Clock (PIN_AF30)
    input  wire        aud_adclrck,     // ADC Left/Right Word Clock (PIN_AJ29)
    input  wire        aud_adcdat,      // ADC Serial Data In (PIN_AH29)
    input  wire        aud_daclrck,     // DAC Left/Right Word Clock (PIN_AG30)
    output reg         aud_dacdat,      // DAC Serial Data Out (PIN_AF29)

    // Giao tiếp với Lõi xử lý DSP (ANC Core)
    output reg         sample_valid,    // Xung kích hoạt 1 chu kỳ 50MHz khi có mẫu mới
    output reg  signed [15:0] mic_data, // Dữ liệu nhận từ Mic (Q1.15)
    input  wire signed [15:0] spk_data  // Dữ liệu xuất ra Loa mini (Q1.15)
);

    // Đồng bộ tín hiệu aud_bclk và aud_adclrck sang miền clock 50 MHz
    reg [2:0] bclk_sync;
    reg [2:0] lrck_sync;
    reg [1:0] adcdat_sync;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            bclk_sync   <= 3'b0;
            lrck_sync   <= 3'b0;
            adcdat_sync <= 2'b0;
        end else begin
            bclk_sync   <= {bclk_sync[1:0], aud_bclk};
            lrck_sync   <= {lrck_sync[1:0], aud_adclrck};
            adcdat_sync <= {adcdat_sync[0], aud_adcdat};
        end
    end

    wire bclk_rise = (bclk_sync[2:1] == 2'b01);
    wire bclk_fall = (bclk_sync[2:1] == 2'b10);
    wire lrck_edge = (lrck_sync[2] != lrck_sync[1]); // Bắt đầu kênh Left hoặc Right mới

    // Thanh ghi dịch thu/phát I2S
    reg [15:0] rx_shift_reg;
    reg [15:0] tx_shift_reg;
    reg [4:0]  bit_count;
    reg        lrck_prev;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            rx_shift_reg <= 16'd0;
            tx_shift_reg <= 16'd0;
            bit_count    <= 5'd0;
            aud_dacdat   <= 1'b0;
            sample_valid <= 1'b0;
            mic_data     <= 16'd0;
            lrck_prev    <= 1'b0;
        end else begin
            sample_valid <= 1'b0; // Mặc định là 0, chỉ bật 1 xung nhịp

            // Phát hiện cạnh đổi kênh LRCK (Chuẩn I2S: dữ liệu bắt đầu sau cạnh LRCK 1 chu kỳ BCLK)
            if (lrck_sync[1] != lrck_prev) begin
                lrck_prev <= lrck_sync[1];
                bit_count <= 5'd0;
                // Nạp mẫu xuất vào thanh ghi dịch phát
                tx_shift_reg <= spk_data;
                // Khi vừa nhận xong kênh Left (LRCK từ 0 lên 1 hoặc 1 xuống 0 tùy codec)
                if (lrck_sync[1] == 1'b1) begin
                    mic_data     <= rx_shift_reg;
                    sample_valid <= 1'b1; // Báo cho lõi ANC biết có mẫu mới
                end
            end

            // Đọc bit tại sườn lên của BCLK (Rx từ Microphone)
            if (bclk_rise) begin
                if (bit_count > 5'd0 && bit_count <= 5'd16) begin
                    rx_shift_reg <= {rx_shift_reg[14:0], adcdat_sync[1]};
                end
                bit_count <= bit_count + 1'b1;
            end

            // Xuất bit tại sườn xuống của BCLK (Tx ra Loa mini)
            if (bclk_fall) begin
                if (bit_count >= 5'd1 && bit_count <= 5'd16) begin
                    aud_dacdat   <= tx_shift_reg[15];
                    tx_shift_reg <= {tx_shift_reg[14:0], 1'b0};
                end else begin
                    aud_dacdat   <= 1'b0;
                end
            end
        end
    end

endmodule
