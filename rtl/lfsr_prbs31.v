// Module: lfsr_prbs31.v
// Mục đích: Bộ phát số ngẫu nhiên giả (PRBS-31 LFSR) tạo nhiễu trắng (White Noise)
//           sử dụng cho giai đoạn học ước lượng đường dẫn thứ cấp S_hat(z).
// Đa thức: x^31 + x^28 + 1 (chu kỳ cực đại 2^31 - 1 mẫu, không bị lặp âm)
// Tín hiệu ra: 16-bit signed Q1.15, biên độ giới hạn an toàn ~ 0.25 Full-Scale

module lfsr_prbs31 (
    input  wire        clk,          // Clock 50 MHz
    input  wire        reset_n,      // Reset tích cực thấp
    input  wire        enable,       // Cho phép nhảy sang mẫu ngẫu nhiên tiếp theo (nối với sample_valid)
    output wire signed [15:0] noise_out // Tín hiệu nhiễu trắng Q1.15
);

    reg [30:0] lfsr_state;

    // Phản hồi đa thức Galois/Fibonacci PRBS-31: Bit 30 ^ Bit 27
    wire feedback = lfsr_state[30] ^ lfsr_state[27];

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            // Giá trị khởi tạo khác 0 (Seed)
            lfsr_state <= 31'h5A5A_A5A5;
        end else if (enable) begin
            lfsr_state <= {lfsr_state[29:0], feedback};
        end
    end

    // Thu nhỏ biên độ: Lấy 14-bit cao và nhân tỉ lệ để biên độ max ~ 8191 (tức 0.25 FS)
    // Giúp loa mini phát tiếng rào rào êm ái, không bị vỡ tiếng (clipping)
    assign noise_out = { {2{lfsr_state[30]}}, lfsr_state[30:17] };

endmodule
