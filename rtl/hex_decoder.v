// ============================================================================
// Module: hex_decoder.v
// Mục đích: Hiển thị trạng thái hệ thống ANC lên 6 LED 7-đoạn của DE10-Standard
//   - "CALIb " khi đang học đường dẫn thứ cấp
//   - "  dOnE" khi học xong
//   - "ANC On" khi đang khử tiếng ồn
//   - " bPASS" khi ở chế độ Bypass (tắt khử ồn)
// ============================================================================

module hex_decoder (
    input  wire        clk,
    input  wire        calib_busy,
    input  wire        calib_ready,
    input  wire        anc_active,
    input  wire [1:0]  mu_sel,
    output reg  [6:0]  hex0,
    output reg  [6:0]  hex1,
    output reg  [6:0]  hex2,
    output reg  [6:0]  hex3,
    output reg  [6:0]  hex4,
    output reg  [6:0]  hex5
);

    // Bảng mã 7 đoạn (Tích cực mức THẤP - 0 là sáng, 1 là tắt)
    localparam CH_BLANK = 7'b111_1111;
    localparam CH_C     = 7'b100_0110;
    localparam CH_A     = 7'b000_1000;
    localparam CH_L     = 7'b100_0111;
    localparam CH_I     = 7'b111_1001;
    localparam CH_b     = 7'b000_0011;
    localparam CH_d     = 7'b010_0001;
    localparam CH_O     = 7'b100_0000;
    localparam CH_n     = 7'b010_1011;
    localparam CH_E     = 7'b000_0110;
    localparam CH_P     = 7'b000_1100;
    localparam CH_S     = 7'b001_0010;

    // Hiển thị số 0, 1, 2, 3
    function [6:0] seg_digit;
        input [1:0] val;
        case (val)
            2'd0: seg_digit = 7'b100_0000; // 0
            2'd1: seg_digit = 7'b111_1001; // 1
            2'd2: seg_digit = 7'b010_0100; // 2
            2'd3: seg_digit = 7'b011_0000; // 3
        endcase
    endfunction

    always @(*) begin
        if (calib_busy) begin
            // "CALIb "
            hex5 = CH_C;
            hex4 = CH_A;
            hex3 = CH_L;
            hex2 = CH_I;
            hex1 = CH_b;
            hex0 = CH_BLANK;
        end else if (anc_active) begin
            // "ANC On" + mu_sel
            hex5 = CH_A;
            hex4 = CH_n;
            hex3 = CH_C;
            hex2 = CH_O;
            hex1 = CH_n;
            hex0 = seg_digit(mu_sel); // Hiển thị mức mu 0..3
        end else if (calib_ready) begin
            // "  dOnE"
            hex5 = CH_BLANK;
            hex4 = CH_BLANK;
            hex3 = CH_d;
            hex2 = CH_O;
            hex1 = CH_n;
            hex0 = CH_E;
        end else begin
            // " bPASS"
            hex5 = CH_BLANK;
            hex4 = CH_b;
            hex3 = CH_P;
            hex2 = CH_A;
            hex1 = CH_S;
            hex0 = CH_S;
        end
    end

endmodule
