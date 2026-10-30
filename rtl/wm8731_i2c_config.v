// Module: wm8731_i2c_config.v
// Mục đích: Cấu hình IC Audio CODEC Wolfson WM8731 (dùng cho DE2 và DE10)
// Cấu hình:
//   - Ngõ vào: Cổng MIC IN (Jack màu hồng), kích hoạt +20dB Mic Preamp Boost
//   - Ngõ ra:  Cổng LINE OUT (Jack màu xanh lá), 0dB Gain
//   - Định dạng: I2S Mode, 16-bit
//   - Sampling: Chuẩn 8 kHz
//
// Giao tiếp chân vật lý (ánh xạ tại Top-level, KHÔNG phải tại đây):
//   DE2         : I2C_SCLK = PIN_A6 | I2C_SDAT = PIN_B6
//   DE10-Standard: FPGA_I2C_SCLK = PIN_Y24 | FPGA_I2C_SDAT = PIN_Y23
module wm8731_i2c_config (
    input  wire        clk,          // Clock hệ thống 50 MHz
    input  wire        reset_n,      // Tín hiệu reset tích cực thấp (KEY[0])
    output reg         i2c_sclk,     // Xung nhịp I2C (nối ra I2C_SCLK trên board)
    inout  wire        i2c_sdat,     // Dữ liệu I2C  (nối ra I2C_SDAT trên board)
    output reg         config_done   // Báo đã cấu hình xong toàn bộ thanh ghi
);

    // Địa chỉ I2C của WM8731 trên DE10-Standard: 0x1A (7-bit) -> 0x34 khi Write
    localparam [7:0] I2C_DEV_ADDR = 8'h34;

    // Bộ chia xung tạo I2C clock ~ 50 kHz từ 50 MHz (50MHz / 1000 = 50kHz)
    reg [9:0] clk_div;
    reg       i2c_tick;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            clk_div  <= 10'd0;
            i2c_tick <= 1'b0;
        end else begin
            if (clk_div == 10'd999) begin
                clk_div  <= 10'd0;
                i2c_tick <= 1'b1;
            end else begin
                clk_div  <= clk_div + 1'b1;
                i2c_tick <= 1'b0;
            end
        end
    end

    // Mảng lưu 11 lệnh cấu hình thanh ghi WM8731: {Reg_Addr[6:0], Reg_Data[8:0]} (16 bit)
    localparam NUM_REGS = 11;
    reg [15:0] reg_lut [0:NUM_REGS-1];

    initial begin
        // R15 (0x0F): Reset chip về mặc định
        reg_lut[0]  = {7'h0F, 9'b0_0000_0000};
        // R6  (0x06): Power Down Control -> Bật nguồn toàn bộ các khối (ADC, DAC, MIC, OUT, OSC)
        reg_lut[1]  = {7'h06, 9'b0_0000_0000};
        // R0  (0x00): Left Line In -> Mute Line In
        reg_lut[2]  = {7'h00, 9'b0_1001_0111};
        // R1  (0x01): Right Line In -> Mute Line In
        reg_lut[3]  = {7'h01, 9'b0_1001_0111};
        // R2  (0x02): Left Headphone/Line Out -> 0 dB volume, update cả 2 kênh
        reg_lut[4]  = {7'h02, 9'b1_0111_1001};
        // R3  (0x03): Right Headphone/Line Out -> 0 dB volume
        reg_lut[5]  = {7'h03, 9'b1_0111_1001};
        // R4  (0x04): Analog Audio Path Control
        //   - INSEL = 1: Chọn đầu vào Microphone (Jack hồng)
        //   - MICBOOST = 1: Kích hoạt mạch khuếch đại Mic +20dB
        //   - MUTEMIC = 0: Mở tiếng Mic
        //   - BYPASS = 0: Không đấu tắt analog trực tiếp vào Line Out
        //   - DACSEL = 1: Cho phép tín hiệu từ DAC ra Line Out
        //   Bit: [8:0] = 0_0001_0101 = 0x015
        reg_lut[6]  = {7'h04, 9'b0_0001_0101};
        // R5  (0x05): Digital Audio Path Control -> Bật High-Pass filter cho ADC, tắt soft-mute
        reg_lut[7]  = {7'h05, 9'b0_0000_0000};
        // R7  (0x07): Digital Audio Interface Format -> I2S mode, 16-bit data length
        reg_lut[8]  = {7'h07, 9'b0_0000_0010};
        // R8  (0x08): Sampling Control -> Normal Mode, 8 kHz sample rate (SR=0011, BOSR=0)
        reg_lut[9]  = {7'h08, 9'b0_0000_1100};
        // R9  (0x09): Active Control -> Kích hoạt lõi Audio hoạt động
        reg_lut[10] = {7'h09, 9'b0_0000_0001};
    end

    // Máy trạng thái FSM phát I2C Byte
    localparam ST_IDLE       = 4'd0;
    localparam ST_START      = 4'd1;
    localparam ST_SEND_BYTE  = 4'd2;
    localparam ST_ACK        = 4'd3;
    localparam ST_STOP       = 4'd4;
    localparam ST_WAIT_NEXT  = 4'd5;
    localparam ST_FINISHED   = 4'd6;

    reg [3:0]  state;
    reg [3:0]  reg_index;
    reg [1:0]  byte_count;  // 0: DevAddr, 1: High byte, 2: Low byte
    reg [7:0]  shift_reg;
    reg [2:0]  bit_index;
    reg [1:0]  sub_step;    // 4 pha của 1 chu kỳ I2C clock (SCL low, data set, SCL high, data hold)
    reg        sdat_out;
    reg        sdat_oe;

    assign i2c_sdat = sdat_oe ? sdat_out : 1'bz;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state        <= ST_IDLE;
            reg_index    <= 4'd0;
            byte_count   <= 2'd0;
            shift_reg    <= 8'd0;
            bit_index    <= 3'd7;
            sub_step     <= 2'd0;
            sdat_out     <= 1'b1;
            sdat_oe      <= 1'b0;
            i2c_sclk     <= 1'b1;
            config_done  <= 1'b0;
        end else if (i2c_tick) begin
            case (state)
                ST_IDLE: begin
                    i2c_sclk    <= 1'b1;
                    sdat_out    <= 1'b1;
                    sdat_oe     <= 1'b1;
                    config_done <= 1'b0;
                    if (reg_index < NUM_REGS) begin
                        state <= ST_START;
                    end else begin
                        state <= ST_FINISHED;
                    end
                end

                ST_START: begin
                    // Điều kiện START: SDA từ High xuống Low trong khi SCL High
                    case (sub_step)
                        2'd0: begin sdat_oe <= 1'b1; sdat_out <= 1'b1; i2c_sclk <= 1'b1; end
                        2'd1: begin sdat_out <= 1'b0; i2c_sclk <= 1'b1; end
                        2'd2: begin sdat_out <= 1'b0; i2c_sclk <= 1'b0; end
                        2'd3: begin
                            // Nạp byte đầu tiên (I2C Device Address)
                            shift_reg  <= I2C_DEV_ADDR;
                            byte_count <= 2'd0;
                            bit_index  <= 3'd7;
                            state      <= ST_SEND_BYTE;
                        end
                    endcase
                    sub_step <= sub_step + 1'b1;
                end

                ST_SEND_BYTE: begin
                    case (sub_step)
                        2'd0: begin
                            i2c_sclk <= 1'b0;
                            sdat_oe  <= 1'b1;
                            sdat_out <= shift_reg[bit_index];
                        end
                        2'd1: begin
                            i2c_sclk <= 1'b1; // SCL lên High để Slave đọc bit
                        end
                        2'd2: begin
                            i2c_sclk <= 1'b1;
                        end
                        2'd3: begin
                            i2c_sclk <= 1'b0;
                            if (bit_index == 3'd0) begin
                                state <= ST_ACK;
                            end else begin
                                bit_index <= bit_index - 1'b1;
                            end
                        end
                    endcase
                    sub_step <= sub_step + 1'b1;
                end

                ST_ACK: begin
                    case (sub_step)
                        2'd0: begin
                            i2c_sclk <= 1'b0;
                            sdat_oe  <= 1'b0; // Thả SDA cho Slave kéo Low ACK
                        end
                        2'd1: begin
                            i2c_sclk <= 1'b1; // SCL High để đọc ACK
                        end
                        2'd2: begin
                            i2c_sclk <= 1'b1;
                        end
                        2'd3: begin
                            i2c_sclk <= 1'b0;
                            // Kiểm tra đã gửi hết 3 bytes: Addr, RegAddr/Data_MSB, Data_LSB
                            if (byte_count == 2'd0) begin
                                // Nạp Byte 1: {Reg_Addr[6:0], Reg_Data[8]}
                                shift_reg  <= reg_lut[reg_index][15:8];
                                byte_count <= 2'd1;
                                bit_index  <= 3'd7;
                                state      <= ST_SEND_BYTE;
                            end else if (byte_count == 2'd1) begin
                                // Nạp Byte 2: Reg_Data[7:0]
                                shift_reg  <= reg_lut[reg_index][7:0];
                                byte_count <= 2'd2;
                                bit_index  <= 3'd7;
                                state      <= ST_SEND_BYTE;
                            end else begin
                                state <= ST_STOP;
                            end
                        end
                    endcase
                    sub_step <= sub_step + 1'b1;
                end

                ST_STOP: begin
                    // Điều kiện STOP: SDA từ Low lên High trong khi SCL High
                    case (sub_step)
                        2'd0: begin sdat_oe <= 1'b1; sdat_out <= 1'b0; i2c_sclk <= 1'b0; end
                        2'd1: begin i2c_sclk <= 1'b1; sdat_out <= 1'b0; end
                        2'd2: begin sdat_out <= 1'b1; i2c_sclk <= 1'b1; end
                        2'd3: begin
                            reg_index <= reg_index + 1'b1;
                            state     <= ST_WAIT_NEXT;
                        end
                    endcase
                    sub_step <= sub_step + 1'b1;
                end

                ST_WAIT_NEXT: begin
                    // Chờ vài nhịp trước khi gửi lệnh tiếp theo
                    state <= ST_IDLE;
                end

                ST_FINISHED: begin
                    config_done <= 1'b1;
                    i2c_sclk    <= 1'b1;
                    sdat_oe     <= 1'b0;
                end
            endcase
        end
    end

endmodule
