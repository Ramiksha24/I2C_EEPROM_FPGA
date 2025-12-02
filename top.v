`timescale 1ns/1ps

module i2c_master_debug_wrapper(
    input         sys_clk,          // Board clock (100MHz or 200MHz)
    input         sys_rst_n,        // Board reset (active low)
    
    // I2C Physical Interface
    inout         scl,
    inout         sda
);

// Reset synchronization and control
reg [2:0] reset_sync;
wire board_reset_n;
wire combined_reset_n;

// VIO signals
wire [7:0]  vio_addr_w_rw;
wire [15:0] vio_sub_addr;
wire        vio_sub_len;
wire [23:0] vio_byte_len;
wire [7:0]  vio_data_write;
wire        vio_req_trans;
wire        vio_reset_n;

// I2C Master outputs
wire [7:0]  data_out;
wire        valid_out;
wire        req_data_chunk;
wire        busy;
wire        nack;

// I2C internal signals (tri-state control)
wire        scl_out_en;  // 1 = drive SCL low
wire        sda_out_en;  // 1 = drive SDA low
wire        sda_in;      // Input from SDA pin

// Debug signals from I2C master
wire [3:0]  state;
wire [3:0]  next_state;
wire [7:0]  addr;
wire        rw;
wire [15:0] sub_addr;
wire        sub_len;
wire [23:0] byte_len;
wire        en_scl;
wire        byte_sent;
wire [23:0] num_byte_sent;
wire [2:0]  cntr;
wire [7:0]  byte_sr;
wire        read_sub_addr_sent_flag;
wire [7:0]  data_to_write;
wire [7:0]  data_in_sr;
wire        clk_i2c;
wire [15:0] clk_i2c_cntr;
wire        sda_prev;
wire [1:0]  sda_curr;
wire        scl_prev;
wire        scl_curr;
wire        ack_in_prog;
wire        ack_nack;
wire        en_end_indicator;
wire        grab_next_data;
wire        scl_is_high;
wire        scl_is_low;

// Sampled I2C lines
reg [1:0] sda_sample;
reg [1:0] scl_sample;

// Synchronize board reset to sys_clk domain
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n)
        reset_sync <= 3'b000;
    else
        reset_sync <= {reset_sync[1:0], 1'b1};
end

assign board_reset_n = reset_sync[2];

// Combined reset: Board reset AND VIO reset
// This allows software reset via VIO while respecting hardware reset
reg vio_reset_sync1, vio_reset_sync2;

always @(posedge sys_clk) begin
    vio_reset_sync1 <= vio_reset_n;
    vio_reset_sync2 <= vio_reset_sync1;
end

wire clean_vio_reset_n = vio_reset_sync2;

assign combined_reset_n = board_reset_n & clean_vio_reset_n;


// Sample the I2C lines for monitoring
always @(posedge sys_clk) begin
    sda_sample <= {sda_sample[0], sda_in};
    scl_sample <= {scl_sample[0], scl};
end

// I2C tri-state buffers - proper implementation
// Drive low when output_en=1, otherwise release (pull-up makes it high)
assign scl = scl_out_en ? 1'b0 : 1'bz;
assign sda = sda_out_en ? 1'b0 : 1'bz;
assign sda_in = sda;  // Read back from pin

// VIO for runtime control
vio_0 vio_inst (
    .clk(sys_clk),
    
    // Outputs to I2C master (probe_out)
    .probe_out0(vio_addr_w_rw),      // [7:0]
    .probe_out1(vio_sub_addr),       // [15:0]
    .probe_out2(vio_sub_len),        // [0:0]
    .probe_out3(vio_byte_len),       // [23:0]
    .probe_out4(vio_data_write),     // [7:0]
    .probe_out5(vio_req_trans),      // [0:0]
    .probe_out6(vio_reset_n),        // [0:0] - Software reset (active high in VIO, acts as active low)
    
    // Inputs from I2C master (probe_in)
    .probe_in0(data_out),            // [7:0]
    .probe_in1(valid_out),           // [0:0]
    .probe_in2(req_data_chunk),      // [0:0]
    .probe_in3(busy),                // [0:0]
    .probe_in4(nack),                // [0:0]
    .probe_in5(state),               // [3:0]
    .probe_in6(sda_sample[1]),       // [0:0]
    .probe_in7(scl_sample[1]),       // [0:0]
    .probe_in8(combined_reset_n)     // [0:0] - Monitor actual reset state
);

// ILA for signal capture
ila_0 ila_inst (
    .clk(sys_clk),
    
    // Trigger and data probes
    .probe0(state),                  // [3:0] - Current state
    .probe1(next_state),             // [3:0] - Next state
    .probe2(sda_sample[1]),          // [0:0] - SDA line
    .probe3(scl_sample[1]),          // [0:0] - SCL line
    .probe4(sda_out_en),              // [0:0] - SDA output from master
    .probe5(clk_i2c),                // [0:0] - I2C clock
    .probe6(byte_sr),                // [7:0] - Byte shift register
    .probe7(cntr),                   // [2:0] - Bit counter
    .probe8(busy),                   // [0:0] - Busy flag
    .probe9(nack),                   // [0:0] - NACK flag
    .probe10(ack_nack),              // [0:0] - ACK/NACK value
    .probe11(ack_in_prog),           // [0:0] - ACK in progress
    .probe12(valid_out),             // [0:0] - Valid data out
    .probe13(data_out),              // [7:0] - Data output
    .probe14(sda_prev),              // [0:0] - SDA previous
    .probe15(sda_curr),              // [1:0] - SDA current sync
    .probe16(byte_sent),             // [0:0] - Byte sent flag
    .probe17(num_byte_sent),         // [23:0] - Number of bytes sent
    .probe18(vio_req_trans),         // [0:0] - Request transaction
    .probe19(en_scl),                // [0:0] - SCL enable
    .probe20(addr),                  // [7:0] - Slave address
    .probe21(rw)                     // [0:0] - Read/Write bit
);

// I2C Master instance with debug signals exposed
i2c_master i2c_master_inst (
    .i_clk(sys_clk),
    .reset_n(combined_reset_n),
    .i_addr_w_rw(vio_addr_w_rw),
    .i_sub_addr(vio_sub_addr),
    .i_sub_len(vio_sub_len),
    .i_byte_len(vio_byte_len),
    .i_data_write(vio_data_write),
    .req_trans(vio_req_trans),
    
    .data_out(data_out),
    .valid_out(valid_out),
    
    // New interface for proper tri-state
    .scl_out_en(scl_out_en),
    .sda_out_en(sda_out_en),
    .sda_in(sda_in),
    
    .req_data_chunk(req_data_chunk),
    .busy(busy),
    .nack(nack),
    
    // Debug outputs
    .state(state),
    .next_state(next_state),
    .addr(addr),
    .rw(rw),
    .sub_addr(sub_addr),
    .sub_len(sub_len),
    .byte_len(byte_len),
    .en_scl(en_scl),
    .byte_sent(byte_sent),
    .num_byte_sent(num_byte_sent),
    .cntr(cntr),
    .byte_sr(byte_sr),
    .read_sub_addr_sent_flag(read_sub_addr_sent_flag),
    .data_to_write(data_to_write),
    .data_in_sr(data_in_sr),
    .clk_i2c(clk_i2c),
    .clk_i2c_cntr(clk_i2c_cntr),
    .sda_prev(sda_prev),
    .sda_curr(sda_curr),
    .scl_prev(scl_prev),
    .scl_curr(scl_curr),
    .ack_in_prog(ack_in_prog),
    .ack_nack(ack_nack),
    .en_end_indicator(en_end_indicator),
    .grab_next_data(grab_next_data),
    .scl_is_high(scl_is_high),
    .scl_is_low(scl_is_low)
);

endmodule