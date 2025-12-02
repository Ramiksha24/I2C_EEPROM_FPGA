`timescale 1fs/1fs
module i2c_master #(
    parameter DEBUG = 0
)(
    input             i_clk,
    input             reset_n,
    input      [7:0]  i_addr_w_rw,
    input      [15:0] i_sub_addr,
    input             i_sub_len,
    input      [23:0] i_byte_len,
    input      [7:0]  i_data_write,
    input             req_trans,
    
    // Outputs
    output reg [7:0]  data_out,
    output reg        valid_out,
    output reg        req_data_chunk,
    output reg        busy,
    output reg        nack,
    
    // I2C interface - separate input/output for proper IOBUF
    output reg        scl_out_en,    // 1 = drive SCL low, 0 = release
    output reg        sda_out_en,    // 1 = drive SDA low, 0 = release
    input             sda_in         // Read value from SDA pin
    
    `ifdef EXPOSE_DEBUG_SIGNALS
    ,
    output reg [3:0]  state,
    output reg [3:0]  next_state,
    output reg [7:0]  addr,
    output reg        rw,
    output reg [15:0] sub_addr,
    output reg        sub_len,
    output reg [23:0] byte_len,
    output reg        en_scl,
    output reg        byte_sent,
    output reg [23:0] num_byte_sent,
    output reg [2:0]  cntr,
    output reg [7:0]  byte_sr,
    output reg        read_sub_addr_sent_flag,
    output reg [7:0]  data_to_write,
    output reg [7:0]  data_in_sr,
    output reg        clk_i2c,
    output reg [15:0] clk_i2c_cntr,
    output reg        sda_prev,
    output reg [1:0]  sda_curr,
    output reg        scl_prev,
    output reg        scl_curr,
    output reg        ack_in_prog,
    output reg        ack_nack,
    output reg        en_end_indicator,
    output reg        grab_next_data,
    output reg        scl_is_high,
    output reg        scl_is_low
    `endif
);

localparam [3:0] IDLE        = 4'd0,
                 START       = 4'd1,
                 RESTART     = 4'd2,
                 SLAVE_ADDR  = 4'd3,
                 SUB_ADDR    = 4'd4,
                 READ        = 4'd5,
                 WRITE       = 4'd6,
                 GRAB_DATA   = 4'd7,
                 ACK_NACK_RX = 4'd8,
                 ACK_NACK_TX = 4'd9,
                 STOP        = 4'hA,
                 RELEASE_BUS = 4'hB;

localparam [15:0] DIV_100MHZ = 16'd260;
localparam [7:0]  START_IND_SETUP  = 120,
                  START_IND_HOLD   = 120,
                  DATA_SETUP_TIME  = 20,
                  DATA_HOLD_TIME   = 3,
                  STOP_IND_SETUP   = 120;

`ifndef EXPOSE_DEBUG_SIGNALS
reg [3:0]  state;
reg [3:0]  next_state;
reg [7:0]  addr;
reg        rw;
reg [15:0] sub_addr;
reg        sub_len;
reg [23:0] byte_len;
reg        en_scl;
reg        byte_sent;
reg [23:0] num_byte_sent;
reg [2:0]  cntr;
reg [7:0]  byte_sr;
reg        read_sub_addr_sent_flag;
reg [7:0]  data_to_write;
reg [7:0]  data_in_sr;
reg        clk_i2c;
reg [15:0] clk_i2c_cntr;
reg [1:0]  sda_curr;
reg        sda_prev;
reg        scl_prev, scl_curr;
reg        ack_in_prog;
reg        ack_nack;
reg        en_end_indicator;
reg        grab_next_data;
reg        scl_is_high;
reg        scl_is_low;
`endif

// I2C clock generation
always@(posedge i_clk or negedge reset_n) begin
    if(!reset_n) begin
        {clk_i2c_cntr, clk_i2c} <= 17'b1;
        scl_out_en <= 1'b0;  // Release SCL
    end
    else if(!en_scl) begin
        {clk_i2c_cntr, clk_i2c} <= 17'b1;
        scl_out_en <= 1'b0;  // Release SCL
    end
    else begin
        clk_i2c_cntr <= clk_i2c_cntr + 1;
        if(clk_i2c_cntr == DIV_100MHZ-1) begin
            clk_i2c <= !clk_i2c;
            clk_i2c_cntr <= 0;
        end
        // Drive SCL low when clk_i2c is low, release when high
        scl_out_en <= !clk_i2c;
    end
end

//Main FSM
always@(posedge i_clk or negedge reset_n) begin
    if(!reset_n) begin
        {data_out, valid_out} <= 0;
        {req_data_chunk, busy, nack} <= 0;
        {addr, rw, sub_addr, sub_len, byte_len, en_scl} <= 0;
        {byte_sent, num_byte_sent, cntr, byte_sr} <= 0;
        {read_sub_addr_sent_flag, data_to_write, data_in_sr} <= 0;
        {ack_nack, ack_in_prog, en_end_indicator} <= 0;
        {scl_is_high, scl_is_low, grab_next_data} <= 0;
        sda_out_en <= 1'b0;  // Release SDA
        state <= IDLE;
        next_state <= IDLE;
    end
    else begin
        valid_out <= 1'b0;
        req_data_chunk <= 1'b0;
        case(state)
            IDLE: begin
                if(req_trans & !busy) begin
                    busy <= 1'b1;
                    state <= START;
                    next_state <= SLAVE_ADDR;
                    addr <= i_addr_w_rw;
                    rw <= i_addr_w_rw[0];
                    sub_addr <= i_sub_len ? i_sub_addr : {i_sub_addr[7:0], 8'b0};
                    sub_len <= i_sub_len;
                    data_to_write <= i_data_write;
                    byte_len <= i_byte_len;
                    en_scl <= 1'b1;
                    sda_out_en <= 1'b0;  // Release SDA
                    nack <= 1'b0;  
                    read_sub_addr_sent_flag <= 1'b0;
                    num_byte_sent <= 0;
                    byte_sent <= 1'b0;
                end
            end
            
            START: begin
                if(scl_prev & scl_curr & clk_i2c_cntr == START_IND_SETUP) begin
                    sda_out_en <= 1'b1;  // Drive SDA low for START
                    byte_sr <= {addr[7:1], 1'b0};
                    state <= SLAVE_ADDR;
                    if(DEBUG) $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: START INDICATION!", $time);
                end
            end
            
            RESTART: begin
                if(!scl_curr & scl_prev) begin
                    sda_out_en <= 1'b0;  // Release SDA
                end
                
                if(!scl_prev & scl_curr) begin
                    scl_is_high <= 1'b1;
                end
                
                if(scl_is_high) begin
                    if(clk_i2c_cntr == START_IND_SETUP) begin
                        scl_is_high <= 1'b0;
                        sda_out_en <= 1'b1;  // Drive SDA low
                        state <= SLAVE_ADDR;
                        byte_sr <= addr;
                    end
                end
            end
            
                  
            SLAVE_ADDR: begin
                if(byte_sent) begin
                    byte_sent <= 1'b0;
                    next_state <= read_sub_addr_sent_flag ? READ : SUB_ADDR;
                    byte_sr <= sub_addr[15:8];
                    state <= ACK_NACK_RX;
                    sda_out_en <= 1'b0;  // Release SDA for ACK
                    cntr <= 0;
                    if(DEBUG) $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: SLAVE_ADDR SENT!", $time);
                end
                else begin
                    if(!scl_curr & scl_prev) begin
                        scl_is_low <= 1'b1;
                    end
                    
                    if(scl_is_low) begin
                        if(clk_i2c_cntr == DATA_HOLD_TIME) begin
                            {byte_sent, cntr} <= {byte_sent, cntr} + 1;
                            sda_out_en <= !byte_sr[7];  // Drive low if bit is 0
                            byte_sr <= {byte_sr[6:0], 1'b0};
                            scl_is_low <= 1'b0;
                        end
                    end
                end
            end
            
            SUB_ADDR: begin
                if(byte_sent) begin
                    if(sub_len) begin
                        state <= ACK_NACK_RX;
                        next_state <= SUB_ADDR;
                        sub_len <= 1'b0;
                        byte_sr <= sub_addr[7:0];
                        if(DEBUG) $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: MSB OF SUB ADDR SENT", $time);
                    end
                    else begin
                        next_state <= rw ? RESTART : WRITE;
                        byte_sr <= rw ? byte_sr : data_to_write;
                        read_sub_addr_sent_flag <= 1'b1;
                        if(DEBUG) $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: SUB ADDR SENT", $time);
                    end
                    
                    cntr <= 0;
                    byte_sent <= 1'b0;
                    state <= ACK_NACK_RX;
                    sda_out_en <= 1'b0;  // Release for ACK
                end
                else begin
                    if(!scl_curr & scl_prev) begin
                        scl_is_low <= 1'b1;
                    end
                    
                    if(scl_is_low) begin
                        if(clk_i2c_cntr == DATA_HOLD_TIME) begin
                            scl_is_low <= 1'b0;
                            {byte_sent, cntr} <= {byte_sent, cntr} + 1;
                            sda_out_en <= !byte_sr[7];
                            byte_sr <= {byte_sr[6:0], 1'b0};
                        end
                    end
                end
            end
            
            READ: begin
                if(byte_sent) begin
                    byte_sent <= 1'b0;
                    data_out  <= data_in_sr;
                    valid_out <= 1'b1;
                    state <= ACK_NACK_TX;
                    next_state <= (num_byte_sent == byte_len-1) ? STOP : READ;
                    ack_nack <= num_byte_sent == byte_len-1;
                    num_byte_sent <= num_byte_sent + 1;
                    ack_in_prog <= 1'b1;
                    if(DEBUG) $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: READ BYTE #%d SENT!", $time, num_byte_sent);
                end
                else begin
                    if(!scl_prev & scl_curr) begin
                        scl_is_high <= 1'b1;
                    end
                    
                    if(scl_is_high) begin
                        if(clk_i2c_cntr == START_IND_SETUP) begin
                            valid_out <= 1'b0;
                            {byte_sent, cntr} <= cntr + 1;
                            data_in_sr <= {data_in_sr[6:0], sda_prev};
                            scl_is_high <= 1'b0;
                        end
                    end
                end
            end
            
            WRITE: begin
                if(byte_sent) begin
                    cntr <= 0;
                    byte_sent <= 1'b0;
                    state <= ACK_NACK_RX;
                    sda_out_en <= 1'b0;  // Release for ACK
                    next_state <= (num_byte_sent == byte_len-1) ? STOP : GRAB_DATA;
                    num_byte_sent <= num_byte_sent + 1'b1;
                    grab_next_data <= 1'b1;
                    if(DEBUG) $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: WRITE BYTE #%d SENT!", $time, num_byte_sent);
                end
                else begin
                    if(!scl_curr & scl_prev) begin
                        scl_is_low <= 1'b1;
                    end
                    
                    if(scl_is_low) begin
                        if(clk_i2c_cntr == DATA_HOLD_TIME) begin
                            {byte_sent, cntr} <= {byte_sent, cntr} + 1;
                            sda_out_en <= !byte_sr[7];
                            byte_sr <= {byte_sr[6:0], 1'b0};
                            scl_is_low <= 1'b0;
                        end
                    end
                end
            end
            
            GRAB_DATA: begin
                if(grab_next_data) begin
                    req_data_chunk <= 1'b1;
                    grab_next_data <= 1'b0;
                end
                else begin
                    state <= WRITE;
                    byte_sr <= i_data_write;
                end
            end
            
            ACK_NACK_RX: begin
                if(!scl_prev & scl_curr) begin
                    scl_is_high <= 1'b1;
                end
                
                if(scl_is_high) begin
                    if(clk_i2c_cntr == START_IND_SETUP) begin
                        if(!sda_prev) begin
                            state <= next_state;
                            if(DEBUG) $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: rx ack encountered", $time);
                        end
                        else begin
                            if(DEBUG) $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: rx nack encountered", $time);
                            nack <= 1'b1;
                            busy <= 1'b0;
                            sda_out_en <= 1'b0;
                            en_scl <= 1'b0;
                            state <= IDLE;
                        end  
                        scl_is_high <= 1'b0;
                    end
                end
            end
            
            ACK_NACK_TX: begin
                if(!scl_curr & scl_prev) begin
                    scl_is_low <= 1'b1;
                end
                
                if(scl_is_low) begin
                    if(clk_i2c_cntr == DATA_HOLD_TIME) begin
                        scl_is_low <= 1'b0;
                        
                        if(ack_in_prog) begin 
                            sda_out_en <= !ack_nack;  // ACK=drive low, NACK=release
                            ack_in_prog <= 1'b0;
                            if(DEBUG) $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: TX ACK/NACK = %b", $time, ack_nack);
                        end
                        else begin
                            sda_out_en <= (next_state == STOP) ? 1'b1 : 1'b0;  // Drive low for STOP prep
                            en_end_indicator <= next_state == STOP ? 1'b1 : en_end_indicator;
                            state <= next_state;
                            if(DEBUG) $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: ACK complete, moving to %d", $time, next_state);
                        end
                    end
                end
            end    
            
            STOP: begin 
                if(!scl_curr & scl_prev & !rw) begin
                    sda_out_en <= 1'b1;  // Drive SDA low
                    en_end_indicator <= 1'b1;
                end
                
                if(scl_curr & scl_prev & en_end_indicator) begin
                    scl_is_high <= 1'b1;
                    en_end_indicator <= 1'b0;
                end
                
                if(scl_is_high) begin
                    if(clk_i2c_cntr == STOP_IND_SETUP) begin
                        sda_out_en <= 1'b0;  // Release SDA for STOP
                        state <= RELEASE_BUS;
                        scl_is_high <= 1'b0;
                    end
                end
            end
            
            RELEASE_BUS: begin
                if(clk_i2c_cntr == DIV_100MHZ-3) begin
                    en_scl <= 1'b0;
                    state <= IDLE;
                    sda_out_en <= 1'b0;
                    busy <= 1'b0;
                end
            end
            
            default:
                state <= IDLE;
        endcase
    end
end

// Sample SDA input
always@(negedge i_clk or negedge reset_n) begin
    if(!reset_n) begin
        {sda_curr, sda_prev} <= 0;
        {scl_curr, scl_prev} <= 0;
    end
    else begin
        sda_curr <= {sda_curr[0], sda_in};
        sda_prev <= sda_curr[1];
        scl_curr <= clk_i2c;
        scl_prev <= scl_curr;
    end
end

endmodule