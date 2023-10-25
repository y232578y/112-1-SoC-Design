//`include "../../bram/bram11.v"

`timescale 1ns / 1ps
module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    //Read address channel(RA)
    output  wire                     arready,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    input   wire                     arvalid, 
    //Read data channel(RD)
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,
    input   wire                     rready,
    //Write address channel(WA)
    output  wire                     awready,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     awvalid,
    //Write data channel(WD)
    output  wire                     wready,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,


    //streaming
    output  wire                     ss_tready,
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    input   wire                     sm_tready, 

    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

reg [(pDATA_WIDTH-1):0] multi_result;
/*
bram11 bram11_axilite(
    .CLK        (axis_clk),
    .WE         (tap_WE),
    .EN         (tap_EN),
    .Di         (tap_Di),
    .Do         (tap_Do),
    .A          (tap_A)
);

bram11 bram11_axistream(
    .CLK        (axis_clk),
    .WE         (data_WE),
    .EN         (data_EN),
    .Di         (data_Di),
    .Do         (data_Do),
    .A          (data_A)
);
*/

/******************FSM*********************/
parameter                   IDLE = 2'b00;
parameter                   INPUT_TAP = 2'b01;
parameter                   COMPUTE = 2'b10;
parameter                   DONE = 2'b11;
reg     [1:0]               next_state;
reg     [1:0]               current_state;

/******************AXI LITE*********************/
reg                         write_cnt;  // let awready follows awvalid's second clock and pulls up to 1, 
                                            // then pulls down to 0, wready follows wvalid's second clock and pulls up to 1, 
                                            // then pulls down to 0
reg [1:0]                   read_cnt;
reg [5:0]                   tap_cnt;                                         
reg                         awready_r;    // write address control : if awready is 1, tap_A = awaddr
reg                         wready_r;

reg [(pADDR_WIDTH-1):0]     tap_A_r;
reg [(pDATA_WIDTH-1):0]     tap_Di_r;
reg [3:0]                   tap_WE_r;

reg                         arready_r;
reg                         rvalid_r;


reg     [(pDATA_WIDTH-1):0] data_Di_r;
reg     [(pADDR_WIDTH-1):0] data_A_r;
reg     [3:0]               data_WE_r;

reg     [(pADDR_WIDTH-1):0] compute_cnt;
reg     [3:0]               pattern_cycle;
reg     [9:0]               pattern_number;

reg    [(pDATA_WIDTH-1):0]  Xn_reg1;
reg    [(pDATA_WIDTH-1):0]  Xn_reg2;
reg    [(pDATA_WIDTH-1):0]  coefficient_data;
reg    [(pDATA_WIDTH-1):0]  Xn;
reg    [(pDATA_WIDTH-1):0]  Product;
reg    [(pDATA_WIDTH-1):0]  Yn;
wire                        pattern_done;


/***************************************************************CIRCUIT***************************************************************/


/***********************************************/
/******************TAP BRAM*********************/
/***********************************************/
assign tap_EN = 1'b1;
assign tap_A = tap_A_r;
assign tap_Di = tap_Di_r;
assign tap_WE = tap_WE_r;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) tap_cnt <= 6'b000000;
    else if (current_state == INPUT_TAP) tap_cnt <= 0;
    else if (current_state == COMPUTE) begin
        if (data_WE == 4'b1111) tap_cnt <= 6'b000000;
        else if(ss_tready == 1) tap_cnt <= 6'b000000;
        else tap_cnt <= tap_cnt + 1'b1;
    end
    else tap_cnt <= tap_cnt;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) tap_A_r <= 12'h000;
    else if (awvalid == 1 && awready == 1 && current_state == INPUT_TAP) begin
        if(awaddr >= 12'h020) tap_A_r <= awaddr - 12'h020;
        else if(awaddr == 12'h010) tap_A_r <= awaddr;
        else tap_A_r <= 12'h000;
    end
    else if (arvalid == 1 && current_state == INPUT_TAP) begin
        if(araddr >= 12'h020) tap_A_r <= araddr - 12'h020;
        else if(araddr == 12'h000) tap_A_r <= 12'h000;
        else tap_A_r <= tap_A_r;
    end
    else if (current_state == COMPUTE) begin    
        if(tap_cnt == 6'd0) tap_A_r <= 12'h000;
        else if (tap_cnt == 6'd1) tap_A_r <= 12'h004;
        else if (tap_cnt == 6'd2) tap_A_r <= 12'h008; 
        else if (tap_cnt == 6'd3) tap_A_r <= 12'h00C;
        else if (tap_cnt == 6'd4) tap_A_r <= 12'h010;
        else if (tap_cnt == 6'd5) tap_A_r <= 12'h014;
        else if (tap_cnt == 6'd6) tap_A_r <= 12'h018;
        else if (tap_cnt == 6'd7) tap_A_r <= 12'h01C;
        else if (tap_cnt == 6'd8) tap_A_r <= 12'h020;
        else if (tap_cnt == 6'd9) tap_A_r <= 12'h024;
        else if (tap_cnt == 6'd10) tap_A_r <= 12'h028;
        else tap_A_r <= tap_A_r;
    end
    else tap_A_r <= tap_A_r;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) tap_Di_r <= 32'h0000_0000;
    else if (wvalid && wready == 1) tap_Di_r <= wdata;
    else tap_Di_r <= 32'h0000_0000;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) tap_WE_r <= 4'b0000;
    else if ((wvalid && wready == 1) && (awaddr != 12'h000)) tap_WE_r <= 4'b1111;
    else tap_WE_r <= 4'b0000;
end


/***********************************************/
/******************AXI LITE*********************/
/***********************************************/
assign rdata =  (rvalid && current_state == INPUT_TAP)?     tap_Do:
                (rvalid && current_state == COMPUTE)?               tap_Do:
                (rvalid && current_state == DONE)?                  32'd2:
                                                                    32'd4; 

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) write_cnt <= 1'b0;
    else if(awaddr != 12'h000) begin
        if (write_cnt == 1'b0 && awvalid == 1'b1) write_cnt <= 1'b1;
        else if (write_cnt == 1'b1 && awvalid == 1'b1) write_cnt <= 1'b0;
    end
    else write_cnt <= 1'b0;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) read_cnt <= 1'b0;
    else if (read_cnt == 2'b00 && arvalid == 1'b1) read_cnt <= 2'b01;
    else if (read_cnt == 2'b01 && arvalid == 1'b1) read_cnt <= 2'b10;
    else if (read_cnt == 2'b10 && arvalid == 1'b1) read_cnt <= 2'b00;
    else read_cnt <= read_cnt;
end

assign awready = (write_cnt)? 1:0;

assign wready = (write_cnt)? 1:0;

assign arready = (read_cnt == 2'b01)? 1:0; 

assign rvalid = rvalid_r;
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) rvalid_r <= 1'b0;
    else rvalid_r <= arready;
end


/******************************************/
/******************FSM*********************/
/******************************************/
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) current_state <= IDLE;
    else current_state <= next_state;
end

always @(*) begin
    case(current_state)
        IDLE: begin
            if(wdata == 32'd600) next_state = INPUT_TAP;
            else next_state = IDLE;
        end
        INPUT_TAP: begin
            if(awaddr == 12'h000 && wdata == 32'd1) next_state = COMPUTE;
            else next_state = INPUT_TAP;
        end
        COMPUTE: begin
            if(pattern_number == 600 && sm_tready && sm_tvalid) next_state = DONE;
            else next_state = COMPUTE;
        end
        DONE: begin
            if(rdata == 32'd2) next_state = IDLE;
            else next_state = DONE;
        end
        default:begin
            next_state = IDLE;
        end
    endcase
end


/************************************************/
/******************DATA BRAM*********************/
/************************************************/
assign data_Di = data_Di_r;
assign data_A = data_A_r;
assign data_EN = 1'b1;
assign data_WE = data_WE_r;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) data_WE_r <= 4'b0000;
    else if ((wvalid && wready == 1) && (awaddr != 12'h000)) data_WE_r <= 4'b1111;
    else if(current_state == COMPUTE && ss_tready == 1) data_WE_r <= 4'b1111;
    else data_WE_r <= 4'b0000;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) data_Di_r <= 32'b0;
    else if(current_state == INPUT_TAP) data_Di_r <= 32'b0;
    else if(current_state == COMPUTE) data_Di_r <= ss_tdata;
    else data_Di_r <= data_Di_r;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) data_A_r <= 12'h000;
    else if (current_state == INPUT_TAP && awvalid == 1 && awready == 1) begin
        if (awaddr != 12'h010) data_A_r <= awaddr - 12'h020;
        else if (awaddr == 12'h010) data_A_r <= 12'h000;
        else data_A_r <= data_A_r;
    end
    else if(current_state == INPUT_TAP && arvalid == 1) data_A_r <= araddr - 12'h020;
    else if(current_state == COMPUTE) begin
        if(ss_tready == 1 && pattern_cycle < 12'd11) data_A_r <= (pattern_cycle << 2);    // write data
        else if(ss_tready == 1 && pattern_cycle == 12'd11) data_A_r <= 0;                 // wrtie data
        else if(ss_tready == 0) begin
            if(data_A_r == 12'h000) data_A_r <= 12'h028;                                // read data
            else data_A_r <= data_A_r - 12'h004;
        end
        else data_A_r <= data_A_r;
    end
    else data_A_r <= data_A_r;
end
/****************************************************/
/******************AXI Streaming*********************/
/****************************************************/
assign ss_tready = (current_state == COMPUTE && compute_cnt == 0)? 1:0;
assign sm_tvalid = (pattern_number != 9'd1)? pattern_done:0;
assign sm_tlast = (pattern_number == 600 && pattern_done == 1)? 1:0;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) compute_cnt <= 4'd0;
    else if(current_state == COMPUTE && compute_cnt <= 4'd10) compute_cnt <= compute_cnt + 1'b1;
    else if(current_state == COMPUTE && compute_cnt == 4'd11) compute_cnt <= 0;
    else compute_cnt <= compute_cnt;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) pattern_cycle <= 4'd0;
    else if(current_state == COMPUTE && ss_tready == 1 && pattern_cycle < 4'd11) pattern_cycle <= pattern_cycle +4'd1;
    else if (pattern_cycle == 4'd11 && ss_tready == 1) pattern_cycle <= 4'd1;
    else pattern_cycle <= pattern_cycle;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) pattern_number <= 10'd0;
    else if(current_state == COMPUTE && pattern_done == 1'd1) pattern_number <= pattern_number + 10'd1;
    else pattern_number <= pattern_number;
end

/****************************************************/
/******************FIR Calculate*********************/
/****************************************************/
assign pattern_done = (tap_cnt == 2)? 1:0;

assign sm_tdata = Yn;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) coefficient_data <= 0;
    else if (tap_A <= 12'h028) coefficient_data <= tap_Do;
    else coefficient_data <= coefficient_data;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) Xn_reg1 <= 0;
    else if (data_WE == 4'b1111) Xn_reg1 <= 0;
    else if (data_WE == 4'b0000) Xn_reg1 <= data_Do;
    else Xn_reg1 <= Xn_reg1;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) Xn_reg2 <= 0;
    else if(data_EN == 1) Xn_reg2 <= Xn_reg1;
    else Xn_reg2 <= Xn_reg2;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) Xn <= 0;
    else if(data_EN == 1) Xn <= Xn_reg2;
    else Xn <= Xn;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) Product <= 0;
    else if(data_EN == 1) Product <= Xn * coefficient_data;
    else Product <= Product;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) Yn <= 0;
    else if (data_EN == 1 && current_state == COMPUTE) begin
        if (pattern_done == 0) Yn <= Yn + Product;
        else if (pattern_done == 1) Yn <= 0;
        else Yn <= Yn;
    end
    else Yn <= Yn;
end
endmodule