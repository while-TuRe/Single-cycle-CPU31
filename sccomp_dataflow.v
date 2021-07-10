`timescale 1ns / 1ps

module sccomp_dataflow(
    input clk_in,
    input reset,
    output [31:0] inst,
    output [31:0] pc
);


    wire [31:0] im_addr,result;
//    assign inst = instr;
    //assign dm_addr = (res - 32'h10010000) / 4;
    assign im_addr = pc - 32'h0040_0000;
    IMEM imemory(
       .addr(im_addr[12:2]),
       .inst(inst) // output
    );
    wire dm_w,dm_r;
    wire[31:0]dm_rdata,dm_wdata;
    wire [10:0] dm_addr;
    DMEM dmemory(
       .clk(clk_in), .ena(dm_w|dm_r), .DM_W(dm_w), .DM_R(dm_r), .DM_addr(dm_addr[10:0]), 
       .DM_wdata(dm_wdata), .DM_rdata(dm_rdata)
    );
    CPU31 sccpu(
       .clk(clk_in), .ena(1'b1),.rst(reset), .IM_inst(inst), 
       .PC(pc), .DM_W(dm_w), .DM_R(dm_r), .DM_rdata(dm_rdata),
       .DM_wdata(dm_wdata), .DM_addr(dm_addr)
    );

endmodule
