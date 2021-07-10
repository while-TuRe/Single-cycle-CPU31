`timescale 1ns / 1ps
module DMEM(
    input clk,
    input ena,
    input DM_W,
    input DM_R,
    input [10:0] DM_addr,
    input [31:0] DM_wdata,
    output [31:0] DM_rdata
    );
    
    reg [31:0] D_mem[0:31];
    //always @(posedge DM_W or DM_addr or DM_wdata) begin
    always @(posedge clk) begin
        if (DM_W && ena) begin
            D_mem[DM_addr] <= DM_wdata;
        end
    end
    
    assign DM_rdata = (DM_R && ena) ? D_mem[DM_addr] : 32'bz;
endmodule

