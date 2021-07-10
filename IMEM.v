`timescale 1ns / 1ps
module IMEM(
    input [10:0] addr,
    output [31:0] inst
    );
    
    dist_mem_gen_0 instr_mem(
        .a(addr),
        .spo(inst)
    );
endmodule
