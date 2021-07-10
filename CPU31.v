`timescale 1ns / 1ps
//`define AKL  1
module CPU31(
    input clk,
    input ena,
    input rst,
    input [31:0] IM_inst,
    output reg [31:0]PC, //指令位
    //Data Memory 参数
    output DM_W,
    output DM_R,
    input [31:0] DM_rdata,
    output [31:0] DM_wdata,
    output [10:0] DM_addr

   // output [31:0] ALU_out
    );
    
    //指令类型
    wire [5:0]OP,func;
    assign OP = IM_inst[31:26];//RIJ指令类型
    assign func = OP ? 6'bz : IM_inst[5:0];//R指令类型
    
    wire add_, addu_, sub_, subu_, and_, or_, xor_, nor_;
    wire slt_, sltu_, sll_, srl_, sra_, sllv_, srlv_, srav_, jr_;
    assign add_ = (OP == 0 && func == 6'b100_000);
    assign addu_ = (OP == 0 && func == 6'b100_001);
    assign sub_ = (OP == 0 && func == 6'b100_010);
    assign subu_ = (OP == 0 && func == 6'b100_011);
    assign and_ = (OP == 0 && func == 6'b100_100);
    assign or_ = (OP == 0 && func == 6'b100_101);
    assign xor_ = (OP == 0 && func == 6'b100_110);
    assign nor_ = (OP == 0 && func == 6'b100_111);
    assign slt_ = (OP == 0 && func == 6'b101_010);
    assign sltu_ = (OP == 0 && func == 6'b101_011);
    assign sll_ = (OP == 0 && func == 6'b000_000);
    assign srl_ = (OP == 0 && func == 6'b000_010);
    assign sra_ = (OP == 0 && func == 6'b000_011);
    assign sllv_ = (OP == 0 && func == 6'b000_100);
    assign srlv_ = (OP == 0 && func == 6'b000_110);
    assign srav_ = (OP == 0 && func == 6'b000_111);
    assign jr_ = (OP == 0 && func == 6'b001_000);
    
    wire addi_, addiu_, andi_, ori_, xori_, lw_, sw_, beq_, bne_;
    wire slti_, sltiu_, lui_;
    assign addi_ = (OP == 6'b001_000);
    assign addiu_ = (OP == 6'b001_001);
    assign andi_ = (OP == 6'b001_100);
    assign ori_ = (OP == 6'b001_101);
    assign xori_ = (OP == 6'b001_110);
    assign lw_ = (OP == 6'b100_011);
    assign sw_ = (OP == 6'b101_011);
    assign beq_ = (OP == 6'b000_100);
    assign bne_ = (OP == 6'b000_101);
    assign slti_ = (OP == 6'b001_010);
    assign sltiu_ = (OP == 6'b001_011);
    assign lui_ = (OP == 6'b001_111);
    wire j_,jal_;
    assign j_ = (OP == 6'b000_010);
    assign jal_ = (OP==6'b000_011);
    
    wire ZF, SF, CF, OF;    //ALU标志位
    wire [3:0] ALUC;    //ALU控制位，区分不同运算

    //从寄存器中得到新操作数
    wire [4:0] Xd, Yd, Rd;       //寄存器地址
    wire [31:0] X, Y, R;     //寄存器值
    wire RF_W;  //是否写入答案（仅时钟上升沿）
    assign RF_W = ~(sw_|jr_|j_|beq_|bne_);
    //R: R = X + Y  R = Y + shamt   I: R = X + im 
    //R: [15:11]=[25:21][20:16] or [20:16][10;6]    I:[20:16]=[25:21][15：0]
    assign Xd = IM_inst[25:21];//取出3个R型指令操作数地址
    assign Yd = IM_inst[20:16];
    assign Rd = jal_ ? 31 //jal:
                        : (OP ? IM_inst[20:16]  // I|sw
                                : IM_inst[15:11]);  //R
    wire is_shamt;//是否是shamt命令
    wire is_sign; //有符号数，进行有符号拓展
    wire [31:0]A,B,Res;//ALU的两个操作数
    assign is_shamt = sll_ | srl_ | sra_;
    assign is_sign= addi_ |addiu_| slti_ | sltiu_ ;
    assign A = is_shamt ? {23'b0,IM_inst[10:6]} //sll srl sra
                          : X;
    assign B = OP&&!beq_&&!bne_ ? (is_sign ? {{16{IM_inst[15]}},IM_inst[15:0]} 
                                            : {16'b0,IM_inst[15:0]})
                                  :Y;
    assign R = lw_ ? DM_rdata : 
                jal_ ? PC+4 : 
                Res;//在读取到指令后执行，PC在下降沿变化
    assign ALUC[0] = subu_ | sub_| beq_|bne_| or_ |ori_| nor_ | slt_ | slti_ | srl_ | srlv_;
    assign ALUC[1] = add_ | addi_ | sub_ | beq_|bne_| xor_ | xori_ | nor_ | slt_ | slti_ | sltu_ | sltiu_ | sll_ | sllv_;
    assign ALUC[2] = and_ | andi_ | or_ | ori_ |xor_ | xori_ |nor_ | sra_|srav_ | sll_ |sllv_|srl_|srlv_;
    assign ALUC[3] = lui_|slt_|slti_|sltu_|sltiu_|sra_|srav_|sll_|sllv_|srl_|srlv_;

//data memory 操作
    assign DM_W = sw_;
    assign DM_R = lw_;
    assign DM_wdata = Y; 
    assign DM_addr = (X + {{16{IM_inst[15]}},IM_inst[15:0]}- 32'h10010000)/4;
    
    always@(negedge clk or posedge rst)
    begin
        if (rst) begin
           PC <= 32'h00400000;
        end
        else if (jr_)begin
            PC<=X;
        end else if (j_|jal_)begin
        // PC <- (PC+4)[31..28],address,0,0   ；address=10000/4
            PC = PC+4;
            PC = {PC[31:28],IM_inst[25:0],2'b0};
        end else if((beq_&&(Res==0)) || (bne_ && (Res!=0)))begin
        // if (rs == rt) PC <- PC+4 + (sign-extend)immediate<<2 
            PC <= PC+4 + {{14{IM_inst[15]}},IM_inst[15:0],2'b0};
        end else begin
            PC <= PC+4;
        end
    end
    RegFile cpu_ref(
        .RF_ena(ena),
        .RF_rst(rst),
        .RF_clk(clk),
        .Rsc(Xd),
        .Rtc(Yd),
        .Rdc(Rd),
        .Rs(X),
        .Rt(Y),
        .Rd(R),
        .RF_W(RF_W)
    );
    
    
    ALU alu(
        .a(A),
        .b(B),
        .r(Res),
        .aluc(ALUC),
        .zero(ZF),  // 0 is true, others are false
        .carry(CF), // unsigned cf
        .negative(SF), // 1 is false, 0 is true
        .overflow(OF) // signed of
    );
    
endmodule

